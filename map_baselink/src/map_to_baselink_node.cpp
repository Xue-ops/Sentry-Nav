#include <memory>
#include <string>
#include <chrono>
#include <algorithm>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"

class MapToBaseLinkNode : public rclcpp::Node
{
public:
  MapToBaseLinkNode()
  : Node("map_to_baselink_node")
  {
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    odom_nav_frame_ = this->declare_parameter<std::string>("odom_nav_frame", "odom_nav");
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "odin1_base_link");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/map_base_pose");
    publish_tf_ = this->declare_parameter<bool>("publish_tf", true);
    if_nav2_ = this->declare_parameter<bool>("if_nav2", true);
    if_on_robot_ = this->declare_parameter<bool>("if_on_robot", true);
    tf_pub_frame_ = this->declare_parameter<std::string>("tf_child_frame", "map_corrected_base_link");
    odin1_camera_frame_ = this->declare_parameter<std::string>("odin1_camera_frame", "odin1_camera_link");
    mid360_camera_frame_ = this->declare_parameter<std::string>("mid360_camera_frame", "livox_frame");
    publish_rate_ = this->declare_parameter<double>("publish_rate", 20.0);

    auto make_transform_param = [this](
      const std::string & param_name,
      const std::string & child_frame,
      const std::vector<double> & default_value) {
        auto values = this->declare_parameter<std::vector<double>>(param_name, default_value);
        if (values.size() != 7) {
          RCLCPP_WARN(
            this->get_logger(),
            "%s must be [x, y, z, qx, qy, qz, qw], using default value",
            param_name.c_str());
          values = default_value;
        }

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.frame_id = tf_pub_frame_;
        tf_msg.child_frame_id = child_frame;
        tf_msg.transform.translation.x = values[0];
        tf_msg.transform.translation.y = values[1];
        tf_msg.transform.translation.z = values[2];
        tf_msg.transform.rotation.x = values[3];
        tf_msg.transform.rotation.y = values[4];
        tf_msg.transform.rotation.z = values[5];
        tf_msg.transform.rotation.w = values[6];
        return tf_msg;
      };

    tf_baselink_odin1_ = make_transform_param(
      "tf_baselink_odin1",
      odin1_camera_frame_,
      {-0.02897, 0.11458, 0.24799, 0.0, 0.0, 0.7071068, 0.7071068});

    tf_baselink_mid360_ = make_transform_param(
      "tf_baselink_mid360",
      mid360_camera_frame_,
      {0.0025, -0.13509, 0.29325, 0.0, 0.382684, -0.923879, 0.0});

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(),
      this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(output_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&MapToBaseLinkNode::timerCallback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Started. odom_frame=%s, map_frame=%s, base_frame=%s, output_topic=%s, publish_tf=%s",
      odom_frame_.c_str(),
      map_frame_.c_str(),
      base_frame_.c_str(),
      output_topic_.c_str(),
      publish_tf_ ? "true" : "false");
  }

private:
  std::string odom_frame_;
  std::string map_frame_;
  std::string base_frame_;
  std::string output_topic_;
  std::string odom_nav_frame_;
  bool publish_tf_;
  bool if_nav2_;
  bool if_on_robot_;
  std::string tf_pub_frame_;
  std::string odin1_camera_frame_;
  std::string mid360_camera_frame_;
  double publish_rate_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  geometry_msgs::msg::TransformStamped tf_baselink_odin1_;
  geometry_msgs::msg::TransformStamped tf_baselink_mid360_;
  void timerCallback()
  {
    geometry_msgs::msg::TransformStamped tf_odom_map;
    geometry_msgs::msg::TransformStamped tf_odom_base;

    try {
      // 正变换：map -> odom
      tf_odom_map = tf_buffer_->lookupTransform(
        map_frame_,
        odom_frame_,
        tf2::TimePointZero);

      // 正变换：odom -> base
      tf_odom_base = tf_buffer_->lookupTransform(
        odom_frame_,
        base_frame_, 
        tf2::TimePointZero);

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Cannot lookup transforms: %s",
        ex.what());
      return;
    }

    // 转成 tf2::Transform
    tf2::Transform T_odom_map, T_odom_base;
    tf2::fromMsg(tf_odom_map.transform, T_odom_map);
    tf2::fromMsg(tf_odom_base.transform, T_odom_base);

    // 按你要求：直接把 odom->map 实施到 base_link 上
    tf2::Transform T_result = T_odom_map * T_odom_base;


    // 发布 PoseStamped
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = odom_frame_;
    pose_msg.pose.position.x = T_result.getOrigin().x();
    pose_msg.pose.position.y = T_result.getOrigin().y();
    pose_msg.pose.position.z = T_result.getOrigin().z();
    pose_msg.pose.orientation = tf2::toMsg(T_result.getRotation());
    pose_pub_->publish(pose_msg);

    // 可选发布 TF
    if (publish_tf_) {
      //if on robot, the baselink need transformed by the camera to baselink transform
      if (if_on_robot_) {
        tf2::Transform T_baselink_odin1;
        tf2::fromMsg(tf_baselink_odin1_.transform, T_baselink_odin1);
        const tf2::Transform T_baselink = T_result * T_baselink_odin1.inverse();

        geometry_msgs::msg::TransformStamped tf_baselink;
        tf_baselink.header.stamp = this->now();
        tf_baselink.header.frame_id = odom_nav_frame_;
        tf_baselink.child_frame_id = tf_pub_frame_;
        tf_baselink.transform.translation.x = T_baselink.getOrigin().x();
        tf_baselink.transform.translation.y = T_baselink.getOrigin().y();
        tf_baselink.transform.translation.z = T_baselink.getOrigin().z();
        tf_baselink.transform.rotation = tf2::toMsg(T_baselink.getRotation());

        geometry_msgs::msg::TransformStamped tf_odin1;
        tf_odin1.header.stamp = this->now();
        tf_odin1.header.frame_id = tf_pub_frame_;
        tf_odin1.child_frame_id = odin1_camera_frame_;
        tf_odin1.transform = tf_baselink_odin1_.transform;

        geometry_msgs::msg::TransformStamped tf_mid360;
        tf_mid360.header.stamp = this->now();
        tf_mid360.header.frame_id = tf_pub_frame_;
        tf_mid360.child_frame_id = mid360_camera_frame_;
        tf_mid360.transform = tf_baselink_mid360_.transform;

        tf_broadcaster_->sendTransform(
          std::vector<geometry_msgs::msg::TransformStamped>{
            tf_baselink,
            tf_odin1,
            tf_mid360});

      } else {
        geometry_msgs::msg::TransformStamped tf_out;
        tf_out.header.stamp = this->now();
        tf_out.header.frame_id = odom_nav_frame_;
        tf_out.child_frame_id = tf_pub_frame_;
        tf_out.transform.translation.x = T_result.getOrigin().x();
        tf_out.transform.translation.y = T_result.getOrigin().y();
        tf_out.transform.translation.z = T_result.getOrigin().z();
        tf_out.transform.rotation = tf2::toMsg(T_result.getRotation());
        tf_broadcaster_->sendTransform(tf_out);
      }
      
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapToBaseLinkNode>());
  rclcpp::shutdown();
  return 0;
}

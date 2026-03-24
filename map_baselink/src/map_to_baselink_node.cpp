#include <memory>
#include <string>
#include <chrono>
#include <algorithm>

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
    tf_pub_frame_ = this->declare_parameter<std::string>("tf_child_frame", "map_corrected_base_link");
    publish_rate_ = this->declare_parameter<double>("publish_rate", 20.0);

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
  std::string tf_pub_frame_;
  double publish_rate_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

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
      geometry_msgs::msg::TransformStamped tf_out;
      tf_out.header.stamp = this->now();
      tf_out.header.frame_id = odom_frame_;
      tf_out.child_frame_id = tf_pub_frame_;
      tf_out.transform.translation.x = T_result.getOrigin().x();
      tf_out.transform.translation.y = T_result.getOrigin().y();
      tf_out.transform.translation.z = T_result.getOrigin().z();
      tf_out.transform.rotation = tf2::toMsg(T_result.getRotation());
      tf_broadcaster_->sendTransform(tf_out);
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
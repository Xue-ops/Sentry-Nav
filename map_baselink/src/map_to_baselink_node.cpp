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

class MapToBaseLinkNode : public rclcpp::Node
{
public:
  MapToBaseLinkNode()
  : Node("map_to_baselink_node")
  {
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "odin1_base_link");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/map_base_pose");
    publish_tf_ = this->declare_parameter<bool>("publish_tf", false);
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

    RCLCPP_INFO(this->get_logger(),
      "Started. map_frame=%s, base_frame=%s, output_topic=%s, publish_tf=%s",
      map_frame_.c_str(),
      base_frame_.c_str(),
      output_topic_.c_str(),
      publish_tf_ ? "true" : "false");
  }

private:
  std::string map_frame_;
  std::string base_frame_;
  std::string output_topic_;
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
    geometry_msgs::msg::TransformStamped tf_map_base;

    try {
      // 直接让 tf2 算 map -> base
      tf_map_base = tf_buffer_->lookupTransform(
        map_frame_,
        base_frame_,
        tf2::TimePointZero);

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Cannot lookup %s -> %s: %s",
        map_frame_.c_str(),
        base_frame_.c_str(),
        ex.what());
      return;
    }

    // 发布 PoseStamped
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = tf_map_base.header.stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose.position.x = tf_map_base.transform.translation.x;
    pose_msg.pose.position.y = tf_map_base.transform.translation.y;
    pose_msg.pose.position.z = tf_map_base.transform.translation.z;
    pose_msg.pose.orientation = tf_map_base.transform.rotation;
    pose_pub_->publish(pose_msg);

    // 可选发布 TF
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf_out;
      tf_out.header.stamp = tf_map_base.header.stamp;
      tf_out.header.frame_id = map_frame_;
      tf_out.child_frame_id = tf_pub_frame_;
      tf_out.transform = tf_map_base.transform;
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
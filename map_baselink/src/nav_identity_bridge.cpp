#include <memory>
#include <string>
#include <chrono>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

class NavIdentityBridge : public rclcpp::Node
{
public:
  NavIdentityBridge()
  : Node("nav_identity_bridge")
  {
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    nav_map_frame_ = this->declare_parameter<std::string>("nav_map_frame", "map_nav");
    nav_odom_frame_ = this->declare_parameter<std::string>("nav_odom_frame", "odom_nav");
    publish_rate_ = this->declare_parameter<double>("publish_rate", 20.0);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    publishStaticMapBridge();

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&NavIdentityBridge::timerCallback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Started. publishing identity TFs: %s -> %s (static), %s -> %s (dynamic)",
      map_frame_.c_str(),
      nav_map_frame_.c_str(),
      nav_map_frame_.c_str(),
      nav_odom_frame_.c_str());
  }

private:
  std::string map_frame_;
  std::string nav_map_frame_;
  std::string nav_odom_frame_;
  double publish_rate_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  geometry_msgs::msg::TransformStamped makeIdentityTransform(
    const std::string & parent_frame,
    const std::string & child_frame) const
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->now();
    tf_msg.header.frame_id = parent_frame;
    tf_msg.child_frame_id = child_frame;

    tf_msg.transform.translation.x = 0.0;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.0;

    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = 0.0;
    tf_msg.transform.rotation.w = 1.0;

    return tf_msg;
  }

  void publishStaticMapBridge()
  {
    static_tf_broadcaster_->sendTransform(makeIdentityTransform(map_frame_, nav_map_frame_));
  }

  void timerCallback()
  {
    tf_broadcaster_->sendTransform(makeIdentityTransform(nav_map_frame_, nav_odom_frame_));
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavIdentityBridge>());
  rclcpp::shutdown();
  return 0;
}

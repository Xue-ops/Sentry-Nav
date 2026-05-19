#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

class FakeOdomMap : public rclcpp::Node
{
public:
  FakeOdomMap()
  : Node("fake_odom_map")
  {
    parent_frame_ = this->declare_parameter<std::string>("parent_frame", "odom");
    child_frame_ = this->declare_parameter<std::string>("child_frame", "map");

    static_tf_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    publishStaticIdentityTf();

    RCLCPP_INFO(
      this->get_logger(),
      "Published static fake TF: %s -> %s identity",
      parent_frame_.c_str(),
      child_frame_.c_str()
    );
  }

private:
  std::string parent_frame_;
  std::string child_frame_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  void publishStaticIdentityTf()
  {
    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = this->now();
    tf_msg.header.frame_id = parent_frame_;
    tf_msg.child_frame_id = child_frame_;

    // 零平移
    tf_msg.transform.translation.x = 0.0;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.0;

    // 零旋转：identity quaternion
    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = 0.0;
    tf_msg.transform.rotation.w = 1.0;

    static_tf_broadcaster_->sendTransform(tf_msg);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FakeOdomMap>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

class CloudFrameTransformer : public rclcpp::Node
{
public:
  CloudFrameTransformer()
  : Node("cloud_frame_transformer"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/odin1/cloud_slam");
    output_topic_ = declare_parameter<std::string>("output_topic", "/cloud_nav2");
    target_frame_ = declare_parameter<std::string>("target_frame", "odin1_base_link");

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&CloudFrameTransformer::cloudCallback, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_,
      rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(), "Transforming %s -> %s -> %s",
      input_topic_.c_str(), target_frame_.c_str(), output_topic_.c_str());
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    try {
      auto tf = tf_buffer_.lookupTransform(
        target_frame_,
        msg->header.frame_id,
        msg->header.stamp,
        rclcpp::Duration::from_seconds(0.1));

      sensor_msgs::msg::PointCloud2 out_cloud;
      tf2::doTransform(*msg, out_cloud, tf);
      out_cloud.header.frame_id = target_frame_;
      pub_->publish(out_cloud);

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Transform failed from %s to %s: %s",
        msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudFrameTransformer>());
  rclcpp::shutdown();
  return 0;
}
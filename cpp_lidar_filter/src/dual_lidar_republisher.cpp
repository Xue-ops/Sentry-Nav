#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class DualLidarRepublisher : public rclcpp::Node
{
public:
  DualLidarRepublisher()
  : Node("dual_lidar_republisher")
  {
    declare_parameter("front_input_topic", "/livox/lidar_192_168_1_188");
    declare_parameter("front_output_topic", "/mid360_front/lidar");
    declare_parameter("front_frame_id", "mid360_front_frame");
    declare_parameter("back_input_topic", "/livox/lidar_192_168_1_182");
    declare_parameter("back_output_topic", "/mid360_back/lidar");
    declare_parameter("back_frame_id", "mid360_back_frame");

    const auto front_input_topic = get_parameter("front_input_topic").as_string();
    const auto front_output_topic = get_parameter("front_output_topic").as_string();
    const auto front_frame_id = get_parameter("front_frame_id").as_string();
    const auto back_input_topic = get_parameter("back_input_topic").as_string();
    const auto back_output_topic = get_parameter("back_output_topic").as_string();
    const auto back_frame_id = get_parameter("back_frame_id").as_string();

    const auto qos = rclcpp::SensorDataQoS();
    front_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(front_output_topic, qos);
    back_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(back_output_topic, qos);

    front_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      front_input_topic, qos,
      [this, front_frame_id](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        auto output = *message;
        output.header.frame_id = front_frame_id;
        front_publisher_->publish(output);
      });

    back_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      back_input_topic, qos,
      [this, back_frame_id](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        auto output = *message;
        output.header.frame_id = back_frame_id;
        back_publisher_->publish(output);
      });

    RCLCPP_INFO(
      get_logger(), "Front: %s -> %s (frame_id=%s)",
      front_input_topic.c_str(), front_output_topic.c_str(), front_frame_id.c_str());
    RCLCPP_INFO(
      get_logger(), "Back: %s -> %s (frame_id=%s)",
      back_input_topic.c_str(), back_output_topic.c_str(), back_frame_id.c_str());
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr front_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr back_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr front_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr back_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DualLidarRepublisher>());
  rclcpp::shutdown();
  return 0;
}

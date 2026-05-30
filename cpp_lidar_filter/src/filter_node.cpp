#include <memory>
#include <string>
#include <functional>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

class LidarFilterNode : public rclcpp::Node
{
public:
  LidarFilterNode()
  : Node("lidar_filter_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    this->declare_parameter("input_topic", "/cloud_nav2_raw");
    this->declare_parameter("output_topic", "/cloud_nav2");

    // 裁剪盒所在的 TF frame
    // 比如 base_link / odin1_base_link
    this->declare_parameter("crop_frame", "map_corrected_base_link");

    // 裁剪参数：这些范围现在是相对于 crop_frame 的
    this->declare_parameter("min_x", -0.3);
    this->declare_parameter("max_x", 0.3);
    this->declare_parameter("min_y", -0.3);
    this->declare_parameter("max_y", 0.3);
    this->declare_parameter("min_z", -0.1);
    this->declare_parameter("max_z", 0.6);
    this->declare_parameter("negative", true);  // true = 挖掉盒子内部

    // 降采样参数
    this->declare_parameter("use_voxel", false);
    this->declare_parameter("leaf_size", 0.05);

    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    crop_frame_ = this->get_parameter("crop_frame").as_string();

    RCLCPP_INFO(this->get_logger(), "Listening on: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Crop box frame: %s", crop_frame_.c_str());

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&LidarFilterNode::cloud_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_,
      10);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "crop_box_marker",
      10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&LidarFilterNode::publish_marker, this));
  }

private:
  void publish_marker()
  {
    double min_x = this->get_parameter("min_x").as_double();
    double max_x = this->get_parameter("max_x").as_double();
    double min_y = this->get_parameter("min_y").as_double();
    double max_y = this->get_parameter("max_y").as_double();
    double min_z = this->get_parameter("min_z").as_double();
    double max_z = this->get_parameter("max_z").as_double();

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = crop_frame_;
    marker.header.stamp = this->now();

    marker.ns = "vehicle_body";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = (max_x + min_x) / 2.0;
    marker.pose.position.y = (max_y + min_y) / 2.0;
    marker.pose.position.z = (max_z + min_z) / 2.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = max_x - min_x;
    marker.scale.y = max_y - min_y;
    marker.scale.z = max_z - min_z;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.4;

    marker_pub_->publish(marker);
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    try {
      // 1. 把输入点云从 msg->header.frame_id 转到 crop_frame_
      auto tf_cloud_to_crop = tf_buffer_.lookupTransform(
        crop_frame_,
        msg->header.frame_id,
        rclcpp::Time(0),
        rclcpp::Duration::from_seconds(0.1));

      sensor_msgs::msg::PointCloud2 cloud_in_crop_frame;
      tf2::doTransform(*msg, cloud_in_crop_frame, tf_cloud_to_crop);
      cloud_in_crop_frame.header.frame_id = crop_frame_;

      // 2. 转成 PCL，在 crop_frame_ 下裁剪
      pcl::PCLPointCloud2::Ptr cloud_in(new pcl::PCLPointCloud2);
      pcl_conversions::toPCL(cloud_in_crop_frame, *cloud_in);

      pcl::CropBox<pcl::PCLPointCloud2> crop;
      crop.setInputCloud(cloud_in);

      Eigen::Vector4f min_pt, max_pt;
      min_pt << this->get_parameter("min_x").as_double(),
                this->get_parameter("min_y").as_double(),
                this->get_parameter("min_z").as_double(),
                1.0;

      max_pt << this->get_parameter("max_x").as_double(),
                this->get_parameter("max_y").as_double(),
                this->get_parameter("max_z").as_double(),
                1.0;

      crop.setMin(min_pt);
      crop.setMax(max_pt);
      crop.setNegative(this->get_parameter("negative").as_bool());

      pcl::PCLPointCloud2::Ptr cloud_cropped(new pcl::PCLPointCloud2);
      crop.filter(*cloud_cropped);

      // 3. 可选降采样
      pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);

      bool use_voxel = this->get_parameter("use_voxel").as_bool();
      if (use_voxel) {
        pcl::VoxelGrid<pcl::PCLPointCloud2> voxel;
        voxel.setInputCloud(cloud_cropped);

        float leaf_size =
          static_cast<float>(this->get_parameter("leaf_size").as_double());

        voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel.filter(*cloud_filtered);
      } else {
        cloud_filtered = cloud_cropped;
      }

      sensor_msgs::msg::PointCloud2 output_in_crop_frame;
      pcl_conversions::fromPCL(*cloud_filtered, output_in_crop_frame);
      output_in_crop_frame.header = cloud_in_crop_frame.header;

      // 4. 再从 crop_frame_ 转回原始点云 frame
      auto tf_crop_to_cloud = tf_buffer_.lookupTransform(
        msg->header.frame_id,
        crop_frame_,
        rclcpp::Time(0),
        rclcpp::Duration::from_seconds(0.1));

      sensor_msgs::msg::PointCloud2 output;
      tf2::doTransform(output_in_crop_frame, output, tf_crop_to_cloud);

      output.header.stamp = msg->header.stamp;
      output.header.frame_id = msg->header.frame_id;

      pub_->publish(output);

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "TF failed for crop_frame [%s] and cloud frame [%s]: %s",
        crop_frame_.c_str(),
        msg->header.frame_id.c_str(),
        ex.what());
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string crop_frame_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarFilterNode>());
  rclcpp::shutdown();
  return 0;
}
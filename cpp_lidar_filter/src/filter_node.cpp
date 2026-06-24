#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <memory>
#include <string>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "tf2/exceptions.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

class LidarFilterNode : public rclcpp::Node
{
public:
  LidarFilterNode()
  : Node("lidar_filter_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    this->declare_parameter("input_topic", "/mid360_front/lidar");
    this->declare_parameter("output_topic", "/mid360_front/lidar_filtered");

    this->declare_parameter("input_topic_2", "/odin1/cloud_slam_filter");
    this->declare_parameter("output_topic_2", "/odin1/cloud_slam_filtered");

    this->declare_parameter("crop_frame", "base_link");
    this->declare_parameter("z_filter_frame", "base_link");
    this->declare_parameter("z_threshold", 100.0);

    // 直接在点云自己的 frame 里切 z
    // 例如 frame_id = mid360_front_frame，就按 mid360_front_frame 的 z 删除点
    this->declare_parameter("use_cloud_frame_z_cut", true);
    this->declare_parameter("cloud_frame_remove_z_min", 0.0);
    this->declare_parameter("cloud_frame_remove_z_max", 0.1);

    // 裁剪参数 CropBox
    this->declare_parameter("min_x", -0.3);
    this->declare_parameter("max_x", 0.3);
    this->declare_parameter("min_y", -0.3);
    this->declare_parameter("max_y", 0.3);
    this->declare_parameter("min_z", -0.1);
    this->declare_parameter("max_z", 0.6);
    this->declare_parameter("negative", true);  // true = 挖掉中间车体

    // 降采样参数
    this->declare_parameter("use_voxel", false);
    this->declare_parameter("leaf_size", 0.05);

    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    std::string input_topic_2 = this->get_parameter("input_topic_2").as_string();
    std::string output_topic_2 = this->get_parameter("output_topic_2").as_string();

    crop_frame_ = this->get_parameter("crop_frame").as_string();
    z_filter_frame_ = this->get_parameter("z_filter_frame").as_string();

    RCLCPP_INFO(this->get_logger(), "Listening on: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Listening on: %s", input_topic_2.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic_2.c_str());
    RCLCPP_INFO(this->get_logger(), "Crop box frame: %s", crop_frame_.c_str());

    RCLCPP_INFO(
      this->get_logger(),
      "Cloud-frame z cut: %s, remove %.3f <= z <= %.3f in original cloud frame",
      this->get_parameter("use_cloud_frame_z_cut").as_bool() ? "enabled" : "disabled",
      this->get_parameter("cloud_frame_remove_z_min").as_double(),
      this->get_parameter("cloud_frame_remove_z_max").as_double());

    RCLCPP_INFO(
      this->get_logger(),
      "Livox z filter frame: %s, keep z <= %.3f",
      z_filter_frame_.c_str(),
      this->get_parameter("z_threshold").as_double());

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, 10);

    pub2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_2, 10);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "crop_box_marker", 10);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        cloud_callback(msg, pub_, true);
      });

    sub2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_2,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        cloud_callback(msg, pub2_, false);
      });

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&LidarFilterNode::publish_marker, this));
  }

private:
  bool getFloat32FieldOffset(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const std::string & field_name,
    size_t & offset) const
  {
    for (const auto & field : cloud.fields) {
      if (field.name == field_name &&
        field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
        field.count >= 1)
      {
        offset = field.offset;
        return true;
      }
    }
    return false;
  }

  sensor_msgs::msg::PointCloud2 cutZRangeInOwnFrame(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const double remove_z_min,
    const double remove_z_max) const
  {
    size_t z_offset = 0;

    if (!getFloat32FieldOffset(cloud, "z", z_offset)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Cannot apply cloud-frame z cut: point cloud field z must be FLOAT32");
      return cloud;
    }

    if (z_offset + sizeof(float) > cloud.point_step) {
      RCLCPP_WARN(
        this->get_logger(),
        "Cannot apply cloud-frame z cut: z offset exceeds point_step");
      return cloud;
    }

    sensor_msgs::msg::PointCloud2 filtered = cloud;
    filtered.height = 1;
    filtered.width = 0;
    filtered.row_step = 0;
    filtered.is_dense = false;

    filtered.data.resize(
      static_cast<size_t>(cloud.width) *
      static_cast<size_t>(cloud.height) *
      cloud.point_step);

    size_t kept_points = 0;

    for (uint32_t row = 0; row < cloud.height; ++row) {
      const size_t row_start = static_cast<size_t>(row) * cloud.row_step;

      for (uint32_t col = 0; col < cloud.width; ++col) {
        const size_t point_start =
          row_start + static_cast<size_t>(col) * cloud.point_step;

        if (point_start + cloud.point_step > cloud.data.size()) {
          continue;
        }

        float z = 0.0f;
        std::memcpy(&z, &cloud.data[point_start + z_offset], sizeof(float));

        if (!std::isfinite(z)) {
          continue;
        }

        // 直接按点云自己的 frame 删除 z 范围
        // 例如 mid360_front_frame 下 z=0.0~0.2 的点全部不要
        const bool remove_this_point =
          (z >= remove_z_min && z <= remove_z_max);

        if (!remove_this_point) {
          const size_t output_start = kept_points * cloud.point_step;
          std::memcpy(
            &filtered.data[output_start],
            &cloud.data[point_start],
            cloud.point_step);
          ++kept_points;
        }
      }
    }

    filtered.width = static_cast<uint32_t>(kept_points);
    filtered.row_step = filtered.width * filtered.point_step;
    filtered.data.resize(filtered.row_step);

    return filtered;
  }

  void filterByZPlaneInPlace(
    sensor_msgs::msg::PointCloud2 & cloud,
    const geometry_msgs::msg::TransformStamped & tf_crop_to_z_filter,
    const double z_threshold) const
  {
    size_t x_offset = 0;
    size_t y_offset = 0;
    size_t z_offset = 0;

    if (!getFloat32FieldOffset(cloud, "x", x_offset) ||
      !getFloat32FieldOffset(cloud, "y", y_offset) ||
      !getFloat32FieldOffset(cloud, "z", z_offset))
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Cannot apply z plane filter: point cloud fields x/y/z must be FLOAT32");
      return;
    }

    if (x_offset + sizeof(float) > cloud.point_step ||
      y_offset + sizeof(float) > cloud.point_step ||
      z_offset + sizeof(float) > cloud.point_step)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Cannot apply z plane filter: point cloud x/y/z offsets exceed point_step");
      return;
    }

    tf2::Transform T_z_filter_crop;
    T_z_filter_crop.setOrigin(tf2::Vector3(
      tf_crop_to_z_filter.transform.translation.x,
      tf_crop_to_z_filter.transform.translation.y,
      tf_crop_to_z_filter.transform.translation.z));

    T_z_filter_crop.setRotation(tf2::Quaternion(
      tf_crop_to_z_filter.transform.rotation.x,
      tf_crop_to_z_filter.transform.rotation.y,
      tf_crop_to_z_filter.transform.rotation.z,
      tf_crop_to_z_filter.transform.rotation.w));

    const tf2::Vector3 z_axis_in_crop = T_z_filter_crop.getBasis().getRow(2);
    const double plane_offset = T_z_filter_crop.getOrigin().z() - z_threshold;

    sensor_msgs::msg::PointCloud2 filtered = cloud;
    filtered.height = 1;
    filtered.width = 0;
    filtered.row_step = 0;
    filtered.is_dense = false;

    filtered.data.resize(
      static_cast<size_t>(cloud.width) *
      static_cast<size_t>(cloud.height) *
      cloud.point_step);

    size_t kept_points = 0;

    for (uint32_t row = 0; row < cloud.height; ++row) {
      const size_t row_start = static_cast<size_t>(row) * cloud.row_step;

      for (uint32_t col = 0; col < cloud.width; ++col) {
        const size_t point_start = row_start + static_cast<size_t>(col) * cloud.point_step;

        if (point_start + cloud.point_step > cloud.data.size()) {
          continue;
        }

        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;

        std::memcpy(&x, &cloud.data[point_start + x_offset], sizeof(float));
        std::memcpy(&y, &cloud.data[point_start + y_offset], sizeof(float));
        std::memcpy(&z, &cloud.data[point_start + z_offset], sizeof(float));

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }

        const double z_in_filter_frame =
          z_axis_in_crop.x() * x +
          z_axis_in_crop.y() * y +
          z_axis_in_crop.z() * z +
          plane_offset;

        if (z_in_filter_frame <= 0.0) {
          const size_t output_start = kept_points * cloud.point_step;
          std::memcpy(
            &filtered.data[output_start],
            &cloud.data[point_start],
            cloud.point_step);
          ++kept_points;
        }
      }
    }

    filtered.width = static_cast<uint32_t>(kept_points);
    filtered.row_step = filtered.width * filtered.point_step;
    filtered.data.resize(filtered.row_step);

    cloud = std::move(filtered);
  }

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

  void cloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
    const bool apply_z_filter)
  {
    crop_frame_ = this->get_parameter("crop_frame").as_string();
    z_filter_frame_ = this->get_parameter("z_filter_frame").as_string();

    try {
      sensor_msgs::msg::PointCloud2 cloud_before_tf;

      if (this->get_parameter("use_cloud_frame_z_cut").as_bool()) {
        cloud_before_tf = cutZRangeInOwnFrame(
          *msg,
          this->get_parameter("cloud_frame_remove_z_min").as_double(),
          this->get_parameter("cloud_frame_remove_z_max").as_double());
      } else {
        cloud_before_tf = *msg;
      }

      auto tf_cloud_to_crop = tf_buffer_.lookupTransform(
        crop_frame_,
        msg->header.frame_id,
        rclcpp::Time(0),
        rclcpp::Duration::from_seconds(0.1));

      sensor_msgs::msg::PointCloud2 cloud_in_crop_frame;
      tf2::doTransform(cloud_before_tf, cloud_in_crop_frame, tf_cloud_to_crop);
      cloud_in_crop_frame.header.frame_id = crop_frame_;

      pcl::PCLPointCloud2::Ptr cloud_in(new pcl::PCLPointCloud2);
      pcl_conversions::toPCL(cloud_in_crop_frame, *cloud_in);

      // CropBox 裁剪：去除车身
      pcl::CropBox<pcl::PCLPointCloud2> crop;
      crop.setInputCloud(cloud_in);

      Eigen::Vector4f min_pt;
      Eigen::Vector4f max_pt;

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

      pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);

      if (this->get_parameter("use_voxel").as_bool()) {
        pcl::VoxelGrid<pcl::PCLPointCloud2> voxel;
        voxel.setInputCloud(cloud_cropped);

        float leaf_size = static_cast<float>(
          this->get_parameter("leaf_size").as_double());

        voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel.filter(*cloud_filtered);
      } else {
        cloud_filtered = cloud_cropped;
      }

      sensor_msgs::msg::PointCloud2 output_in_crop_frame;
      pcl_conversions::fromPCL(*cloud_filtered, output_in_crop_frame);
      output_in_crop_frame.header = cloud_in_crop_frame.header;

      if (apply_z_filter) {
        auto tf_crop_to_z_filter = tf_buffer_.lookupTransform(
          z_filter_frame_,
          crop_frame_,
          rclcpp::Time(0),
          rclcpp::Duration::from_seconds(0.1));

        filterByZPlaneInPlace(
          output_in_crop_frame,
          tf_crop_to_z_filter,
          this->get_parameter("z_threshold").as_double());
      }

      auto tf_crop_to_cloud = tf_buffer_.lookupTransform(
        msg->header.frame_id,
        crop_frame_,
        rclcpp::Time(0),
        rclcpp::Duration::from_seconds(0.1));

      sensor_msgs::msg::PointCloud2 output;
      tf2::doTransform(output_in_crop_frame, output, tf_crop_to_cloud);

      output.header.stamp = this->now();
      output.header.frame_id = msg->header.frame_id;

      publisher->publish(output);

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

  std::string crop_frame_;
  std::string z_filter_frame_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub2_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub2_;

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
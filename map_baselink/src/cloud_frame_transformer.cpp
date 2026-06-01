#include <memory>
#include <string>
#include <functional>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

class CloudFrameTransformer : public rclcpp::Node
{
public:
  CloudFrameTransformer()
  : Node("cloud_frame_transformer"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/odin1/cloud_slam_filter");
    mid360_pointcloud_topic_ = declare_parameter<std::string>("mid360_pointcloud_topic", "livox/lidar");
    mid360_frame_ = declare_parameter<std::string>("mid360_frame", "livox_frame");
    output_topic_ = declare_parameter<std::string>("output_topic", "/cloud_nav2_raw");
    target_frame_ = declare_parameter<std::string>("target_frame", "odom_nav");

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&CloudFrameTransformer::cloudCallback, this, std::placeholders::_1));

    mid360_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      mid360_pointcloud_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&CloudFrameTransformer::mid360CloudCallback, this, std::placeholders::_1));

    rclcpp::QoS pub_qos(rclcpp::KeepLast(10));
    pub_qos.reliable();

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_,
      pub_qos);

    RCLCPP_INFO(get_logger(), "Transforming and merging %s + %s -> %s -> %s",
      input_topic_.c_str(), mid360_pointcloud_topic_.c_str(),
      target_frame_.c_str(), output_topic_.c_str());
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    handleCloud(msg, latest_primary_cloud_, "primary", "");
  }

  void mid360CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    handleCloud(msg, latest_mid360_cloud_, "mid360", mid360_frame_);
  }

  void handleCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    sensor_msgs::msg::PointCloud2::SharedPtr & latest_cloud,
    const char * source_name,
    const std::string & fallback_frame)
  {
    const std::string source_frame =
      msg->header.frame_id.empty() ? fallback_frame : msg->header.frame_id;

    try {
      sensor_msgs::msg::PointCloud2 source_cloud = *msg;
      source_cloud.header.frame_id = source_frame;

      auto tf = tf_buffer_.lookupTransform(
        target_frame_,
        source_frame,
        rclcpp::Time(0),
        rclcpp::Duration::from_seconds(0.1));

      sensor_msgs::msg::PointCloud2 out_cloud;
      tf2::doTransform(source_cloud, out_cloud, tf);

      const auto output_stamp = now();
      out_cloud.header.stamp = output_stamp;
      out_cloud.header.frame_id = target_frame_;

      sensor_msgs::msg::PointCloud2 merged_cloud;
      {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        latest_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(std::move(out_cloud));
        merged_cloud = buildMergedCloudLocked(*latest_cloud);
      }
      merged_cloud.header.stamp = output_stamp;

      pub_->publish(merged_cloud);

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Transform failed for %s cloud from %s to %s: %s",
        source_name, source_frame.c_str(), target_frame_.c_str(), ex.what());
    } catch (const std::runtime_error & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Merge failed for %s cloud: %s",
        source_name, ex.what());
    }
  }

  sensor_msgs::msg::PointCloud2 buildMergedCloudLocked(
    const sensor_msgs::msg::PointCloud2 & fallback_cloud) const
  {
    if (!latest_primary_cloud_) {
      return latest_mid360_cloud_ ? *latest_mid360_cloud_ : fallback_cloud;
    }
    if (!latest_mid360_cloud_) {
      return *latest_primary_cloud_;
    }

    if (cloudLayoutsMatch(*latest_primary_cloud_, *latest_mid360_cloud_)) {
      return mergeRawClouds(*latest_primary_cloud_, *latest_mid360_cloud_);
    }

    return mergeAsXyzCloud(*latest_primary_cloud_, *latest_mid360_cloud_);
  }

  static bool cloudLayoutsMatch(
    const sensor_msgs::msg::PointCloud2 & first,
    const sensor_msgs::msg::PointCloud2 & second)
  {
    return first.fields == second.fields &&
           first.is_bigendian == second.is_bigendian &&
           first.point_step == second.point_step;
  }

  static size_t pointCount(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    return static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);
  }

  static void appendRawPoints(
    const sensor_msgs::msg::PointCloud2 & source,
    std::vector<uint8_t> & data)
  {
    const auto row_points = static_cast<size_t>(source.width);
    const auto point_step = static_cast<size_t>(source.point_step);
    const auto row_step = static_cast<size_t>(source.row_step);

    for (uint32_t row = 0; row < source.height; ++row) {
      const auto row_offset = static_cast<size_t>(row) * row_step;
      const auto row_bytes = row_points * point_step;
      data.insert(
        data.end(),
        source.data.begin() + row_offset,
        source.data.begin() + row_offset + row_bytes);
    }
  }

  static sensor_msgs::msg::PointCloud2 mergeRawClouds(
    const sensor_msgs::msg::PointCloud2 & first,
    const sensor_msgs::msg::PointCloud2 & second)
  {
    sensor_msgs::msg::PointCloud2 merged = first;
    merged.header.stamp = first.header.stamp;
    merged.header.frame_id = first.header.frame_id;
    merged.height = 1;
    merged.width = static_cast<uint32_t>(pointCount(first) + pointCount(second));
    merged.row_step = merged.point_step * merged.width;
    merged.is_dense = first.is_dense && second.is_dense;
    merged.data.clear();
    merged.data.reserve(static_cast<size_t>(merged.row_step));

    appendRawPoints(first, merged.data);
    appendRawPoints(second, merged.data);
    return merged;
  }

  static sensor_msgs::msg::PointCloud2 mergeAsXyzCloud(
    const sensor_msgs::msg::PointCloud2 & first,
    const sensor_msgs::msg::PointCloud2 & second)
  {
    sensor_msgs::msg::PointCloud2 merged;
    merged.header.stamp = first.header.stamp;
    merged.header.frame_id = first.header.frame_id;
    merged.height = 1;
    merged.width = static_cast<uint32_t>(pointCount(first) + pointCount(second));
    merged.is_bigendian = false;
    merged.is_dense = first.is_dense && second.is_dense;

    sensor_msgs::PointCloud2Modifier modifier(merged);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(pointCount(first) + pointCount(second));

    sensor_msgs::PointCloud2Iterator<float> out_x(merged, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(merged, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(merged, "z");

    appendXyzPoints(first, out_x, out_y, out_z);
    appendXyzPoints(second, out_x, out_y, out_z);
    return merged;
  }

  static void appendXyzPoints(
    const sensor_msgs::msg::PointCloud2 & source,
    sensor_msgs::PointCloud2Iterator<float> & out_x,
    sensor_msgs::PointCloud2Iterator<float> & out_y,
    sensor_msgs::PointCloud2Iterator<float> & out_z)
  {
    sensor_msgs::PointCloud2ConstIterator<float> in_x(source, "x");
    sensor_msgs::PointCloud2ConstIterator<float> in_y(source, "y");
    sensor_msgs::PointCloud2ConstIterator<float> in_z(source, "z");

    for (size_t i = 0; i < pointCount(source); ++i, ++in_x, ++in_y, ++in_z, ++out_x, ++out_y, ++out_z) {
      *out_x = *in_x;
      *out_y = *in_y;
      *out_z = *in_z;
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  std::string mid360_pointcloud_topic_;
  std::string mid360_frame_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mid360_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  mutable std::mutex cloud_mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_primary_cloud_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_mid360_cloud_;

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

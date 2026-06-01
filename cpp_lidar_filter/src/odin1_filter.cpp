#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

class Odin1FilterNode : public rclcpp::Node
{
public:
  Odin1FilterNode()
  : Node("odin1_filter_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/odin1/cloud_slam");
    output_topic_ = declare_parameter<std::string>("output_topic", "/odin1/cloud_slam_filter");
    z_threshold_ = declare_parameter<double>("z_threshold", 1.0);
    leaf_size_ = declare_parameter<double>("leaf_size", 0.05);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&Odin1FilterNode::cloudCallback, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_,
      10);

    RCLCPP_INFO(
      get_logger(),
      "Filtering %s -> %s, keep z <= %.3f, voxel leaf %.3f",
      input_topic_.c_str(), output_topic_.c_str(), z_threshold_, leaf_size_);
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PCLPointCloud2::Ptr cloud_in(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*msg, *cloud_in);

    pcl::PCLPointCloud2::Ptr cloud_z_filtered(new pcl::PCLPointCloud2);
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(
      -std::numeric_limits<float>::max(),
      static_cast<float>(z_threshold_));
    pass.filter(*cloud_z_filtered);

    pcl::PCLPointCloud2::Ptr cloud_sparse(new pcl::PCLPointCloud2);
    if (leaf_size_ > 0.0) {
      const float leaf = static_cast<float>(leaf_size_);
      pcl::VoxelGrid<pcl::PCLPointCloud2> voxel;
      voxel.setInputCloud(cloud_z_filtered);
      voxel.setLeafSize(leaf, leaf, leaf);
      voxel.filter(*cloud_sparse);
    } else {
      cloud_sparse = cloud_z_filtered;
    }

    sensor_msgs::msg::PointCloud2 output;
    pcl_conversions::fromPCL(*cloud_sparse, output);
    output.header = msg->header;
    pub_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  double z_threshold_;
  double leaf_size_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odin1FilterNode>());
  rclcpp::shutdown();
  return 0;
}

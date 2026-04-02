#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <chrono>

class PCDPublisher : public rclcpp::Node
{
public:
  explicit PCDPublisher(const rclcpp::NodeOptions& options)
  : Node("pcd_publisher", options)
  {
    // ---- parameters ----
    this->declare_parameter<std::string>("pcd_file", "");
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<std::string>("topic", "/pcd_cloud");
    this->declare_parameter<double>("leaf_size", 0.03);
    this->declare_parameter<int>("exit_after_sec", 10);

    const std::string pcd_path = this->get_parameter("pcd_path").as_string();
    const std::string frame_id = this->get_parameter("frame_id").as_string();
    const std::string topic    = this->get_parameter("topic").as_string();
    const double leaf          = this->get_parameter("leaf_size").as_double();
    const int exit_after_sec   = this->get_parameter("exit_after_sec").as_int();

    if (pcd_path.empty()) {
      RCLCPP_ERROR(get_logger(),
        "pcd_path is empty.\n"
        "Example:\n"
        "  ros2 run graph_show pcd_publisher --ros-args -p pcd_path:=/abs/path/file.pcd");
      rclcpp::shutdown();
      return;
    }

    // ---- publisher (transient_local so RViz can receive even if it starts later) ----
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      topic, rclcpp::QoS(1).transient_local()
    );

    // ---- load PCD once into shared_ptr ----
    auto cloud_ptr = std::make_shared<pcl::PCLPointCloud2>();
    if (pcl::io::loadPCDFile(pcd_path, *cloud_ptr) != 0) {
      RCLCPP_ERROR(get_logger(), "Failed to load PCD: %s", pcd_path.c_str());
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(get_logger(),
      "Loaded PCD: %s | width=%u height=%u point_step=%u data=%zu bytes",
      pcd_path.c_str(), cloud_ptr->width, cloud_ptr->height,
      cloud_ptr->point_step, cloud_ptr->data.size()
    );

    // ---- voxel grid downsample ----
    pcl::PCLPointCloud2 cloud_ds;
    pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
    vox.setInputCloud(cloud_ptr);  // std::shared_ptr works with PCL 1.12
    vox.setLeafSize((float)leaf, (float)leaf, (float)leaf);
    vox.filter(cloud_ds);

    RCLCPP_INFO(get_logger(),
      "Downsampled leaf=%.3f m | new width=%u height=%u (points=%u)",
      leaf, cloud_ds.width, cloud_ds.height, cloud_ds.width * cloud_ds.height
    );

    // ---- convert to ROS2 PointCloud2 ----
    sensor_msgs::msg::PointCloud2 msg;
    pcl_conversions::fromPCL(cloud_ds, msg);
    msg.header.frame_id = frame_id;
    msg.header.stamp = this->now();

    // ---- publish once ----
    pub_->publish(msg);
    RCLCPP_INFO(get_logger(),
      "Published once: topic=%s frame=%s",
      topic.c_str(), frame_id.c_str()
    );

    // ---- exit after N seconds (so RViz has time to subscribe) ----
    exit_timer_ = this->create_wall_timer(
      std::chrono::seconds(exit_after_sec),
      []() { rclcpp::shutdown(); }
    );
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr exit_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  rclcpp::spin(std::make_shared<PCDPublisher>(options));
  return 0;
}

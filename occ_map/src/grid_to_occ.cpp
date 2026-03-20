#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"

#include "grid_map_ros/grid_map_ros.hpp"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"

class GridMapToOccNode : public rclcpp::Node
{
public:
  GridMapToOccNode()
  : Node("gridmap_to_occ_node")
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/grid_map");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/map");
    elevation_layer_ = this->declare_parameter<std::string>("elevation_layer", "elevation");
    slope_layer_ = this->declare_parameter<std::string>("slope_layer", "slope");
    occupancy_layer_ = this->declare_parameter<std::string>("occupancy_layer", "occupancy");

    slope_threshold_ = this->declare_parameter<double>("slope_threshold", 0.8);
    min_obstacle_height_ = this->declare_parameter<double>("min_obstacle_height", -1e9);
    max_obstacle_height_ = this->declare_parameter<double>("max_obstacle_height", 1e9);

    use_slope_ = this->declare_parameter<bool>("use_slope", true);
    use_height_bounds_ = this->declare_parameter<bool>("use_height_bounds", false);

    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic_, 10);

    grid_sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
      input_topic_, 10,
      std::bind(&GridMapToOccNode::gridMapCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "gridmap_to_occ_node started");
    RCLCPP_INFO(this->get_logger(), "Subscribing: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing:  %s", output_topic_.c_str());
  }

private:
  void gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    grid_map::GridMap grid_map;
    try {
      grid_map::GridMapRosConverter::fromMessage(*msg, grid_map);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to convert GridMap message: %s", e.what());
      return;
    }

    if (!grid_map.exists(elevation_layer_)) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Elevation layer '%s' does not exist.", elevation_layer_.c_str());
      return;
    }

    if (use_slope_ && !grid_map.exists(slope_layer_)) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Slope layer '%s' does not exist.", slope_layer_.c_str());
      return;
    }

    if (!grid_map.exists(occupancy_layer_)) {
      grid_map.add(occupancy_layer_);
    }

    auto & occ = grid_map[occupancy_layer_];
    occ.setConstant(std::nanf(""));

    for (grid_map::GridMapIterator it(grid_map); !it.isPastEnd(); ++it) {
      const grid_map::Index index(*it);

      const float elevation = grid_map.at(elevation_layer_, index);

      // 无效高程 -> unknown
      if (!std::isfinite(elevation)) {
        occ(index(0), index(1)) = -1.0f;
        continue;
      }

      bool occupied = false;

      if (use_slope_) {
        const float slope = grid_map.at(slope_layer_, index);
        if (!std::isfinite(slope)) {
          occ(index(0), index(1)) = -1.0f;
          continue;
        }
        if (std::fabs(slope) > slope_threshold_) {
          occupied = true;
        }
      }

      if (use_height_bounds_) {
        if (elevation < min_obstacle_height_ || elevation > max_obstacle_height_) {
          occupied = true;
        }
      }

      occ(index(0), index(1)) = occupied ? 100.0f : 0.0f;
    }

    nav_msgs::msg::OccupancyGrid occ_msg;
    try {
      // 这里 dataMin=0, dataMax=100，因为我们已经自己把 occupancy layer 填成 0/100/-1 了
      grid_map::GridMapRosConverter::toOccupancyGrid(
        grid_map, occupancy_layer_, 0.0, 100.0, occ_msg);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to convert to OccupancyGrid: %s", e.what());
      return;
    }

    // 保留原始 header/frame
    occ_msg.header.stamp = msg->header.stamp;
    occ_msg.header.frame_id = msg->header.frame_id;

    map_pub_->publish(occ_msg);
  }

private:
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

  std::string input_topic_;
  std::string output_topic_;
  std::string elevation_layer_;
  std::string slope_layer_;
  std::string occupancy_layer_;

  double slope_threshold_;
  double min_obstacle_height_;
  double max_obstacle_height_;
  bool use_slope_;
  bool use_height_bounds_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridMapToOccNode>());
  rclcpp::shutdown();
  return 0;
}
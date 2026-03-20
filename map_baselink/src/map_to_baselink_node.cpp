#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class MapToBaseLinkNode : public rclcpp::Node
{
public:
  MapToBaseLinkNode()
  : Node("map_to_baselink_node")
  {
    RCLCPP_INFO(this->get_logger(), "map_to_baselink_node started");
  }

private:
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapToBaseLinkNode>());
  rclcpp::shutdown();
  return 0;
}
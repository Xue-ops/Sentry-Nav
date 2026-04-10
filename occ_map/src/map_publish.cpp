#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdexcept>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace fs = std::filesystem;
using namespace std::chrono_literals;

class MapPublishNode : public rclcpp::Node
{
public:
  MapPublishNode()
  : Node("map_publish")
  {
    yaml_path_ = this->declare_parameter<std::string>("yaml_path", "");
    topic_name_ = this->declare_parameter<std::string>("topic_name", "/map");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map_nav");
    publish_period_ = this->declare_parameter<double>("publish_period", 1.0);

    if (yaml_path_.empty()) {
      RCLCPP_FATAL(this->get_logger(), "Parameter 'yaml_path' is empty.");
      throw std::runtime_error("yaml_path is empty");
    }

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();
    qos.transient_local();

    pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(topic_name_, qos);

    loadMapFromYaml(yaml_path_);
    publishMap();

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(publish_period_),
      std::bind(&MapPublishNode::publishMap, this));

    RCLCPP_INFO(this->get_logger(), "map_publish started.");
    RCLCPP_INFO(this->get_logger(), "yaml_path: %s", yaml_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "topic_name: %s", topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "publish_period: %.3f", publish_period_);
  }

private:
  static std::string trim(const std::string & s)
  {
    const auto begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
      return "";
    }
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
  }

  static std::string stripQuotes(const std::string & s)
  {
    if (s.size() >= 2 &&
      ((s.front() == '"' && s.back() == '"') ||
       (s.front() == '\'' && s.back() == '\'')))
    {
      return s.substr(1, s.size() - 2);
    }
    return s;
  }

  void loadMapFromYaml(const std::string & yaml_path)
  {
    std::ifstream fin(yaml_path);
    if (!fin.is_open()) {
      throw std::runtime_error("Failed to open yaml: " + yaml_path);
    }

    std::string image_name;
    double resolution = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    double origin_yaw = 0.0;
    int negate = 0;
    double occupied_thresh = 0.65;
    double free_thresh = 0.25;

    std::string line;
    while (std::getline(fin, line)) {
      line = trim(line);
      if (line.empty() || line[0] == '#') {
        continue;
      }

      if (line.rfind("image:", 0) == 0) {
        image_name = stripQuotes(trim(line.substr(6)));
      } else if (line.rfind("resolution:", 0) == 0) {
        resolution = std::stod(trim(line.substr(11)));
      } else if (line.rfind("origin:", 0) == 0) {
        std::string v = trim(line.substr(7));
        auto lb = v.find('[');
        auto rb = v.find(']');
        if (lb == std::string::npos || rb == std::string::npos || rb <= lb) {
          throw std::runtime_error("Invalid origin format in yaml");
        }
        std::string inside = v.substr(lb + 1, rb - lb - 1);
        std::replace(inside.begin(), inside.end(), ',', ' ');
        std::stringstream ss(inside);
        ss >> origin_x >> origin_y >> origin_yaw;
      } else if (line.rfind("negate:", 0) == 0) {
        negate = std::stoi(trim(line.substr(7)));
      } else if (line.rfind("occupied_thresh:", 0) == 0) {
        occupied_thresh = std::stod(trim(line.substr(17)));
      } else if (line.rfind("free_thresh:", 0) == 0) {
        free_thresh = std::stod(trim(line.substr(12)));
      }
    }

    if (image_name.empty()) {
      throw std::runtime_error("image field not found in yaml");
    }

    fs::path yaml_fs(yaml_path);
    fs::path image_path = image_name;
    if (image_path.is_relative()) {
      image_path = yaml_fs.parent_path() / image_path;
    }

    loadPgm(
      image_path.string(),
      resolution,
      origin_x, origin_y, origin_yaw,
      negate, occupied_thresh, free_thresh);
  }

  void loadPgm(
    const std::string & pgm_path,
    double resolution,
    double origin_x,
    double origin_y,
    double origin_yaw,
    int negate,
    double occupied_thresh,
    double free_thresh)
  {
    std::ifstream file(pgm_path, std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open pgm: " + pgm_path);
    }

    std::string magic;
    file >> magic;
    if (magic != "P5") {
      throw std::runtime_error("Only binary PGM (P5) is supported");
    }

    auto skip_comments = [&file]() {
      file >> std::ws;
      while (file.peek() == '#') {
        std::string dummy;
        std::getline(file, dummy);
        file >> std::ws;
      }
    };

    skip_comments();
    int width = 0;
    int height = 0;
    file >> width;
    skip_comments();
    file >> height;
    skip_comments();
    int maxval = 0;
    file >> maxval;
    file.get();  // consume one whitespace/newline after maxval

    if (width <= 0 || height <= 0 || maxval <= 0 || maxval > 255) {
      throw std::runtime_error("Invalid pgm header");
    }

    std::vector<unsigned char> pixels(static_cast<size_t>(width) * static_cast<size_t>(height));
    file.read(reinterpret_cast<char *>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
    if (!file) {
      throw std::runtime_error("Failed to read pgm pixels");
    }

    map_msg_.header.frame_id = frame_id_;
    map_msg_.info.resolution = static_cast<float>(resolution);
    map_msg_.info.width = static_cast<uint32_t>(width);
    map_msg_.info.height = static_cast<uint32_t>(height);

    map_msg_.info.origin = geometry_msgs::msg::Pose();
    map_msg_.info.origin.position.x = origin_x;
    map_msg_.info.origin.position.y = origin_y;
    map_msg_.info.origin.position.z = 0.0;
    map_msg_.info.origin.orientation.x = 0.0;
    map_msg_.info.origin.orientation.y = 0.0;
    map_msg_.info.origin.orientation.z = std::sin(origin_yaw / 2.0);
    map_msg_.info.origin.orientation.w = std::cos(origin_yaw / 2.0);

    map_msg_.data.clear();
    map_msg_.data.reserve(static_cast<size_t>(width) * static_cast<size_t>(height));

    for (int y = height - 1; y >= 0; --y) {
      for (int x = 0; x < width; ++x) {
        const unsigned char p = pixels[static_cast<size_t>(y) * width + x];

        double occ = 0.0;
        if (negate) {
          occ = static_cast<double>(p) / 255.0;
        } else {
          occ = static_cast<double>(255 - p) / 255.0;
        }

        int8_t val = -1;
        if (occ > occupied_thresh) {
          val = 100;
        } else if (occ < free_thresh) {
          val = 0;
        } else {
          val = -1;
        }

        map_msg_.data.push_back(val);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Loaded map: %dx%d from %s", width, height, pgm_path.c_str());
  }

  void publishMap()
  {
    map_msg_.header.stamp = this->now();
    pub_->publish(map_msg_);
  }

private:
  std::string yaml_path_;
  std::string topic_name_;
  std::string frame_id_;
  double publish_period_{1.0};

  nav_msgs::msg::OccupancyGrid map_msg_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<MapPublishNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    fprintf(stderr, "map_publish exception: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
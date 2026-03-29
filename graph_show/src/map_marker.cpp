#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <filesystem>

#include <yaml-cpp/yaml.h>

struct SavedPoint
{
  double x;
  double y;
  double z;
};

class MapMarker : public rclcpp::Node
{
public:
  MapMarker()
  : Node("map_marker")
  {
    this->declare_parameter<bool>("renew_file", false);
    renew_file_ = this->get_parameter("renew_file").as_bool();

    this->declare_parameter<std::string>(
      "save_file",
      "/home/xli/catkin_ws/src/graph_show/route/map_points.yaml");
    save_file_ = this->get_parameter("save_file").as_string();

    std::filesystem::create_directories(std::filesystem::path(save_file_).parent_path());

    sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point",
      10,
      std::bind(&MapMarker::clickedCallback, this, std::placeholders::_1));
      
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();
    qos.transient_local();

    pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/map_markers",
        qos
    );

    loadPoints();
    publishMarkers("odom");

    RCLCPP_INFO(this->get_logger(), "Ready. Saving points to: %s", save_file_.c_str());
  }

private:
    bool renew_file_;
    std::string save_file_;
    std::vector<SavedPoint> points_;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;

    static std::string expand_user_home(const std::string & path)
    {
        if (!path.empty() && path[0] == '~') {
        const char * home = std::getenv("HOME");
        if (home) {
            return std::string(home) + path.substr(1);
        }
        }
        return path;
    }

    void loadPoints()
    {
        std::ifstream fin(save_file_);
        if (!fin.good()) {
        RCLCPP_INFO(this->get_logger(), "No existing save file found, starting fresh.");
        return;
        }

        try {
        YAML::Node root = YAML::LoadFile(save_file_);
        if (!root["points"]) {
            RCLCPP_WARN(this->get_logger(), "YAML file exists but has no 'points' field.");
            return;
        }

        for (const auto & p : root["points"]) {
            SavedPoint pt;
            pt.x = p["x"].as<double>();
            pt.y = p["y"].as<double>();
            pt.z = p["z"].as<double>();
            points_.push_back(pt);
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu points from file.", points_.size());
        } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load save file: %s", e.what());
        }
    }

    void savePoints()
    {
        try {
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "points" << YAML::Value << YAML::BeginSeq;

        for (size_t i = 0; i < points_.size(); ++i) {
            out << YAML::BeginMap;
            out << YAML::Key << "id" << YAML::Value << static_cast<int>(i);
            out << YAML::Key << "x" << YAML::Value << points_[i].x;
            out << YAML::Key << "y" << YAML::Value << points_[i].y;
            out << YAML::Key << "z" << YAML::Value << points_[i].z;
            out << YAML::EndMap;
        }

        out << YAML::EndSeq;
        out << YAML::EndMap;

        std::ofstream fout(save_file_);
        fout << out.c_str();
        fout.close();
        } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to save points: %s", e.what());
        }
    }

    void clickedCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        if (!renew_file_) {
            return;
        }
        SavedPoint pt;
        pt.x = msg->point.x;
        pt.y = msg->point.y;
        pt.z = msg->point.z;

        points_.push_back(pt);
        savePoints();

        std::string frame_id = msg->header.frame_id.empty() ? "odom" : msg->header.frame_id;
        publishMarkers(frame_id);

        RCLCPP_INFO(
        this->get_logger(),
        "Saved point #%zu: (%.3f, %.3f, %.3f)",
        points_.size() - 1, pt.x, pt.y, pt.z);
    }

    void publishMarkers(const std::string & frame_id)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        for (size_t i = 0; i < points_.size(); ++i) {
        // 红色球
        visualization_msgs::msg::Marker sphere;
        sphere.header.frame_id = frame_id;
        sphere.header.stamp = this->now();
        sphere.ns = "saved_points";
        sphere.id = static_cast<int>(i);
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;

        sphere.pose.position.x = points_[i].x;
        sphere.pose.position.y = points_[i].y;
        sphere.pose.position.z = points_[i].z;
        sphere.pose.orientation.w = 1.0;

        sphere.scale.x = 0.01;
        sphere.scale.y = 0.01;
        sphere.scale.z = 0.01;

        sphere.color.r = 1.0f;
        sphere.color.g = 0.0f;
        sphere.color.b = 0.0f;
        sphere.color.a = 1.0f;

        marker_array.markers.push_back(sphere);

        // 文字标签
        visualization_msgs::msg::Marker text;
        text.header.frame_id = frame_id;
        text.header.stamp = this->now();
        text.ns = "saved_labels";
        text.id = 10000 + static_cast<int>(i);
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;

        text.pose.position.x = points_[i].x;
        text.pose.position.y = points_[i].y;
        text.pose.position.z = points_[i].z + 0.3;
        text.pose.orientation.w = 1.0;

        text.scale.z = 0.2;

        text.color.r = 1.0f;
        text.color.g = 1.0f;
        text.color.b = 1.0f;
        text.color.a = 1.0f;

        text.text = "P" + std::to_string(i);

        marker_array.markers.push_back(text);
        }

        pub_->publish(marker_array);
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMarker>());
  rclcpp::shutdown();
  return 0;
}
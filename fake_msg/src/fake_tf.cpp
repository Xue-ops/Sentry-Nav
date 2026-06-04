#include <memory>
#include <string>
#include <chrono>
#include <algorithm>
#include <functional>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"

class FakeTF : public rclcpp::Node
{
public:
    FakeTF()
    : Node("fake_tf")
    {
        odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
        map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
        mid_frame_ = this->declare_parameter<std::string>("mid_frame", "livox_frame");
        odin_frame_ = this->declare_parameter<std::string>("odin_frame", "odin1_base_link");
        publish_rate_ = this->declare_parameter<double>("publish_rate", 20.0);

        auto make_transform_param = [this](
        const std::string & param_name,
        const std::string & child_frame,
        const std::vector<double> & default_value) {
            auto values = this->declare_parameter<std::vector<double>>(param_name, default_value);
            if (values.size() != 7) {
            RCLCPP_WARN(
                this->get_logger(),
                "%s must be [x, y, z, qx, qy, qz, qw], using default value",
                param_name.c_str());
            values = default_value;
            }

            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.frame_id = base_frame_;
            tf_msg.child_frame_id = child_frame;
            tf_msg.transform.translation.x = values[0];
            tf_msg.transform.translation.y = values[1];
            tf_msg.transform.translation.z = values[2];
            tf_msg.transform.rotation.x = values[3];
            tf_msg.transform.rotation.y = values[4];
            tf_msg.transform.rotation.z = values[5];
            tf_msg.transform.rotation.w = values[6];
            return tf_msg;
        };

        tf_baselink_odin1_ = make_transform_param(
            "tf_baselink_odin1",
            odin_frame_,
            {-0.02897, 0.11458, 0.24799, 0.0, 0.0, 0.7071068, 0.7071068});

        tf_baselink_mid360_ = make_transform_param(
            "tf_baselink_mid360",
            mid_frame_,
            {0.0025, -0.13509, 0.29325, 0.0, 0.382684, -0.923879, 0.0});

        tf_map_odom_ = make_transform_param(
            "tf_map_odom",
            odom_frame_,
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
        tf_map_odom_.header.frame_id = map_frame_;

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(),
            this->get_node_timers_interface());
        tf_buffer_->setCreateTimerInterface(timer_interface);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_));
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&FakeTF::timerCallback, this));

        RCLCPP_INFO(
            this->get_logger(),
            "Started fake_tf: listen %s -> %s, publish %s -> %s -> %s and %s -> %s",
            odom_frame_.c_str(),
            odin_frame_.c_str(),
            map_frame_.c_str(),
            odom_frame_.c_str(),
            base_frame_.c_str(),
            base_frame_.c_str(),
            mid_frame_.c_str());
    }

private:
    std::string odom_frame_;
    std::string map_frame_;
    std::string base_frame_;
    std::string mid_frame_;
    std::string odin_frame_;
    geometry_msgs::msg::TransformStamped tf_baselink_odin1_;
    geometry_msgs::msg::TransformStamped tf_baselink_mid360_;
    geometry_msgs::msg::TransformStamped tf_map_odom_;
    double publish_rate_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_publish_stamp_{0, 0, RCL_ROS_TIME};

    void resetTfListener()
    {
        tf_listener_.reset();
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(),
            this->get_node_timers_interface());
        tf_buffer_->setCreateTimerInterface(timer_interface);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }
    
    void timerCallback()
    {
        const auto stamp = this->now();
        if (last_publish_stamp_.nanoseconds() > 0 && stamp < last_publish_stamp_) {
            RCLCPP_WARN(
                this->get_logger(),
                "Detected time jump backwards from %.3f to %.3f, resetting TF listener buffer",
                last_publish_stamp_.seconds(),
                stamp.seconds());
            resetTfListener();
        }
        last_publish_stamp_ = stamp;

        //publish map-odom-base and base-mid TFs based on tf_baselink_odin1_ and tf_baselink_mid360_
        //For the odom-base, first get the transfer odom-odin1, than use the tf_baselink_odin1_ to imply odom-base
        geometry_msgs::msg::TransformStamped tf_odom_odin1;
        try {
            tf_odom_odin1 = tf_buffer_->lookupTransform(
                odom_frame_,
                odin_frame_,
                tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Cannot lookup %s -> %s: %s",
                odom_frame_.c_str(),
                odin_frame_.c_str(),
                ex.what());
            return;
        }

        tf2::Transform T_odom_odin1;
        tf2::Transform T_baselink_odin1;
        tf2::fromMsg(tf_odom_odin1.transform, T_odom_odin1);
        tf2::fromMsg(tf_baselink_odin1_.transform, T_baselink_odin1);
        const tf2::Transform T_odom_baselink =
            T_odom_odin1 * T_baselink_odin1.inverse();

        geometry_msgs::msg::TransformStamped tf_map_odom = tf_map_odom_;
        tf_map_odom.header.stamp = stamp;
        tf_map_odom.header.frame_id = map_frame_;
        tf_map_odom.child_frame_id = odom_frame_;

        geometry_msgs::msg::TransformStamped tf_odom_baselink;
        tf_odom_baselink.header.stamp = stamp;
        tf_odom_baselink.header.frame_id = odom_frame_;
        tf_odom_baselink.child_frame_id = base_frame_;
        tf_odom_baselink.transform.translation.x = T_odom_baselink.getOrigin().x();
        tf_odom_baselink.transform.translation.y = T_odom_baselink.getOrigin().y();
        tf_odom_baselink.transform.translation.z = T_odom_baselink.getOrigin().z();
        tf_odom_baselink.transform.rotation = tf2::toMsg(T_odom_baselink.getRotation());

        geometry_msgs::msg::TransformStamped tf_baselink_mid360 = tf_baselink_mid360_;
        tf_baselink_mid360.header.stamp = stamp;
        tf_baselink_mid360.header.frame_id = base_frame_;
        tf_baselink_mid360.child_frame_id = mid_frame_;

        tf_broadcaster_->sendTransform(
            std::vector<geometry_msgs::msg::TransformStamped>{
                tf_map_odom,
                tf_odom_baselink,
                tf_baselink_mid360});
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeTF>());
    rclcpp::shutdown();
    return 0;
}

#ifndef MOTION_CONTROLLER_PKG__MOTION_CONTROLLER_HPP_
#define MOTION_CONTROLLER_PKG__MOTION_CONTROLLER_HPP_

#include <memory>
#include <chrono>
#include <string>
#include <vector>
#include <functional>
#include <pluginlib/class_loader.hpp>
#include <cstdlib> // for std::setenv
#include <Eigen/Dense>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>  
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <cmath>  // for std::atan2

// Mensajes de ROS2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "as2_core/node.hpp"

// Base del plugin del controlador (instalado de forma binaria)
#include "as2_motion_controller/controller_base.hpp"

class PidControllerNode : public as2::Node
{
public:
    PidControllerNode();
    void initTfListener();
    ~PidControllerNode();

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void trajCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void timerCallback();

    // Member variables...
    std::unique_ptr<pluginlib::ClassLoader<as2_motion_controller_plugin_base::ControllerBase>> plugin_loader_;
    std::shared_ptr<as2_motion_controller_plugin_base::ControllerBase> controller_plugin_;

    std::vector<std::string> axes;
    std::vector<rclcpp::Parameter> params_;

    as2_msgs::msg::ControlMode input_mode_;
    as2_msgs::msg::ControlMode output_mode_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr traj_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2_ros::Buffer tf_buffer_;

    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::TwistStamped current_twist_;
    as2_msgs::msg::TrajectorySetpoints desired_traj_;
    bool traj_goal_defined_;
    as2_msgs::msg::TrajectorySetpoints circular_traj_;

    geometry_msgs::msg::PoseStamped unused_pose_;
    geometry_msgs::msg::TwistStamped command_twist_;
    as2_msgs::msg::Thrust unused_thrust_;

    const double distance_threshold_ = 0.2;
    const double circular_threshold_ = 0.15;
    const int num_points_ = 20;
    const int radius_ = 3;
};

#endif  // MOTION_CONTROLLER_PKG__MOTION_CONTROLLER_HPP_

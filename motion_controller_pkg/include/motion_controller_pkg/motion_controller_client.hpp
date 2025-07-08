#ifndef MOTION_CONTROLLER_PKG__MOTION_CONTROLLER_CLIENT_HPP_
#define MOTION_CONTROLLER_PKG__MOTION_CONTROLLER_CLIENT_HPP_

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
#include "as2_msgs/action/navigate_to_point.hpp"
#include "as2_core/node.hpp"

class MotionClientNode : public as2::Node
{
public:
    MotionClientNode();
    ~MotionClientNode();

    void sendNavigateGoal(double x, double y, double z);

private:
    rclcpp_action::Client<as2_msgs::action::NavigateToPoint>::SharedPtr nav_client_;
};

#endif  // MOTION_CONTROLLER_PKG__MOTION_CONTROLLER_CLIENT_HPP_
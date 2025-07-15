#ifndef MOTION_CONTROLLER_PKG__MOTION_CONTROLLER_CLIENT_HPP_
#define MOTION_CONTROLLER_PKG__MOTION_CONTROLLER_CLIENT_HPP_

#include <memory>
#include <chrono>
#include <string>
#include <vector>
#include <functional>
#include <pluginlib/class_loader.hpp>
// #include <cstdlib> // for std::setenv
#include <Eigen/Dense>
#include <cmath>  // for std::atan2

// Mensajes de ROS2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "motion_controller_pkg/action/goal_point.hpp"
#include "as2_core/node.hpp"

class MotionClientNode : public as2::Node
{
public:
    MotionClientNode();
    ~MotionClientNode();

private:
    rclcpp_action::Client<motion_controller_pkg::action::GoalPoint>::SharedPtr nav_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;

    void sendGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void feedbackCallback(
      std::shared_ptr<rclcpp_action::ClientGoalHandle<motion_controller_pkg::action::GoalPoint>>,
      const std::shared_ptr<const motion_controller_pkg::action::GoalPoint::Feedback> &);

    void resultCallback(
      const rclcpp_action::ClientGoalHandle<motion_controller_pkg::action::GoalPoint>::WrappedResult &);
};

#endif  // MOTION_CONTROLLER_PKG__MOTION_CONTROLLER_CLIENT_HPP_
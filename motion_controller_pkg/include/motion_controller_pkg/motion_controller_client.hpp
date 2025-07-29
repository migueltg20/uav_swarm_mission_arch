#ifndef MOTION_CONTROLLER_PKG__MOTION_CONTROLLER_CLIENT_HPP_
#define MOTION_CONTROLLER_PKG__MOTION_CONTROLLER_CLIENT_HPP_

// C++ standard libraries
#include <memory>
#include <chrono>
#include <string>
#include <vector>
#include <functional>
#include <map>

// ROS2 libraries
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "motion_controller_pkg/msg/goal_command.hpp"
#include "motion_controller_pkg/action/goal_point.hpp"

// Aerostack2 libraries
#include "as2_core/node.hpp"



/*!*******************************************************************************************
 *  \file       motion_controller_client.hpp
 *  \brief      Controller client class definition
 *  \authors    Miguel Tejado García
 ********************************************************************************************/



class MotionClientNode : public as2::Node
{
public:
  MotionClientNode();
  ~MotionClientNode();

private:
  // Callback for the goal subscriber
  void sendGoal(const motion_controller_pkg::msg::GoalCommand::SharedPtr msg);


  // Callback for the action client goal response
  void goalResponseCallback(
    int id,
    std::shared_ptr<rclcpp_action::ClientGoalHandle<motion_controller_pkg::action::GoalPoint>> goal_handle);


  // Callbacks for the action client feedback and result
  void feedbackCallback(
    int id,
    std::shared_ptr<rclcpp_action::ClientGoalHandle<motion_controller_pkg::action::GoalPoint>>,
    const std::shared_ptr<const motion_controller_pkg::action::GoalPoint::Feedback> &);

  void resultCallback(
    int id,
    const rclcpp_action::ClientGoalHandle<motion_controller_pkg::action::GoalPoint>::WrappedResult &);


  // Member variables
  rclcpp::Subscription<motion_controller_pkg::msg::GoalCommand>::SharedPtr goal_subscriber_;
  std::map<int, rclcpp_action::Client<motion_controller_pkg::action::GoalPoint>::SharedPtr> nav_clients_;
  std::map<int, rclcpp_action::ClientGoalHandle<motion_controller_pkg::action::GoalPoint>::SharedPtr> active_goals_;
};

#endif  // MOTION_CONTROLLER_PKG__MOTION_CONTROLLER_CLIENT_HPP_
#ifndef MOTION_CONTROLLER_PKG__MOTION_CONTROLLER_CLIENT_HPP_
#define MOTION_CONTROLLER_PKG__MOTION_CONTROLLER_CLIENT_HPP_

// C++ standard libraries
#include <memory>
#include <chrono>
#include <string>
#include <vector>
#include <functional>
#include <mutex>

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

/* 
 * Motion Controller Client Node
 * Manages action clients for multiple drones (up to 4) and handles goal commands
 * for coordinated swarm navigation and motion control
 */
class MotionClientNode : public as2::Node
{
  using GoalCommand = motion_controller_pkg::msg::GoalCommand;
  using GoalPoint = motion_controller_pkg::action::GoalPoint;


public:
  /* Constructor - initializes the motion controller client node */
  MotionClientNode();
  
  /* Destructor - ensures clean shutdown of all active connections */
  ~MotionClientNode();

  
private:
  /* Main goal processing function - validates and sends goals to appropriate drone */
  void sendGoal(const GoalCommand::SharedPtr msg);

  /* Handles server response when a goal is accepted or rejected */
  void goalResponseCallback(
    int id,
    std::shared_ptr<rclcpp_action::ClientGoalHandle<GoalPoint>> goal_handle);

  /* Receives periodic updates during goal execution */
  void feedbackCallback(
    int id,
    std::shared_ptr<rclcpp_action::ClientGoalHandle<GoalPoint>> goal_handle,
    const std::shared_ptr<const GoalPoint::Feedback> & feedback);

  /* Processes final result when goal completes, fails, or is cancelled */
  void resultCallback(
    int id,
    const rclcpp_action::ClientGoalHandle<GoalPoint>::WrappedResult & result);

  /* Manually cancels an active goal for specified drone */
  void cancelGoal(int id);

  /* Thread safety mutexes for each drone's goal operations */
  std::array<std::mutex, 4> goal_mutexes_;

  /* Subscription to receive goal commands from external sources */
  rclcpp::Subscription<GoalCommand>::SharedPtr goal_subscriber_;
  
  /* Action clients for each drone (max 4 drones supported) */
  std::array<rclcpp_action::Client<GoalPoint>::SharedPtr, 4> nav_clients_;
  
  /* Currently active goal handles for tracking and cancellation */
  std::array<rclcpp_action::ClientGoalHandle<GoalPoint>::SharedPtr, 4> active_goals_;
};

#endif  // MOTION_CONTROLLER_PKG__MOTION_CONTROLLER_CLIENT_HPP_
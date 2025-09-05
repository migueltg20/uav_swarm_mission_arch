#ifndef BEHAVIOUR_TREES_PKG__ARMING_BH_SERVER_HPP
#define BEHAVIOUR_TREES_PKG__ARMING_BH_SERVER_HPP

/* C++ standard library includes for core functionality */
#include <array>                    // for fixed-size arrays
#include <memory>                   // for smart pointers
#include <future>                   // for asynchronous operations
#include <string>                   // for string handling
#include <cstdlib>                  // for environment variables

/* ROS2 includes for communication and behavior framework */
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "as2_behavior/behavior_server.hpp"
#include "as2_msgs/action/set_arming_state.hpp"

/*!*******************************************************************************************
 *  \file       arming_bh_server.hpp
 *  \brief      Arming behaviour server class declaration and member variable definitions
 *  \authors    Miguel Tejado García
 ********************************************************************************************/

/* Behavior server class for managing drone arming/disarming operations across multiple UAVs */
class ArmingServer : public as2_behavior::BehaviorServer<as2_msgs::action::SetArmingState>
{
  /* Type alias for cleaner code */
  using ArmingBh = as2_msgs::action::SetArmingState;

public:
  /* Constructor and destructor */
  ArmingServer();   // Initialize service clients for all drones
  ~ArmingServer();  // Cleanup resources


protected:
  /* Behavior lifecycle callbacks - inherited from BehaviorServer */
  bool on_activate(std::shared_ptr<const ArmingBh::Goal> goal) override;         // Start arming process
  bool on_modify(std::shared_ptr<const ArmingBh::Goal> goal) override;           // Modify arming parameters
  bool on_deactivate(const std::shared_ptr<std::string>&) override;              // Stop arming (not allowed)
  bool on_pause(const std::shared_ptr<std::string>&) override;                   // Pause arming (not allowed)
  bool on_resume(const std::shared_ptr<std::string>&) override;                  // Resume arming (not allowed)
  
  /* Main execution function - called repeatedly during behavior execution */
  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const ArmingBh::Goal>& goal,
    std::shared_ptr<ArmingBh::Feedback>& feedback,
    std::shared_ptr<ArmingBh::Result>& result) override;                         // Monitor arming progress
  
  /* Cleanup callback when execution ends */
  void on_execution_end(const as2_behavior::ExecutionStatus& state) override;    // Log final status


private:
  /* Timing for timeout handling */
  rclcpp::Time activation_time_;                                                  // Record when arming started

  /* Service communication arrays - one entry per drone (4 total) */
  std::array<rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr, 4> platform_arm_cli_;        // Service clients
  std::array<std::shared_ptr<std_srvs::srv::SetBool::Request>, 4> platform_arm_request_;     // Pre-allocated requests
  std::array<std::shared_future<std_srvs::srv::SetBool::Response::SharedPtr>, 4> platform_arm_future_;  // Async response futures
};

#endif  // BEHAVIOUR_TREES_PKG__ARMING_BH_SERVER_HPP

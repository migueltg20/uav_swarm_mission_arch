#ifndef BEHAVIOUR_TREES_PKG__OFFBOARD_BH_SERVER_HPP
#define BEHAVIOUR_TREES_PKG__OFFBOARD_BH_SERVER_HPP

#include <array>
#include <memory>
#include <future>
#include <string>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "as2_behavior/behavior_server.hpp"
#include "as2_msgs/action/set_offboard_mode.hpp"

/*!*******************************************************************************************
 *  \file       offboard_bh_server.hpp
 *  \brief      Offboard behaviour server class declaration and member variable definitions
 *  \authors    Miguel Tejado García
 ********************************************************************************************/

/* Behavior server class for managing offboard mode transitions across UAV swarm */
class OffboardServer : public as2_behavior::BehaviorServer<as2_msgs::action::SetOffboardMode>
{
  using OffboardBh = as2_msgs::action::SetOffboardMode;  // Action type alias for convenience


public:
  /* Class lifecycle methods */
  OffboardServer();
  ~OffboardServer();


protected:
  /* Behavior server interface methods - override base class virtual functions */
  bool on_activate(std::shared_ptr<const OffboardBh::Goal> goal) override;  // Initialize offboard transition
  bool on_modify(std::shared_ptr<const OffboardBh::Goal> goal) override;   // Modify behavior (not supported)
  bool on_deactivate(const std::shared_ptr<std::string>& msg) override;   // Deactivate behavior (not supported)
  bool on_pause(const std::shared_ptr<std::string>& msg) override;        // Pause behavior (not supported)
  bool on_resume(const std::shared_ptr<std::string>& msg) override;       // Resume behavior (not supported)

  /* Main execution method - monitors offboard transition progress */
  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const OffboardBh::Goal>& goal,
    std::shared_ptr<OffboardBh::Feedback>& feedback,
    std::shared_ptr<OffboardBh::Result>& result) override;

  /* Execution completion handler */
  void on_execution_end(const as2_behavior::ExecutionStatus& state) override;

  
private:
  rclcpp::Time activation_time_;  // Timestamp for timeout calculations

  /* Service communication arrays for 4-UAV swarm */
  std::array<rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr, 4> platform_offboard_cli_;  // Service clients
  std::array<std::shared_ptr<std_srvs::srv::SetBool::Request>, 4> platform_offboard_request_;  // Request objects
  std::array<std::shared_future<std_srvs::srv::SetBool::Response::SharedPtr>, 4> platform_offboard_future_;  // Async response futures
};

#endif  // BEHAVIOUR_TREES_PKG__OFFBOARD_BH_SERVER_HPP

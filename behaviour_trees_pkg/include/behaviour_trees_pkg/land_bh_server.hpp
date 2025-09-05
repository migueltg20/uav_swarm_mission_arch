#ifndef BEHAVIOUR_TREES_PKG__LAND_BH_SERVER_HPP
#define BEHAVIOUR_TREES_PKG__LAND_BH_SERVER_HPP

#include <mutex>
#include <array>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "as2_behavior/behavior_server.hpp"
#include "behaviour_trees_pkg/action/land_bh.hpp"
#include "motion_controller_pkg/msg/goal_command.hpp"

/*!*******************************************************************************************
 *  \file       land_bh_server.hpp
 *  \brief      Land behaviour server class declaration and member variable definitions
 *  \authors    Miguel Tejado García
 ********************************************************************************************/

/* 
 * LandServer class - Manages autonomous landing behavior for drone swarms
 * Inherits from BehaviorServer to provide standardized behavior lifecycle management
 */
class LandServer : public as2_behavior::BehaviorServer<behaviour_trees_pkg::action::LandBh>
{
  /* Type aliases for cleaner code */
  using LandBh = behaviour_trees_pkg::action::LandBh;
  using GoalCommand = motion_controller_pkg::msg::GoalCommand;


public:
  LandServer();   // Initialize subscribers, publishers and data structures
  ~LandServer();  // Cleanup resources


protected:
  /* Behavior lifecycle management methods - called by behavior server framework */
  bool on_activate(std::shared_ptr<const LandBh::Goal> goal) override;     // Start landing sequence
  bool on_modify(std::shared_ptr<const LandBh::Goal> goal) override;       // Update landing parameters
  bool on_deactivate(const std::shared_ptr<std::string>& /*msg*/) override; // Stop landing (disabled for safety)
  bool on_pause(const std::shared_ptr<std::string>& /*msg*/) override;      // Pause landing (disabled for safety)
  bool on_resume(const std::shared_ptr<std::string>& /*msg*/) override;     // Resume landing (disabled for safety)
  
  /* Main execution method - monitors landing progress and completion */
  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const LandBh::Goal>& goal,
    std::shared_ptr<LandBh::Feedback>& feedback,
    std::shared_ptr<LandBh::Result>& result) override;
  
  void on_execution_end(const as2_behavior::ExecutionStatus& state) override; // Cleanup after completion

  
private:
  /* Callback methods for processing incoming sensor data */
  void poseCallback(int drone_id, const geometry_msgs::msg::PoseStamped::SharedPtr msg);   // Process altitude updates
  void twistCallback(int drone_id, const geometry_msgs::msg::TwistStamped::SharedPtr msg); // Process velocity updates

  /* QoS profile optimized for real-time sensor data */
  rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data))
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .durability(rclcpp::DurabilityPolicy::Volatile);

  /* ROS2 communication interfaces - supports up to 4 drones */
  std::array<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr, 4> pose_subscribers_;  // Altitude monitoring
  std::array<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr, 4> twist_subscribers_; // Velocity monitoring
  rclcpp::Publisher<GoalCommand>::SharedPtr goal_publisher_; // Command interface to flight controller

  /* Drone state tracking arrays - indexed by drone_id */
  std::array<float, 4> current_height_;     // Current altitude for each drone (meters)
  std::array<float, 4> initial_height_;     // Starting altitude when landing begins (meters)
  std::array<bool, 4> initial_height_set_;  // Flag indicating if initial height has been recorded
  std::array<geometry_msgs::msg::TwistStamped, 4> current_twist_; // Current velocity for each drone
  std::array<std::mutex, 4> drone_mutexes_; // Thread safety for concurrent access to drone data

  /* Landing operation state */
  GoalCommand goal_command_;           // Command message sent to flight controller
  rclcpp::Time landing_start_time_;    // Timestamp when landing operation began (for timeout)

  float land_threshold_ = 0.01f;       // Height threshold for successful landing detection (meters)
};

#endif  // BEHAVIOUR_TREES_PKG__LAND_BH_SERVER_HPP

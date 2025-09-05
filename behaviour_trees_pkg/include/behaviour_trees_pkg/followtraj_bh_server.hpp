#ifndef BEHAVIOUR_TREES_PKG__FOLLOWTRAJ_BH_SERVER_HPP
#define BEHAVIOUR_TREES_PKG__FOLLOWTRAJ_BH_SERVER_HPP

#include <mutex>
#include <array>
#include <cmath>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "as2_msgs/msg/trajectory_setpoints.hpp"

#include "as2_behavior/behavior_server.hpp"
#include "behaviour_trees_pkg/action/follow_traj_bh.hpp"
#include "motion_controller_pkg/msg/goal_command.hpp"

/*!*******************************************************************************************
 *  \file       followtraj_bh_server.hpp
 *  \brief      Follow trajectory behaviour server class declaration and member variable definitions
 *  \authors    Miguel Tejado García
 ********************************************************************************************/

/* 
 * Behavior server for managing trajectory following of multiple UAVs
 * Supports both regular waypoint following and circular trajectory modes
 * Thread-safe operation for concurrent drone management
 */
class FollowTrajServer : public as2_behavior::BehaviorServer<behaviour_trees_pkg::action::FollowTrajBh>
{
  using FollowTrajBh = behaviour_trees_pkg::action::FollowTrajBh;
  using GoalCommand = motion_controller_pkg::msg::GoalCommand;


public:
  FollowTrajServer();   // Initialize subscribers and publishers
  ~FollowTrajServer();  // Cleanup resources

  
protected:
  /* Behavior lifecycle methods - control trajectory execution state */
  bool on_activate(std::shared_ptr<const FollowTrajBh::Goal> goal) override;
  bool on_modify(std::shared_ptr<const FollowTrajBh::Goal> goal) override;
  bool on_deactivate(const std::shared_ptr<std::string>& msg) override;
  bool on_pause(const std::shared_ptr<std::string>& msg) override;
  bool on_resume(const std::shared_ptr<std::string>& msg) override;
  
  /* Main execution loop - monitors progress and provides feedback */
  as2_behavior::ExecutionStatus on_run(
      const std::shared_ptr<const FollowTrajBh::Goal>& goal,
      std::shared_ptr<FollowTrajBh::Feedback>& feedback,
      std::shared_ptr<FollowTrajBh::Result>& result) override;
  
  /* Called when behavior execution completes */
  void on_execution_end(const as2_behavior::ExecutionStatus& state) override;

  
private:
  /* Callback methods for receiving drone state updates */
  void poseCallback(int drone_id, const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void twistCallback(int drone_id, const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  
  /* Utility methods for trajectory processing */
  void setupGoalFromTrajectory();  // Convert trajectory setpoint to pose goal
  bool isAtPosition(const geometry_msgs::msg::PoseStamped& current,
                    const as2_msgs::msg::TrajectoryPoint& target);  // Position threshold check

  /* QoS profile optimized for sensor data */
  rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data))
      .reliability(rclcpp::ReliabilityPolicy::BestEffort)
      .durability(rclcpp::DurabilityPolicy::Volatile);

  /* ROS2 communication interfaces */
  std::array<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr, 4> pose_subscribers_;   // Pose data from each drone
  std::array<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr, 4> twist_subscribers_; // Velocity data from each drone
  rclcpp::Publisher<GoalCommand>::SharedPtr goal_publisher_;  // Commands to motion controller

  /* Current state data for each drone (indexed by drone_id) */
  std::array<geometry_msgs::msg::PoseStamped, 4> current_pose_;    // Latest position and orientation
  std::array<geometry_msgs::msg::TwistStamped, 4> current_twist_;  // Latest linear and angular velocities
  
  /* Trajectory tracking data for each drone */
  std::array<as2_msgs::msg::TrajectorySetpoints, 4> goal_trajectory_;  // Local copy for progress tracking
  std::array<geometry_msgs::msg::PoseStamped, 4> goal_pose_;           // Single point goals for conversion
  std::array<bool, 4>                         need_height_update_;     // Flags for Z-coordinate updates
  std::array<std::mutex, 4>                   drone_mutexes_;          // Thread safety per drone

  GoalCommand goal_command_;  // Current command being sent to controller

  /* Position tolerance thresholds */
  double distance_threshold_{0.5};  // XY position tolerance (meters)
  double height_threshold_{0.1};    // Z position tolerance (meters)
};

#endif  // BEHAVIOUR_TREES_PKG__FOLLOWTRAJ_BH_SERVER_HPP

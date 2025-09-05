#ifndef MOTION_CONTROLLER_PKG__MOTION_CONTROLLER_HPP_
#define MOTION_CONTROLLER_PKG__MOTION_CONTROLLER_HPP_

/* C++ standard library includes for core functionality */
#include <memory>
#include <chrono>
#include <string>
#include <vector>
#include <functional>
#include <pluginlib/class_loader.hpp>
#include <cstdlib>                                      // for std::setenv
#include <thread>
#include <Eigen/Dense>                                  // for vector mathematics
#include <cmath>                                        // for std::atan2
#include <mutex>                                        // for thread synchronization

/* ROS2 core libraries for node functionality and action servers */
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "motion_controller_pkg/msg/goal_command.hpp"
#include "motion_controller_pkg/action/goal_point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

/* TF2 libraries for coordinate frame transformations */
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>  
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 

/* Aerostack2 framework includes for drone control */
#include "as2_core/node.hpp"
#include "as2_motion_controller/controller_base.hpp"

/*!*******************************************************************************************
 *  \file       motion_controller.hpp
 *  \brief      Controller class declaration and member variable definitions
 *  \authors    Miguel Tejado García
 ********************************************************************************************/

/* Main controller class that handles multiple drone motion control using PID plugins */
class PidControllerNode : public as2::Node
{
  /* Type aliases for cleaner code */
  using GoalCommand = motion_controller_pkg::msg::GoalCommand;
  using GoalPoint = motion_controller_pkg::action::GoalPoint;


public:
    /* Constructor and destructor */
    PidControllerNode();                    // Initialize node with action servers and subscribers
    void initTfListener();                  // Setup transform listener for coordinate conversions
    ~PidControllerNode();                   // Clean shutdown of controllers and plugins

    
private:
    /* Callback functions for ROS2 communication */
    void poseCallback(int drone_id, const geometry_msgs::msg::PoseStamped::SharedPtr msg);     // Handle pose updates
    void twistCallback(int drone_id, const geometry_msgs::msg::TwistStamped::SharedPtr msg);   // Handle velocity updates
    void timerCallback(int drone_id);                                                          // Main control loop

    /* Action server callback functions for goal management */
    rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID & /*uuid*/,
      std::shared_ptr<const GoalPoint::Goal> goal);                                            // Accept/reject new goals

    void cleanupGoal(int drone_id);                                                             // Reset trajectory data

    rclcpp_action::CancelResponse handleCancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoalPoint>> goal_handle);         // Handle goal cancellation

    void handleAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoalPoint>> goal_handle);         // Start goal execution

    void executeGoal(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoalPoint>> goal_handle);         // Initialize controller for goal

    void processGoal(const GoalCommand & msg);                                                  // Generate trajectory from goal

    /* Thread safety - one mutex per drone to prevent race conditions */
    std::array<std::mutex, 4> drone_mutexes_;

    /* Action server infrastructure - one server per drone */
    std::array<rclcpp_action::Server<GoalPoint>::SharedPtr, 4> nav_servers_;
    std::array<std::shared_ptr<rclcpp_action::ServerGoalHandle<GoalPoint>>, 4> active_goals_;  // Track active goals

    /* Plugin management for motion controllers */
    std::unique_ptr<pluginlib::ClassLoader<as2_motion_controller_plugin_base::ControllerBase>> plugin_loader_;
    std::array<std::shared_ptr<as2_motion_controller_plugin_base::ControllerBase>, 4> controllers_;  // One controller per drone

    /* QoS profile optimized for real-time sensor data */
    rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data))
              .reliability(rclcpp::ReliabilityPolicy::BestEffort)
              .durability(rclcpp::DurabilityPolicy::Volatile);

    /* ROS2 communication interfaces - arrays for multi-drone support */
    std::array<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr, 4> pose_subscribers_;      // Position feedback
    std::array<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr, 4> twist_subscribers_;    // Velocity feedback
    std::array<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr, 4> traj_subscribers_;      // Trajectory input
    std::array<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr, 4> twist_publishers_;        // Velocity commands

    /* Transform system for coordinate frame conversions */
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2_ros::Buffer tf_buffer_;   // Thread-safe buffer for transform lookups

    /* Control loop timers - one per drone for independent control */
    std::array<rclcpp::TimerBase::SharedPtr, 4> timers_;

    /* Controller parameters - stored per drone for individual tuning */
    std::array<std::vector<rclcpp::Parameter>, 4> params_;

    /* Control mode configuration - shared across all drones */
    as2_msgs::msg::ControlMode input_mode_;         // Input trajectory mode (ENU frame)
    as2_msgs::msg::ControlMode output_mode_;        // Output velocity mode (ENU frame)

    /* State tracking variables for each drone */
    std::array<geometry_msgs::msg::PoseStamped, 4> current_pose_;           // Current position and orientation
    std::array<geometry_msgs::msg::TwistStamped, 4> current_twist_;         // Current linear and angular velocities
    std::array<as2_msgs::msg::TrajectorySetpoints, 4> desired_traj_;        // Linear trajectory waypoints
    std::array<bool, 4> traj_goal_defined_;                                 // Flag for active linear trajectory
    std::array<bool, 4> not_circular_;                                      // Flag to distinguish trajectory types
    std::array<as2_msgs::msg::TrajectorySetpoints, 4> circular_traj_;       // Circular trajectory waypoints

    /* Trajectory following thresholds - compile-time constants for performance */
    static constexpr double distance_threshold_ = 0.4;     // Horizontal waypoint reach threshold (meters)
    static constexpr double height_threshold_ = 0.1;       // Vertical waypoint reach threshold (meters)
    static constexpr double circular_threshold_ = 0.3;     // Circular trajectory waypoint threshold (meters)
};

#endif  // MOTION_CONTROLLER_PKG__MOTION_CONTROLLER_HPP_

#ifndef MOTION_CONTROLLER_PKG__MOTION_CONTROLLER_HPP_
#define MOTION_CONTROLLER_PKG__MOTION_CONTROLLER_HPP_

// C++ standard libraries
#include <memory>
#include <chrono>
#include <string>
#include <vector>
#include <functional>
#include <pluginlib/class_loader.hpp>
#include <cstdlib>                                      // for std::setenv
#include <thread>
#include <Eigen/Dense>
#include <cmath>                                        // for std::atan2
#include <mutex>

// ROS2 libraries
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "motion_controller_pkg/msg/goal_command.hpp"
#include "motion_controller_pkg/action/goal_point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

// TF2 libraries
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>  
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 

// Aerostack2 libraries
#include "as2_msgs/msg/control_mode.hpp"
#include "as2_core/node.hpp"
#include "as2_motion_controller/controller_base.hpp"



/*!*******************************************************************************************
 *  \file       motion_controller.hpp
 *  \brief      Controller server class definition
 *  \authors    Miguel Tejado García
 ********************************************************************************************/



class PidControllerNode : public as2::Node
{
  using GoalCommand = motion_controller_pkg::msg::GoalCommand;
  using GoalPoint = motion_controller_pkg::action::GoalPoint;

public:
    PidControllerNode();
    void initTfListener();
    ~PidControllerNode();

private:
    // Callbacks for the subscribers and timer
    void poseCallback(int drone_id, const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void twistCallback(int drone_id, const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void timerCallback(int drone_id);


    // Action server callbacks
    rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const GoalPoint::Goal> goal);

    void cleanupGoal(int drone_id, bool cleanup_controller);

    rclcpp_action::CancelResponse handleCancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoalPoint>> goal_handle);

    void handleAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoalPoint>> goal_handle);
    
    void executeGoal(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoalPoint>> goal_handle);

    void trajCallback(const GoalCommand & msg);


    // Per-UAV mutexes (in the rare case that the callbacks get uncoordinated when checking the status of the UAVs)
    std::array<std::mutex, 4> drone_mutexes_;


    // Member variables
    std::array<rclcpp_action::Server<GoalPoint>::SharedPtr, 4> nav_servers_;
    std::array<std::shared_ptr<rclcpp_action::ServerGoalHandle<GoalPoint>>, 4> active_goals_;

    std::unique_ptr<pluginlib::ClassLoader<as2_motion_controller_plugin_base::ControllerBase>> plugin_loader_;
    std::array<std::shared_ptr<as2_motion_controller_plugin_base::ControllerBase>, 4> controllers_;

    rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data))
              .reliability(rclcpp::ReliabilityPolicy::BestEffort)
              .durability(rclcpp::DurabilityPolicy::Volatile);                                  // QoS for AS2 subscribers and publishers

    std::array<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr, 4> pose_subscribers_;
    std::array<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr, 4> twist_subscribers_;
    std::array<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr, 4> traj_subscribers_;
    std::array<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr, 4> twist_publishers_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2_ros::Buffer tf_buffer_;   // no need of mapping since tf2_ros::Buffer is thread-safe

    std::array<rclcpp::TimerBase::SharedPtr, 4> timers_;

    std::array<std::vector<rclcpp::Parameter>, 4> params_;

    std::array<as2_msgs::msg::ControlMode, 4> input_mode_;
    as2_msgs::msg::ControlMode output_mode_;                  // Keep as unique for ease of use (see its description in .cpp file)

    std::array<geometry_msgs::msg::PoseStamped, 4> current_pose_;
    std::array<geometry_msgs::msg::TwistStamped, 4> current_twist_;
    std::array<as2_msgs::msg::TrajectorySetpoints, 4> desired_traj_;
    std::array<bool, 4> traj_goal_defined_;
    std::array<bool, 4> not_circular_;
    std::array<as2_msgs::msg::TrajectorySetpoints, 4> circular_traj_;

    // "static constexpr" allows compile-time evaluation and optimization (one-shared copy)
    static constexpr double distance_threshold_ = 0.2;   
    static constexpr double circular_threshold_ = 0.15;
    static constexpr int num_points_ = 30;
};

#endif  // MOTION_CONTROLLER_PKG__MOTION_CONTROLLER_HPP_

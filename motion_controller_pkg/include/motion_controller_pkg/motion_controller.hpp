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
public:
    PidControllerNode();
    void initTfListener();
    ~PidControllerNode();

private:
    // Callbacks for the subscribers and timer
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void timerCallback();


    // Action server callbacks
    rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const motion_controller_pkg::action::GoalPoint::Goal> goal);
    
    rclcpp_action::CancelResponse handleCancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_controller_pkg::action::GoalPoint>> goal_handle);
    
    void handleAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_controller_pkg::action::GoalPoint>> goal_handle);
    
    void executeGoal(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_controller_pkg::action::GoalPoint>> goal_handle);

    void trajCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);


    // Member variables
    rclcpp_action::Server<motion_controller_pkg::action::GoalPoint>::SharedPtr nav_server_;

    std::unique_ptr<pluginlib::ClassLoader<as2_motion_controller_plugin_base::ControllerBase>> plugin_loader_;
    std::shared_ptr<as2_motion_controller_plugin_base::ControllerBase> controller_plugin_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr traj_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2_ros::Buffer tf_buffer_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::string> axes_;
    std::vector<rclcpp::Parameter> params_;

    as2_msgs::msg::ControlMode input_mode_;
    as2_msgs::msg::ControlMode output_mode_;

    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::TwistStamped current_twist_;
    as2_msgs::msg::TrajectorySetpoints desired_traj_;
    bool traj_goal_defined_;
    as2_msgs::msg::TrajectorySetpoints circular_traj_;

    geometry_msgs::msg::PoseStamped unused_pose_;
    geometry_msgs::msg::TwistStamped command_twist_;
    as2_msgs::msg::Thrust unused_thrust_;

    const double distance_threshold_ = 0.2;
    const double circular_threshold_ = 0.15;
    const int num_points_ = 20;
    const int radius_ = 3;
};

#endif  // MOTION_CONTROLLER_PKG__MOTION_CONTROLLER_HPP_

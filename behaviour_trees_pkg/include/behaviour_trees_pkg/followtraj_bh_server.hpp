#ifndef BEHAVIOUR_TREES_PKG__FOLLOWTRAJ_BH_SERVER_HPP
#define BEHAVIOUR_TREES_PKG__FOLLOWTRAJ_BH_SERVER_HPP

#include <mutex>
#include <array>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "as2_msgs/msg/trajectory_setpoints.hpp"

#include "as2_behavior/behavior_server.hpp"
#include "behaviour_trees_pkg/action/follow_traj_bh.hpp"
#include "motion_controller_pkg/msg/goal_command.hpp"

class FollowTrajServer : public as2_behavior::BehaviorServer<behaviour_trees_pkg::action::FollowTrajBh>
{
  using FollowTrajBh = behaviour_trees_pkg::action::FollowTrajBh;
  using GoalCommand = motion_controller_pkg::msg::GoalCommand;

public:
  FollowTrajServer();
  ~FollowTrajServer();

protected:
  bool on_activate(std::shared_ptr<const FollowTrajBh::Goal> goal) override;
  bool on_modify(std::shared_ptr<const FollowTrajBh::Goal> goal) override;
  bool on_deactivate(const std::shared_ptr<std::string>& msg) override;
  bool on_pause(const std::shared_ptr<std::string>& msg) override;
  bool on_resume(const std::shared_ptr<std::string>& msg) override;
  as2_behavior::ExecutionStatus on_run(
      const std::shared_ptr<const FollowTrajBh::Goal>& goal,
      std::shared_ptr<FollowTrajBh::Feedback>& feedback,
      std::shared_ptr<FollowTrajBh::Result>& result) override;
  void on_execution_end(const as2_behavior::ExecutionStatus& state) override;

private:
  void poseCallback(int drone_id, const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void twistCallback(int drone_id, const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void setupGoalFromTrajectory();
  bool isAtPosition(const geometry_msgs::msg::PoseStamped& current,
                    const as2_msgs::msg::TrajectoryPoint& target);

  rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data))
      .reliability(rclcpp::ReliabilityPolicy::BestEffort)
      .durability(rclcpp::DurabilityPolicy::Volatile);

  std::array<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr, 4> pose_subscribers_;
  std::array<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr, 4> twist_subscribers_;
  rclcpp::Publisher<GoalCommand>::SharedPtr goal_publisher_;

  std::array<geometry_msgs::msg::PoseStamped, 4> current_pose_;
  std::array<geometry_msgs::msg::TwistStamped, 4> current_twist_;
  std::array<as2_msgs::msg::TrajectorySetpoints, 4> goal_trajectory_;
  std::array<geometry_msgs::msg::PoseStamped, 4> goal_pose_;
  std::array<bool, 4>                         need_height_update_;
  std::array<std::mutex, 4>                   drone_mutexes_;

  GoalCommand goal_command_;

  double distance_threshold_{0.1};
  double height_threshold_{0.1};
};

#endif  // BEHAVIOUR_TREES_PKG__FOLLOWTRAJ_BH_SERVER_HPP

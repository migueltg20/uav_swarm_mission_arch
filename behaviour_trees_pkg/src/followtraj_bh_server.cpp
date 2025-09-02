#include "behaviour_trees_pkg/followtraj_bh_server.hpp"

using namespace std::chrono_literals;



FollowTrajServer::FollowTrajServer() : BehaviorServer("follow_traj")  
{
  for (int i = 0; i < 4; ++i) 
  {
    /* Subscribers, publishers and timer */
    pose_subscribers_[i] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/drone" + std::to_string(i) + "/self_localization/pose", qos_,
      [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        this->poseCallback(i, msg);
      });

    twist_subscribers_[i] = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/drone" + std::to_string(i) + "/self_localization/twist", qos_,
      [this, i](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        this->twistCallback(i, msg);
      });
  }

  goal_publisher_ = this->create_publisher<GoalCommand>("/goal", rclcpp::QoS(10));
}



FollowTrajServer::~FollowTrajServer() {}



bool FollowTrajServer::on_activate(std::shared_ptr<const FollowTrajBh::Goal> goal)
{
  if (goal->drone_id >= 4) 
  {
    RCLCPP_ERROR(this->get_logger(), "[FollowTrajBh] Invalid drone_id: %d", goal->drone_id);
    return false;  
  }

  goal_command_.drone_id = goal->drone_id;
  goal_command_.trajectory = goal->trajectory;
  goal_trajectory_[goal_command_.drone_id] = goal_command_.trajectory;
  goal_command_.radius = goal->radius;

  if (goal_command_.radius < 0.0) 
  {
    RCLCPP_ERROR(this->get_logger(), "[FollowTrajBh] Invalid radius for drone %d", goal_command_.drone_id);
    return false;  
  }
  else if (goal_command_.radius > 0.0) 
  {
    goal_command_.circular = true;

    if (goal_command_.trajectory.setpoints.size() == 1) 
    {
      setupGoalFromTrajectory();
    }
  }
  else if (goal_command_.radius == 0.0 && !goal_command_.trajectory.setpoints.empty())
  {
    if (goal_command_.trajectory.setpoints.size() == 1) 
    {
      setupGoalFromTrajectory();
    }
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "[FollowTrajBh] Invalid trajectory for drone %d", goal_command_.drone_id);
    return false;  
  }

  goal_publisher_->publish(goal_command_);
  return true;
}



bool FollowTrajServer::on_modify(std::shared_ptr<const FollowTrajBh::Goal> goal)
{
  std::lock_guard<std::mutex> lock(drone_mutexes_[goal->drone_id]);

  if (goal->drone_id >= 4) 
  {
    RCLCPP_ERROR(this->get_logger(), "[FollowTrajBh] Invalid drone_id: %d", goal->drone_id);
    return false;  
  }

  goal_command_.drone_id = goal->drone_id;
  goal_command_.trajectory = goal->trajectory;
  goal_trajectory_[goal_command_.drone_id] = goal_command_.trajectory;
  goal_pose_[goal_command_.drone_id] = geometry_msgs::msg::PoseStamped();
  goal_command_.radius = goal->radius;

  if (goal_command_.radius < 0.0) 
  {
    RCLCPP_ERROR(this->get_logger(), "[FollowTrajBh] Invalid radius for drone %d", goal->drone_id);
    return false;  
  }
  else if (goal_command_.radius > 0.0) 
  {
    goal_command_.circular = true;

    if (goal_command_.trajectory.setpoints.size() == 1) 
    {
      setupGoalFromTrajectory();
    }
    else if (goal_command_.trajectory.setpoints.size() == 0)
    {
      goal_command_.trajectory.setpoints.clear();
      goal_command_.point = geometry_msgs::msg::PoseStamped();
    }
  }
  else if (goal_command_.radius == 0.0 && !goal_command_.trajectory.setpoints.empty())
  {
    goal_command_.circular = false;

    if (goal_command_.trajectory.setpoints.size() == 1) 
    {
      setupGoalFromTrajectory();
    }
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "[FollowTrajBh] Invalid trajectory for drone %d", goal_command_.drone_id);
    return false;  
  }

  goal_publisher_->publish(goal_command_);
  return true;
}



bool FollowTrajServer::on_deactivate(const std::shared_ptr<std::string> & /*msg*/)
{
  RCLCPP_WARN(this->get_logger(), "[FollowTrajBh] Cannot deactivate follow trajectory");
  return false;
}



bool FollowTrajServer::on_pause(const std::shared_ptr<std::string> & /*msg*/)
{
  RCLCPP_WARN(this->get_logger(), "[FollowTrajBh] Cannot pause follow trajectory");
  return false;
}



bool FollowTrajServer::on_resume(const std::shared_ptr<std::string> & /*msg*/)
{
  RCLCPP_WARN(this->get_logger(), "[FollowTrajBh] Cannot resume follow trajectory");
  return false;
}



as2_behavior::ExecutionStatus FollowTrajServer::on_run(
    const std::shared_ptr<const FollowTrajBh::Goal> & goal,
    std::shared_ptr<FollowTrajBh::Feedback> & feedback,
    std::shared_ptr<FollowTrajBh::Result> & result)
{
  std::lock_guard<std::mutex> lock(drone_mutexes_[goal->drone_id]);

  int drone_id = goal->drone_id;

  if (drone_id >= 4) 
  {
    RCLCPP_ERROR(this->get_logger(), "[FollowTrajBh] Invalid drone_id: %d", drone_id);
    result->follow_path_success = false;
    return as2_behavior::ExecutionStatus::FAILURE;
  }


  /* Check if we have a valid current pose */
  if (current_pose_[drone_id].header.frame_id.empty()) 
  {
    RCLCPP_WARN(this->get_logger(), "[FollowTrajBh] Waiting for pose data for drone %d", drone_id);
    return as2_behavior::ExecutionStatus::RUNNING;
  }


  /* Handle circular trajectory (no setpoints to track) */
  if (goal_trajectory_[drone_id].setpoints.empty() && goal_pose_[drone_id].header.frame_id.empty()) 
  {
    // For circular trajectories, check if drone reached the center point
    auto& target = goal_trajectory_[drone_id].setpoints[0];
    if (isAtPosition(current_pose_[drone_id], target)) 
    {
      RCLCPP_INFO(this->get_logger(), "[FollowTrajBh] Circular trajectory for UAV %d completed.", drone_id);
      result->follow_path_success = true;
      return as2_behavior::ExecutionStatus::SUCCESS;
    }
    else 
    {
      RCLCPP_INFO(this->get_logger(), "[FollowTrajBh] Circular trajectory for UAV %d in progress.", drone_id);
      return as2_behavior::ExecutionStatus::RUNNING;
    }
  }


  /* Handle multi-point trajectory */
  if (goal_trajectory_[drone_id].setpoints.size() > 1) 
  {
    auto& target = goal_trajectory_[drone_id].setpoints[0];
    if (isAtPosition(current_pose_[drone_id], target)) 
    {
      goal_trajectory_[drone_id].setpoints.erase(goal_trajectory_[drone_id].setpoints.begin());
      
      if (goal_trajectory_[drone_id].setpoints.size() <= 1) 
      {
        RCLCPP_INFO(this->get_logger(), "[FollowTrajBh] Trajectory for UAV %d completed.", drone_id);
        result->follow_path_success = true;
        return as2_behavior::ExecutionStatus::SUCCESS;
      }
    }
  }
  /* Handle single point trajectory */
  else if (goal_trajectory_[drone_id].setpoints.size() == 1) 
  {
    auto& target = goal_trajectory_[drone_id].setpoints[0];
    if (isAtPosition(current_pose_[drone_id], target)) 
    {
      RCLCPP_INFO(this->get_logger(), "[FollowTrajBh] Trajectory for UAV %d completed.", drone_id);
      result->follow_path_success = true;
      return as2_behavior::ExecutionStatus::SUCCESS;
    }
  }

  /* Provide feedback */
  if (!current_twist_[drone_id].header.frame_id.empty() && !current_pose_[drone_id].header.frame_id.empty())
  {
    feedback->actual_speed = current_twist_[drone_id];
    feedback->remaining_waypoints = goal_trajectory_[drone_id].setpoints.size();
    feedback->actual_distance_to_next_waypoint = std::hypot(
        current_pose_[drone_id].pose.position.x - goal_trajectory_[drone_id].setpoints[0].position.x,
        current_pose_[drone_id].pose.position.y - goal_trajectory_[drone_id].setpoints[0].position.y,
        current_pose_[drone_id].pose.position.z - goal_trajectory_[drone_id].setpoints[0].position.z);
  }

  return as2_behavior::ExecutionStatus::RUNNING;
}



void FollowTrajServer::on_execution_end(const as2_behavior::ExecutionStatus & state)
{
  if (state == as2_behavior::ExecutionStatus::SUCCESS) 
  {
    RCLCPP_INFO(this->get_logger(), "[FollowTrajBh] Completed successfully");
  } 
  else 
  {
    RCLCPP_ERROR(this->get_logger(), "[FollowTrajBh] Terminated with failure");
  }
}



void FollowTrajServer::poseCallback(int drone_id, const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(drone_mutexes_[drone_id]);

  if (drone_id >= 0 && drone_id < 4) 
  {
    current_pose_[drone_id] = *msg;
  }
}

void FollowTrajServer::twistCallback(int drone_id, const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(drone_mutexes_[drone_id]);

  if (drone_id >= 0 && drone_id < 4) 
  {
    current_twist_[drone_id] = *msg;
  }
}



void FollowTrajServer::setupGoalFromTrajectory() 
{
  if (goal_command_.trajectory.setpoints.size() == 1) 
  {
    goal_command_.point.header = goal_command_.trajectory.header;
    auto& setpoint = goal_command_.trajectory.setpoints.front();
    goal_command_.point.pose.position.x = setpoint.position.x;
    goal_command_.point.pose.position.y = setpoint.position.y;
    goal_command_.point.pose.position.z = setpoint.position.z;

    double yaw = setpoint.yaw_angle;
    goal_command_.point.pose.orientation.x = 0.0;
    goal_command_.point.pose.orientation.y = 0.0;
    goal_command_.point.pose.orientation.z = std::sin(yaw / 2.0);
    goal_command_.point.pose.orientation.w = std::cos(yaw / 2.0);
    
    goal_command_.trajectory.setpoints.clear();
    goal_pose_[goal_command_.drone_id] = goal_command_.point;
  }
}



bool FollowTrajServer::isAtPosition(const geometry_msgs::msg::PoseStamped& current, const as2_msgs::msg::TrajectoryPoint& target) {
  return (std::abs(current.pose.position.x - target.position.x) <= distance_threshold_ &&
          std::abs(current.pose.position.y - target.position.y) <= distance_threshold_ &&
          std::abs(current.pose.position.z - target.position.z) <= height_threshold_);
}



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowTrajServer>());
  rclcpp::shutdown();
  return 0;
}
#include "behaviour_trees_pkg/followtraj_bh_server.hpp"

using namespace std::chrono_literals;

/*!*******************************************************************************************
 *  \file       followtraj_bh_server.cpp
 *  \brief      Follow trajectory behaviour server implementation
 *  \authors    Miguel Tejado García
 ********************************************************************************************/

/* Constructor - Initialize subscribers for all drones and goal publisher */
FollowTrajServer::FollowTrajServer() : BehaviorServer("follow_traj")  
{
  /* Initialize subscribers for up to 4 drones */
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

  /* Publisher for sending goal commands to trajectory controller */
  goal_publisher_ = this->create_publisher<GoalCommand>("/goal", rclcpp::QoS(10));
}



FollowTrajServer::~FollowTrajServer() {}



/* Activate behavior with new trajectory goal */
bool FollowTrajServer::on_activate(std::shared_ptr<const FollowTrajBh::Goal> goal)
{
  /* Validate drone ID is within supported range */
  if (goal->drone_id >= 4) 
  {
    RCLCPP_ERROR(this->get_logger(), "[FollowTrajBh] Invalid drone_id: %d", goal->drone_id);
    return false;  
  }

  /* Set up goal command from received goal */
  goal_command_.drone_id = goal->drone_id;
  goal_command_.trajectory = goal->trajectory;
  goal_trajectory_[goal_command_.drone_id] = goal_command_.trajectory; // Store local copy for tracking
  goal_command_.radius = goal->radius;

  need_height_update_[goal->drone_id] = true; // Flag to update Z coordinates with current height

  /* Validate radius parameter */
  if (goal_command_.radius < 0.0) 
  {
    RCLCPP_ERROR(this->get_logger(), "[FollowTrajBh] Invalid radius for drone %d", goal_command_.drone_id);
    return false;  
  }
  else if (goal_command_.radius > 0.0) 
  {
    goal_command_.circular = true; // Enable circular trajectory mode

    /* Convert single setpoint to goal point for circular motion */
    if (goal_command_.trajectory.setpoints.size() == 1) 
    {
      setupGoalFromTrajectory();
    }
  }
  else if (goal_command_.radius == 0.0 && !goal_command_.trajectory.setpoints.empty())
  {
    /* Handle regular trajectory following */
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

  /* Publish goal command to trajectory controller */
  goal_publisher_->publish(goal_command_);
  return true;
}



/* Modify existing trajectory goal with new parameters */
bool FollowTrajServer::on_modify(std::shared_ptr<const FollowTrajBh::Goal> goal)
{
  std::lock_guard<std::mutex> lock(drone_mutexes_[goal->drone_id]); // Thread safety

  /* Validate drone ID */
  if (goal->drone_id >= 4) 
  {
    RCLCPP_ERROR(this->get_logger(), "[FollowTrajBh] Invalid drone_id: %d", goal->drone_id);
    return false;  
  }

  /* Update goal command with new trajectory */
  goal_command_.drone_id = goal->drone_id;
  goal_command_.trajectory = goal->trajectory;
  goal_trajectory_[goal_command_.drone_id] = goal_command_.trajectory;
  goal_pose_[goal_command_.drone_id] = geometry_msgs::msg::PoseStamped(); // Reset goal pose
  goal_command_.radius = goal->radius;
  
  need_height_update_[goal->drone_id] = true; // Flag for height update

  /* Process radius parameter and set trajectory mode */
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
    else if (goal_command_.trajectory.setpoints.size() == 0)
    {
      /* Clear trajectory for empty circular motion */
      goal_command_.trajectory.setpoints.clear();
      goal_command_.point = geometry_msgs::msg::PoseStamped();
    }
  }
  else if (goal_command_.radius == 0.0 && !goal_command_.trajectory.setpoints.empty())
  {
    goal_command_.circular = false; // Regular trajectory mode

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

  /* Publish updated goal command */
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



/* Main execution loop - check trajectory progress and provide feedback */
as2_behavior::ExecutionStatus FollowTrajServer::on_run(
    const std::shared_ptr<const FollowTrajBh::Goal> & goal,
    std::shared_ptr<FollowTrajBh::Feedback> & feedback,
    std::shared_ptr<FollowTrajBh::Result> & result)
{
  std::lock_guard<std::mutex> lock(drone_mutexes_[goal->drone_id]); // Thread safety

  int drone_id = goal->drone_id;

  /* Validate drone ID */
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

  /* Update Z coordinates with current height if needed */
  if (need_height_update_[drone_id]) 
  {
    /* Update all setpoints to current height */
    for (auto& setpoint : goal_trajectory_[drone_id].setpoints) 
    {
      setpoint.position.z = current_pose_[drone_id].pose.position.z;
    }
    for (auto& setpoint : goal_command_.trajectory.setpoints) 
    {
      setpoint.position.z = current_pose_[drone_id].pose.position.z;
    }
    need_height_update_[drone_id] = false;
    goal_publisher_->publish(goal_command_); // Republish with updated heights
  }


  /* Handle circular trajectory (no setpoints to track) */
  if (goal_trajectory_[drone_id].setpoints.empty() && goal_pose_[drone_id].header.frame_id.empty()) 
  {
    /* For circular trajectories, check if drone reached the center point */
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
  if (goal_trajectory_[drone_id].setpoints.size() >= 1) 
  {
    auto& target = goal_trajectory_[drone_id].setpoints[0];
    if (isAtPosition(current_pose_[drone_id], target)) 
    {
      /* Remove completed waypoint */
      goal_trajectory_[drone_id].setpoints.erase(goal_trajectory_[drone_id].setpoints.begin());
      
      /* Check if trajectory is complete */
      if (goal_trajectory_[drone_id].setpoints.empty()) 
      {
        RCLCPP_INFO(this->get_logger(), "[FollowTrajBh] Trajectory for UAV %d completed.", drone_id);
        result->follow_path_success = true;
        return as2_behavior::ExecutionStatus::SUCCESS;
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "[FollowTrajBh] UAV %d reached waypoint, %zu remaining", 
                    drone_id, goal_trajectory_[drone_id].setpoints.size());
      }
    }
    else
    {
      /* Add debug info about current progress */
      double distance = std::hypot(
          current_pose_[drone_id].pose.position.x - target.position.x,
          current_pose_[drone_id].pose.position.y - target.position.y,
          current_pose_[drone_id].pose.position.z - target.position.z);
      RCLCPP_DEBUG(this->get_logger(), "[FollowTrajBh] UAV %d distance to waypoint: %.2f", drone_id, distance);
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
  else
  {
    /* No waypoints left - this should have been caught earlier */
    RCLCPP_WARN(this->get_logger(), "[FollowTrajBh] No waypoints remaining for UAV %d", drone_id);
    result->follow_path_success = true;
    return as2_behavior::ExecutionStatus::SUCCESS;
  }

  /* Provide feedback on current status */
  if (!current_twist_[drone_id].header.frame_id.empty() && !current_pose_[drone_id].header.frame_id.empty())
  {
    feedback->actual_speed = current_twist_[drone_id];
    feedback->remaining_waypoints = goal_trajectory_[drone_id].setpoints.size();
    /* Calculate 3D distance to next waypoint */
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



/* Callback to receive pose updates from specific drone */
void FollowTrajServer::poseCallback(int drone_id, const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(drone_mutexes_[drone_id]); // Thread safety

  /* Store pose data for valid drone IDs */
  if (drone_id >= 0 && drone_id < 4) 
  {
    current_pose_[drone_id] = *msg;
  }
}

/* Callback to receive twist updates from specific drone */
void FollowTrajServer::twistCallback(int drone_id, const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(drone_mutexes_[drone_id]); // Thread safety

  /* Store twist data for valid drone IDs */
  if (drone_id >= 0 && drone_id < 4) 
  {
    current_twist_[drone_id] = *msg;
  }
}



/* Convert single trajectory setpoint to goal pose command */
void FollowTrajServer::setupGoalFromTrajectory() 
{
  if (goal_command_.trajectory.setpoints.size() == 1) 
  {
    /* Set up pose header */
    goal_command_.point.header = goal_command_.trajectory.header;
    auto& setpoint = goal_command_.trajectory.setpoints.front();
    
    /* Copy position coordinates */
    goal_command_.point.pose.position.x = setpoint.position.x;
    goal_command_.point.pose.position.y = setpoint.position.y;
    goal_command_.point.pose.position.z = setpoint.position.z;

    /* Convert yaw angle to quaternion orientation */
    double yaw = setpoint.yaw_angle;
    goal_command_.point.pose.orientation.x = 0.0;
    goal_command_.point.pose.orientation.y = 0.0;
    goal_command_.point.pose.orientation.z = std::sin(yaw / 2.0);
    goal_command_.point.pose.orientation.w = std::cos(yaw / 2.0);
    
    /* Clear trajectory setpoints and store goal pose */
    goal_command_.trajectory.setpoints.clear();
    goal_pose_[goal_command_.drone_id] = goal_command_.point;
  }
}



/* Check if current position is within threshold of target position */
bool FollowTrajServer::isAtPosition(const geometry_msgs::msg::PoseStamped& current, const as2_msgs::msg::TrajectoryPoint& target) {
  return (std::abs(current.pose.position.x - target.position.x) <= distance_threshold_ &&
          std::abs(current.pose.position.y - target.position.y) <= distance_threshold_ &&
          std::abs(current.pose.position.z - target.position.z) <= height_threshold_);
}



/* Main function - Initialize ROS2 node and spin */
int main(int argc, char **argv)
{
  /* Disable Fast RTPS shared memory transport */
  ::setenv("RMW_FASTRTPS_SHARED_MEMORY_ENABLED", "0", 1);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowTrajServer>()); // Run the behavior server
  rclcpp::shutdown();
  return 0;
}
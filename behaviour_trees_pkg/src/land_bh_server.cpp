#include "behaviour_trees_pkg/land_bh_server.hpp"

using namespace std::chrono_literals;

/*!*******************************************************************************************
 *  \file       land_bh_server.cpp
 *  \brief      Land behaviour server implementation
 *  \authors    Miguel Tejado García
 ********************************************************************************************/

/* Constructor: Initialize subscribers and publishers for up to 4 drones */
LandServer::LandServer() : BehaviorServer("land")  
{
  /* Initialize data structures for each drone in the swarm */
  for (int i = 0; i < 4; ++i) 
  {
    current_height_[i] = 0.0f;                                // In case "on_run" goes first than callbacks
    current_twist_[i] = geometry_msgs::msg::TwistStamped();

    /* Create pose subscriber for each drone to track altitude */
    pose_subscribers_[i] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/drone" + std::to_string(i) + "/self_localization/pose", qos_,
      [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        this->poseCallback(i, msg);
      });

    /* Create twist subscriber for each drone to track velocity */
    twist_subscribers_[i] = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/drone" + std::to_string(i) + "/self_localization/twist", qos_,
      [this, i](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        this->twistCallback(i, msg);
      });
  }

  /* Publisher to send landing commands to the flight controller */
  goal_publisher_ = this->create_publisher<GoalCommand>("/goal", rclcpp::QoS(10));
}



LandServer::~LandServer() {}



/* Activate landing behavior for specified drone */
bool LandServer::on_activate(std::shared_ptr<const LandBh::Goal> goal)
{
  /* Validate drone ID is within valid range */
  if (goal->drone_id >= 4) 
  {
    RCLCPP_ERROR(this->get_logger(), "[LandBh] Invalid drone_id: %d", goal->drone_id);
    return false;  
  }

  initial_height_set_[goal->drone_id] = false; // Reset initial height flag

  landing_start_time_ = this->get_clock()->now(); // Record landing start time for timeout

  /* Prepare and publish landing command */
  goal_command_.drone_id = goal->drone_id;
  goal_command_.land = goal->land;

  goal_publisher_->publish(goal_command_);
  return true;
}

/* Modify active landing behavior parameters */
bool LandServer::on_modify(std::shared_ptr<const LandBh::Goal> goal)
{
  std::lock_guard<std::mutex> lock(drone_mutexes_[goal->drone_id]); // Thread safety

  /* Validate drone ID is within valid range */
  if (goal->drone_id >= 4) 
  {
    RCLCPP_ERROR(this->get_logger(), "[LandBh] Invalid drone_id: %d", goal->drone_id);
    return false;  
  }

  initial_height_set_[goal->drone_id] = false; // Reset initial height flag

  /* Update and republish landing command */
  goal_command_.drone_id = goal->drone_id;
  goal_command_.land = goal->land;

  goal_publisher_->publish(goal_command_);
  return true;
}



/* Landing cannot be deactivated once started for safety reasons */
bool LandServer::on_deactivate(const std::shared_ptr<std::string> & /*msg*/)
{
  RCLCPP_WARN(this->get_logger(), "[LandBh] Cannot deactivate landing");
  return false;
}



/* Landing cannot be paused once started for safety reasons */
bool LandServer::on_pause(const std::shared_ptr<std::string> & /*msg*/)
{
  RCLCPP_WARN(this->get_logger(), "[LandBh] Cannot pause landing");
  return false;
}



/* Landing cannot be resumed since it cannot be paused */
bool LandServer::on_resume(const std::shared_ptr<std::string> & /*msg*/)
{
  RCLCPP_WARN(this->get_logger(), "[LandBh] Cannot resume landing");
  return false;
}



/* Main execution loop - monitors landing progress and determines completion */
as2_behavior::ExecutionStatus LandServer::on_run(
    const std::shared_ptr<const LandBh::Goal> & goal,
    std::shared_ptr<LandBh::Feedback> & feedback,
    std::shared_ptr<LandBh::Result> & result)
{
  std::lock_guard<std::mutex> lock(drone_mutexes_[goal->drone_id]); // Thread safety

  /* Validate drone ID is within valid range */
  if (goal->drone_id >= 4) 
  {
    RCLCPP_ERROR(this->get_logger(), "[LandBh] Invalid drone_id: %d", goal->drone_id);
    result->land_success = false;
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  /* Wait for initial pose data before proceeding */
  if (!initial_height_set_[goal->drone_id]) 
  {
    RCLCPP_WARN(this->get_logger(), "[LandBh] Waiting for initial pose data for drone %d", goal->drone_id);
    return as2_behavior::ExecutionStatus::RUNNING;
  }

  /* Check for landing timeout (10 seconds) */
  auto elapsed = this->get_clock()->now() - landing_start_time_;
  if (elapsed.seconds() > 10.0 && current_height_[goal->drone_id] >= initial_height_[goal->drone_id] - land_threshold_)   // 10 second timeout
  {  
    RCLCPP_ERROR(this->get_logger(), "[LandBh] Landing timeout for drone %d", goal->drone_id);
    result->land_success = false;
    initial_height_set_[goal->drone_id] = false;
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  /* Update feedback with current landing status */
  feedback->actual_land_height = current_height_[goal->drone_id];
  feedback->actual_land_speed = current_twist_[goal->drone_id];

  /* Check if drone has reached landing threshold height */
  if(current_height_[goal->drone_id] <= land_threshold_)
  {
    RCLCPP_INFO(this->get_logger(), "[LandBh] Landing successful");
    result->land_success = true;
    initial_height_set_[goal->drone_id] = false;
    return as2_behavior::ExecutionStatus::SUCCESS;
  }

  return as2_behavior::ExecutionStatus::RUNNING; // Continue landing
}



/* Clean up after landing execution completes */
void LandServer::on_execution_end(const as2_behavior::ExecutionStatus & state)
{
  /* Reset all initial height flags for all drones */
  for (int i = 0; i < 4; ++i) 
  {
    initial_height_set_[i] = false;
  }

  /* Log final execution status */
  if (state == as2_behavior::ExecutionStatus::SUCCESS) 
  {
    RCLCPP_INFO(this->get_logger(), "[LandBh] Completed successfully");
  } 
  else 
  {
    RCLCPP_ERROR(this->get_logger(), "[LandBh] Terminated with failure");
  }
}



/* Callback to receive and process drone pose updates */
void LandServer::poseCallback(int drone_id, const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(drone_mutexes_[drone_id]); // Thread safety

  /* Validate drone ID and update height data */
  if (drone_id >= 0 && drone_id < 4) 
  {
    current_height_[drone_id] = msg->pose.position.z; // Extract Z coordinate (altitude)

    /* Set initial height reference point for landing progress calculation */
    if (!initial_height_set_[drone_id]) 
    {
      initial_height_[drone_id] = current_height_[drone_id];
      initial_height_set_[drone_id] = true;
    }
  }
}

/* Callback to receive and process drone velocity updates */
void LandServer::twistCallback(int drone_id, const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(drone_mutexes_[drone_id]); // Thread safety

  /* Validate drone ID and update velocity data */
  if (drone_id >= 0 && drone_id < 4) 
  {
    current_twist_[drone_id] = *msg; // Store complete twist message
  }
}



/* Main function - entry point for the landing behavior server */
int main(int argc, char **argv)
{
  // Disable Fast RTPS shared memory transport
  ::setenv("RMW_FASTRTPS_SHARED_MEMORY_ENABLED", "0", 1);

  rclcpp::init(argc, argv);                              // Initialize ROS2
  rclcpp::spin(std::make_shared<LandServer>());          // Start server and process callbacks
  rclcpp::shutdown();                                    // Clean shutdown
  return 0;
}
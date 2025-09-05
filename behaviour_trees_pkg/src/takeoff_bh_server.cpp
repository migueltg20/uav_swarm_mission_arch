#include "behaviour_trees_pkg/takeoff_bh_server.hpp"

using namespace std::chrono_literals;

/*!*******************************************************************************************
 *  \file       takeoff_bh_server.cpp
 *  \brief      Take off behaviour server implementation for drone swarm operations
 *  \authors    Miguel Tejado García
 ********************************************************************************************/

/* Constructor: Initialize takeoff behavior server with subscribers and publishers for drone swarm */
TakeOffServer::TakeOffServer() : BehaviorServer("take_off")  
{
  // Initialize data structures for up to 4 drones
  for (int i = 0; i < 4; ++i) 
  {
    current_height_[i] = 0.0f;                                // Initialize height to ground level
    current_twist_[i] = geometry_msgs::msg::TwistStamped();   // Initialize velocity data

    /* Create pose subscribers for each drone to monitor position */
    pose_subscribers_[i] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/drone" + std::to_string(i) + "/self_localization/pose", qos_,
      [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        this->poseCallback(i, msg);
      });

    /* Create twist subscribers for each drone to monitor velocity */
    twist_subscribers_[i] = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/drone" + std::to_string(i) + "/self_localization/twist", qos_,
      [this, i](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        this->twistCallback(i, msg);
      });
  }

  /* Publisher for sending takeoff goal commands */
  goal_publisher_ = this->create_publisher<GoalCommand>("/goal", rclcpp::QoS(10));
}



/* Destructor */
TakeOffServer::~TakeOffServer() {}



/* Activate takeoff behavior - validates drone ID and publishes takeoff command */
bool TakeOffServer::on_activate(std::shared_ptr<const TakeOffBh::Goal> goal)
{
  // Validate drone ID is within supported range
  if (goal->drone_id >= 4) 
  {
    RCLCPP_ERROR(this->get_logger(), "[TakeOffBh] Invalid drone_id: %d", goal->drone_id);
    return false;  
  }

  takeoff_start_time_ = this->get_clock()->now();  // Record start time for timeout monitoring

  // Prepare and publish takeoff command
  goal_command_.drone_id = goal->drone_id;
  goal_command_.takeoff = goal->takeoff_height;

  goal_publisher_->publish(goal_command_);
  return true;
}



/* Modify takeoff parameters during execution */
bool TakeOffServer::on_modify(std::shared_ptr<const TakeOffBh::Goal> goal)
{
  std::lock_guard<std::mutex> lock(drone_mutexes_[goal->drone_id]);  // Thread-safe access

  // Validate drone ID
  if (goal->drone_id >= 4) 
  {
    RCLCPP_ERROR(this->get_logger(), "[TakeOffBh] Invalid drone_id: %d", goal->drone_id);
    return false;  
  }

  // Update and republish goal command with new parameters
  goal_command_.drone_id = goal->drone_id;
  goal_command_.takeoff = goal->takeoff_height;

  goal_publisher_->publish(goal_command_);
  return true;
}



/* Deactivate takeoff behavior - not allowed during takeoff */
bool TakeOffServer::on_deactivate(const std::shared_ptr<std::string> & /*msg*/)
{
  RCLCPP_WARN(this->get_logger(), "[TakeOffBh] Cannot deactivate takeoff");
  return false;
}



/* Pause takeoff behavior - not allowed during takeoff */
bool TakeOffServer::on_pause(const std::shared_ptr<std::string> & /*msg*/)
{
  RCLCPP_WARN(this->get_logger(), "[TakeOffBh] Cannot pause takeoff");
  return false;
}



/* Resume takeoff behavior - not allowed during takeoff */
bool TakeOffServer::on_resume(const std::shared_ptr<std::string> & /*msg*/)
{
  RCLCPP_WARN(this->get_logger(), "[TakeOffBh] Cannot resume takeoff");
  return false;
}



/* Main execution loop - monitors takeoff progress and determines completion status */
as2_behavior::ExecutionStatus TakeOffServer::on_run(
    const std::shared_ptr<const TakeOffBh::Goal> & goal,
    std::shared_ptr<TakeOffBh::Feedback> & feedback,
    std::shared_ptr<TakeOffBh::Result> & result)
{
  std::lock_guard<std::mutex> lock(drone_mutexes_[goal->drone_id]);  // Thread-safe access

  // Validate drone ID
  if (goal->drone_id >= 4) 
  {
    RCLCPP_ERROR(this->get_logger(), "[TakeOffBh] Invalid drone_id: %d", goal->drone_id);
    result->takeoff_success = false;
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  // Check for timeout (10 seconds max if drone hasn't reached minimum height)
  auto elapsed = this->get_clock()->now() - takeoff_start_time_;
  if (elapsed.seconds() > 10.0 && current_height_[goal->drone_id] <= takeoff_threshold_)
  {  
    RCLCPP_ERROR(this->get_logger(), "[TakeOffBh] Takeoff timeout for drone %d", goal->drone_id);
    result->takeoff_success = false;
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  // Update feedback with current drone state
  feedback->actual_takeoff_height = current_height_[goal->drone_id];
  feedback->actual_takeoff_speed = current_twist_[goal->drone_id];

  // Check if drone has reached target height within tolerance
  if(current_height_[goal->drone_id] >= goal->takeoff_height - takeoff_threshold_ &&
    current_height_[goal->drone_id] <= goal->takeoff_height + takeoff_threshold_)
  {
    RCLCPP_INFO(this->get_logger(), "[TakeOffBh] Takeoff successful");
    result->takeoff_success = true;
    return as2_behavior::ExecutionStatus::SUCCESS;
  }

  return as2_behavior::ExecutionStatus::RUNNING;  // Continue execution
}



/* Called when execution ends - logs final status */
void TakeOffServer::on_execution_end(const as2_behavior::ExecutionStatus & state)
{
  if (state == as2_behavior::ExecutionStatus::SUCCESS) 
  {
    RCLCPP_INFO(this->get_logger(), "[TakeOffBh] Completed successfully");
  } 
  else 
  {
    RCLCPP_ERROR(this->get_logger(), "[TakeOffBh] Terminated with failure");
  }
}



/* Callback to update drone position data from pose messages */
void TakeOffServer::poseCallback(int drone_id, const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(drone_mutexes_[drone_id]);  // Thread-safe access

  if (drone_id >= 0 && drone_id < 4)  // Validate drone ID range
  {
    current_height_[drone_id] = msg->pose.position.z;  // Extract Z coordinate (height)
  }
}

/* Callback to update drone velocity data from twist messages */
void TakeOffServer::twistCallback(int drone_id, const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(drone_mutexes_[drone_id]);  // Thread-safe access

  if (drone_id >= 0 && drone_id < 4)  // Validate drone ID range
  {
    current_twist_[drone_id] = *msg;  // Store complete twist message
  }
}



/* Main function - initializes ROS2 node and starts takeoff server */
int main(int argc, char **argv)
{
  // Disable Fast RTPS shared memory transport for better compatibility
  ::setenv("RMW_FASTRTPS_SHARED_MEMORY_ENABLED", "0", 1);

  rclcpp::init(argc, argv);                                    // Initialize ROS2
  rclcpp::spin(std::make_shared<TakeOffServer>());            // Start server and process callbacks
  rclcpp::shutdown();                                         // Clean shutdown
  return 0;
}
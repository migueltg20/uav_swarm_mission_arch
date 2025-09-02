#include "behaviour_trees_pkg/takeoff_bh_server.hpp"

using namespace std::chrono_literals;



TakeOffServer::TakeOffServer() : BehaviorServer("take_off")  
{
  for (int i = 0; i < 4; ++i) 
  {
    current_height_[i] = 0.0f;                                // In case "on_run" goes first than callbacks
    current_twist_[i] = geometry_msgs::msg::TwistStamped();

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



TakeOffServer::~TakeOffServer() {}



bool TakeOffServer::on_activate(std::shared_ptr<const TakeOffBh::Goal> goal)
{
  if (goal->drone_id >= 4) 
  {
    RCLCPP_ERROR(this->get_logger(), "[TakeOffBh] Invalid drone_id: %d", goal->drone_id);
    return false;  
  }

  takeoff_start_time_ = this->get_clock()->now();

  goal_command_.drone_id = goal->drone_id;
  goal_command_.takeoff = goal->takeoff_height;

  goal_publisher_->publish(goal_command_);
  return true;
}



bool TakeOffServer::on_modify(std::shared_ptr<const TakeOffBh::Goal> goal)
{
  std::lock_guard<std::mutex> lock(drone_mutexes_[goal->drone_id]);

  if (goal->drone_id >= 4) 
  {
    RCLCPP_ERROR(this->get_logger(), "[TakeOffBh] Invalid drone_id: %d", goal->drone_id);
    return false;  
  }

  goal_command_.drone_id = goal->drone_id;
  goal_command_.takeoff = goal->takeoff_height;

  goal_publisher_->publish(goal_command_);
  return true;
}



bool TakeOffServer::on_deactivate(const std::shared_ptr<std::string> & /*msg*/)
{
  RCLCPP_WARN(this->get_logger(), "[TakeOffBh] Cannot deactivate takeoff");
  return false;
}



bool TakeOffServer::on_pause(const std::shared_ptr<std::string> & /*msg*/)
{
  RCLCPP_WARN(this->get_logger(), "[TakeOffBh] Cannot pause takeoff");
  return false;
}



bool TakeOffServer::on_resume(const std::shared_ptr<std::string> & /*msg*/)
{
  RCLCPP_WARN(this->get_logger(), "[TakeOffBh] Cannot resume takeoff");
  return false;
}



as2_behavior::ExecutionStatus TakeOffServer::on_run(
    const std::shared_ptr<const TakeOffBh::Goal> & goal,
    std::shared_ptr<TakeOffBh::Feedback> & feedback,
    std::shared_ptr<TakeOffBh::Result> & result)
{
  std::lock_guard<std::mutex> lock(drone_mutexes_[goal->drone_id]);

  if (goal->drone_id >= 4) 
  {
    RCLCPP_ERROR(this->get_logger(), "[TakeOffBh] Invalid drone_id: %d", goal->drone_id);
    result->takeoff_success = false;
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  auto elapsed = this->get_clock()->now() - takeoff_start_time_;
  if (elapsed.seconds() > 10.0 && current_height_[goal->drone_id] <= takeoff_threshold_)   // 10 second timeout
  {  
    RCLCPP_ERROR(this->get_logger(), "[TakeOffBh] Takeoff timeout for drone %d", goal->drone_id);
    result->takeoff_success = false;
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  feedback->actual_takeoff_height = current_height_[goal->drone_id];
  feedback->actual_takeoff_speed = current_twist_[goal->drone_id];

  if(current_height_[goal->drone_id] >= goal->takeoff_height - takeoff_threshold_ &&
    current_height_[goal->drone_id] <= goal->takeoff_height + takeoff_threshold_)
  {
    RCLCPP_INFO(this->get_logger(), "[TakeOffBh] Takeoff successful");
    result->takeoff_success = true;
    return as2_behavior::ExecutionStatus::SUCCESS;
  }

  return as2_behavior::ExecutionStatus::RUNNING;
}



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



void TakeOffServer::poseCallback(int drone_id, const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(drone_mutexes_[drone_id]);

  if (drone_id >= 0 && drone_id < 4) 
  {
    current_height_[drone_id] = msg->pose.position.z;
  }
}

void TakeOffServer::twistCallback(int drone_id, const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(drone_mutexes_[drone_id]);

  if (drone_id >= 0 && drone_id < 4) 
  {
    current_twist_[drone_id] = *msg;
  }
}



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TakeOffServer>());
  rclcpp::shutdown();
  return 0;
}
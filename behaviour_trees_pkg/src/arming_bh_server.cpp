#include "behaviour_trees_pkg/arming_bh_server.hpp"

using namespace std::chrono_literals;

/*!*******************************************************************************************
 *  \file       arming_bh_server.cpp
 *  \brief      Arming behaviour server implementation
 *  \authors    Miguel Tejado García
 ********************************************************************************************/

/* Constructor - initialize arming behavior server and create service clients for all drones */
ArmingServer::ArmingServer() : BehaviorServer("arming")  
{
  /* Create service clients and requests for each of the 4 drones */
  for (int i = 0; i < 4; ++i) 
  {
    /* Service client to communicate with drone's arming system */
    platform_arm_cli_[i] = this->create_client<std_srvs::srv::SetBool>("/drone" + std::to_string(i) + "/set_arming_state");
    
    /* Pre-allocate request objects for better performance */
    platform_arm_request_[i] = std::make_shared<std_srvs::srv::SetBool::Request>();
  }
}



/* Destructor - cleanup resources */
ArmingServer::~ArmingServer() 
{
  // No specific cleanup needed - shared_ptr handles memory management
}



/* Behavior activation - initiate arming process for all drones */
bool ArmingServer::on_activate(std::shared_ptr<const ArmingBh::Goal> goal)
{
  activation_time_ = this->now(); // Record start time for timeout handling

  /* Send arming requests to all drones sequentially */
  for (int i = 0; i < 4; ++i) 
  {
    platform_arm_request_[i]->data = goal->request; // Set arm/disarm command

    /* Wait for service to become available with timeout */
    if (!platform_arm_cli_[i]->wait_for_service(5s)) 
    {
      RCLCPP_ERROR(this->get_logger(), "[ArmingBh] Platform arming service not available for UAV %d", i);
      return false;
    }

    /* Send asynchronous request to avoid blocking */
    platform_arm_future_[i] = platform_arm_cli_[i]->async_send_request(platform_arm_request_[i]).future.share();
    if (!platform_arm_future_[i].valid()) 
    {
      RCLCPP_ERROR(this->get_logger(), "[ArmingBh] Arming service request for UAV %d could not be sent", i);
      return false;
    }

    /* Small delay between UAVs to avoid overwhelming the system */
    std::this_thread::sleep_for(100ms);
  }

  RCLCPP_INFO(this->get_logger(), "[ArmingBh] Arming requested");
  return true;
}



/* Behavior modification - handle changes to the arming request */
bool ArmingServer::on_modify(std::shared_ptr<const ArmingBh::Goal> goal)
{
  activation_time_ = this->now(); // Reset timer for new request

  /* Re-send arming requests with new parameters */
  for (int i = 0; i < 4; ++i) 
  {
    platform_arm_request_[i]->data = goal->request;

    if (!platform_arm_cli_[i]->wait_for_service(5s)) 
    {
      RCLCPP_ERROR(this->get_logger(), "[ArmingBh] Platform arming service not available for UAV %d", i);
      return false;
    }

    platform_arm_future_[i] = platform_arm_cli_[i]->async_send_request(platform_arm_request_[i]).future.share();
    if (!platform_arm_future_[i].valid()) 
    {
      RCLCPP_ERROR(this->get_logger(), "[ArmingBh] Arming service request for UAV %d could not be sent", i);
      return false;
    }

    /*  Small delay between UAVs to avoid overwhelming the system */
    std::this_thread::sleep_for(100ms);
  }

  RCLCPP_INFO(this->get_logger(), "[ArmingBh] Arming requested");
  return true;
}



/* Behavior deactivation - arming cannot be safely deactivated mid-process */
bool ArmingServer::on_deactivate(const std::shared_ptr<std::string> & /*msg*/)
{
  RCLCPP_WARN(this->get_logger(), "[ArmingBh] Cannot deactivate arming");
  return false; // Arming is a critical operation that shouldn't be interrupted
}



/* Behavior pause - arming cannot be paused for safety reasons */
bool ArmingServer::on_pause(const std::shared_ptr<std::string> & /*msg*/)
{
  RCLCPP_WARN(this->get_logger(), "[ArmingBh] Cannot pause arming");
  return false; // Arming must complete or fail, no intermediate state
}



/* Behavior resume - arming cannot be resumed since it cannot be paused */
bool ArmingServer::on_resume(const std::shared_ptr<std::string> & /*msg*/)
{
  RCLCPP_WARN(this->get_logger(), "[ArmingBh] Cannot resume arming");
  return false; // Consistent with pause behavior
}



/* Main execution loop - monitor arming progress and handle completion/timeout */
as2_behavior::ExecutionStatus ArmingServer::on_run(
    const std::shared_ptr<const ArmingBh::Goal> & /*goal*/,
    std::shared_ptr<ArmingBh::Feedback> & /*feedback*/,
    std::shared_ptr<ArmingBh::Result> & result)
{
  int success_counter = 0; // Track how many drones have successfully armed

  /* Check completion status of each drone's arming request */
  for (int i = 0; i < 4; i++)
  {
    /* Non-blocking check if service call is complete */
    if (platform_arm_future_[i].valid() && platform_arm_future_[i].wait_for(0s) == std::future_status::ready) 
    {
        auto srv_result = platform_arm_future_[i].get(); // Get service response
        if (srv_result->success) 
        {
          success_counter++;
          /* All drones successfully armed */
          if (success_counter == 4)
          {
            result->success = true;
            return as2_behavior::ExecutionStatus::SUCCESS;
          }
        }
        else
        {
            /* Any single drone failure causes overall failure */
            result->success = false;
            return as2_behavior::ExecutionStatus::FAILURE;
        }
    }
  }

  /* Check for timeout condition */
  if ((this->now() - activation_time_) > rclcpp::Duration(10s)) 
  {
    RCLCPP_ERROR(get_logger(), "[ArmingBh] Arming timed out");
    result->success = false;
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  std::this_thread::sleep_for(10ms);  // Give CPU a breather and prevent busy waiting
  return as2_behavior::ExecutionStatus::RUNNING; // Continue monitoring
}



/* Execution completion callback - log final status */
void ArmingServer::on_execution_end(const as2_behavior::ExecutionStatus & state)
{
  if (state == as2_behavior::ExecutionStatus::SUCCESS) 
  {
    RCLCPP_INFO(this->get_logger(), "[ArmingBh] Completed successfully");
  } 
  else 
  {
    RCLCPP_ERROR(this->get_logger(), "[ArmingBh] Terminated with failure");
  }
}



/* Main function - entry point for the arming behavior server node */
int main(int argc, char **argv)
{
  // Disable Fast RTPS shared memory transport for better compatibility
  ::setenv("RMW_FASTRTPS_SHARED_MEMORY_ENABLED", "0", 1);

  rclcpp::init(argc, argv); // Initialize ROS 2
  rclcpp::spin(std::make_shared<ArmingServer>()); // Run the behavior server
  rclcpp::shutdown(); // Clean shutdown
  return 0;
}
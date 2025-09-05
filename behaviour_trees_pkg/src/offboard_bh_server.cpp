#include "behaviour_trees_pkg/offboard_bh_server.hpp"

using namespace std::chrono_literals;

/*!*******************************************************************************************
 *  \file       offboard_bh_server.cpp
 *  \brief      Offboard behaviour server implementation for UAV swarm control
 *  \authors    Miguel Tejado García
 ********************************************************************************************/

/* Constructor: Initialize offboard behavior server and set up service clients for 4 UAVs */
OffboardServer::OffboardServer() : BehaviorServer("offboarding")  
{
  // Create service clients for each UAV's offboard mode control
  for (int i = 0; i < 4; ++i) 
  {
    platform_offboard_cli_[i] = this->create_client<std_srvs::srv::SetBool>("/drone" + std::to_string(i) + "/set_offboard_mode");
    platform_offboard_request_[i] = std::make_shared<std_srvs::srv::SetBool::Request>();
  }
}



/* Destructor: Clean up resources */
OffboardServer::~OffboardServer() 
{

}



/* Activate offboard mode for all UAVs in the swarm */
bool OffboardServer::on_activate(std::shared_ptr<const OffboardBh::Goal> goal)
{
  activation_time_ = this->now(); // Record activation timestamp for timeout tracking

  // Send offboard mode requests to all UAVs
  for (int i = 0; i < 4; ++i) 
  {
    platform_offboard_request_[i]->data = goal->request;

    // Wait for service availability with 5 second timeout
    if (!platform_offboard_cli_[i]->wait_for_service(5s)) 
    {
      RCLCPP_ERROR(this->get_logger(), "[OffboardBh] Platform offboard service not available for UAV %d", i);
      return false;
    }

    // Send async request and store future for later checking
    platform_offboard_future_[i] = platform_offboard_cli_[i]->async_send_request(platform_offboard_request_[i]).future.share();
    if (!platform_offboard_future_[i].valid()) 
    {
      RCLCPP_ERROR(this->get_logger(), "[OffboardBh] Offboard service request for UAV %d could not be sent", i);
      return false;
    }

    std::this_thread::sleep_for(100ms); // Small delay between UAVs to avoid overwhelming the system
  }

  RCLCPP_INFO(this->get_logger(), "[OffboardBh] Offboarding requested");
  return true;
}



/* Modify operation - not supported for offboard behavior */
bool OffboardServer::on_modify(std::shared_ptr<const OffboardBh::Goal> /*goal*/)
{
  RCLCPP_WARN(this->get_logger(), "[OffboardBh] Modify not supported");
  return false;
}



/* Deactivate operation - not supported for offboard behavior */
bool OffboardServer::on_deactivate(const std::shared_ptr<std::string> & /*msg*/)
{
  RCLCPP_WARN(this->get_logger(), "[OffboardBh] Offboarding deactivation not supported");
  return false;
}



/* Pause operation - not supported for offboard behavior */
bool OffboardServer::on_pause(const std::shared_ptr<std::string> & /*msg*/)
{
  RCLCPP_WARN(this->get_logger(), "[OffboardBh] Cannot pause offboarding");
  return false;
}



/* Resume operation - not supported for offboard behavior */
bool OffboardServer::on_resume(const std::shared_ptr<std::string> & /*msg*/)
{
  RCLCPP_WARN(this->get_logger(), "[OffboardBh] Cannot resume offboarding");
  return false;
}



/* Main execution loop: Check status of all UAV offboard requests */
as2_behavior::ExecutionStatus OffboardServer::on_run(
    const std::shared_ptr<const OffboardBh::Goal> & /*goal*/,
    std::shared_ptr<OffboardBh::Feedback> & /*feedback*/,
    std::shared_ptr<OffboardBh::Result> & result)
{
  int success_counter = 0; // Track number of successful UAV transitions

  // Check completion status for each UAV
  for (int i = 0; i < 4; i++)
  {
    if (platform_offboard_future_[i].valid() && platform_offboard_future_[i].wait_for(0s) == std::future_status::ready) 
    {
        auto srv_result = platform_offboard_future_[i].get();
        if (srv_result->success) 
        {
          success_counter++;
          if (success_counter == 4) // All UAVs successfully transitioned
          {
            result->success = true;
            return as2_behavior::ExecutionStatus::SUCCESS;
          }
        }
        else
        {
            result->success = false; // Any UAV failure causes overall failure
            return as2_behavior::ExecutionStatus::FAILURE;
        }
    }
  }

  // Check for timeout (10 seconds)
  if ((this->now() - activation_time_) > rclcpp::Duration(10s)) 
  {
    RCLCPP_ERROR(get_logger(), "[OffboardBh] Offboarding timed out");
    result->success = false;
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  std::this_thread::sleep_for(10ms);  // Give CPU a breather while waiting
  return as2_behavior::ExecutionStatus::RUNNING;
}



/* Handle execution completion and log final status */
void OffboardServer::on_execution_end(const as2_behavior::ExecutionStatus & state)
{
  if (state == as2_behavior::ExecutionStatus::SUCCESS) 
  {
    RCLCPP_INFO(this->get_logger(), "[OffboardBh] Completed successfully");
  } 
  else 
  {
    RCLCPP_ERROR(this->get_logger(), "[OffboardBh] Terminated with failure");
  }
}



/* Main function: Initialize ROS2 node and start spinning */
int main(int argc, char **argv)
{
  // Disable Fast RTPS shared memory transport for better compatibility
  ::setenv("RMW_FASTRTPS_SHARED_MEMORY_ENABLED", "0", 1);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardServer>()); // Start the behavior server
  rclcpp::shutdown();
  return 0;
}
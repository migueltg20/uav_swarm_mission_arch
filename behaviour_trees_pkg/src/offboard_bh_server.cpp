#include "behaviour_trees_pkg/offboard_bh_server.hpp"

using namespace std::chrono_literals;



OffboardServer::OffboardServer() : BehaviorServer("offboarding")  
{
  for (int i = 0; i < 4; ++i) 
  {
    platform_offboard_cli_[i] = this->create_client<std_srvs::srv::SetBool>("/drone" + std::to_string(i) + "/set_offboard_mode");
    platform_offboard_request_[i] = std::make_shared<std_srvs::srv::SetBool::Request>();
  }
}



OffboardServer::~OffboardServer() 
{

}



bool OffboardServer::on_activate(std::shared_ptr<const OffboardBh::Goal> goal)
{
  activation_time_ = this->now();

  for (int i = 0; i < 4; ++i) 
  {
    platform_offboard_request_[i]->data = goal->request;

    if (!platform_offboard_cli_[i]->wait_for_service(5s)) 
    {
      RCLCPP_ERROR(this->get_logger(), "[OffboardBh] Platform offboard service not available for UAV %d", i);
      return false;
    }

    platform_offboard_future_[i] = platform_offboard_cli_[i]->async_send_request(platform_offboard_request_[i]).future.share();
    if (!platform_offboard_future_[i].valid()) 
    {
      RCLCPP_ERROR(this->get_logger(), "[OffboardBh] Offboard service request for UAV %d could not be sent", i);
      return false;
    }

    /*  Small delay between UAVs to avoid overwhelming the system */
    std::this_thread::sleep_for(100ms);
  }

  RCLCPP_INFO(this->get_logger(), "[OffboardBh] Offboarding requested");
  return true;
}



bool OffboardServer::on_modify(std::shared_ptr<const OffboardBh::Goal> /*goal*/)
{
  RCLCPP_WARN(this->get_logger(), "[OffboardBh] Modify not supported");
  return false;
}



bool OffboardServer::on_deactivate(const std::shared_ptr<std::string> & /*msg*/)
{
  RCLCPP_WARN(this->get_logger(), "[OffboardBh] Offboarding deactivation not supported");
  return false;
}



bool OffboardServer::on_pause(const std::shared_ptr<std::string> & /*msg*/)
{
  RCLCPP_WARN(this->get_logger(), "[OffboardBh] Cannot pause offboarding");
  return false;
}



bool OffboardServer::on_resume(const std::shared_ptr<std::string> & /*msg*/)
{
  RCLCPP_WARN(this->get_logger(), "[OffboardBh] Cannot resume offboarding");
  return false;
}



as2_behavior::ExecutionStatus OffboardServer::on_run(
    const std::shared_ptr<const OffboardBh::Goal> & /*goal*/,
    std::shared_ptr<OffboardBh::Feedback> & /*feedback*/,
    std::shared_ptr<OffboardBh::Result> & result)
{
  int success_counter = 0;

  for (int i = 0; i < 4; i++)
  {
    if (platform_offboard_future_[i].valid() && platform_offboard_future_[i].wait_for(0s) == std::future_status::ready) 
    {
        auto srv_result = platform_offboard_future_[i].get();
        if (srv_result->success) 
        {
          success_counter++;
          if (success_counter == 4)
          {
            result->success = true;
            return as2_behavior::ExecutionStatus::SUCCESS;
          }
        }
        else
        {
            result->success = false;
            return as2_behavior::ExecutionStatus::FAILURE;
        }
    }
  }

  if ((this->now() - activation_time_) > rclcpp::Duration(10s)) 
  {
    RCLCPP_ERROR(get_logger(), "[OffboardBh] Offboarding timed out");
    result->success = false;
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  std::this_thread::sleep_for(10ms);  // Give CPU a breather
  return as2_behavior::ExecutionStatus::RUNNING;
}



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



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardServer>());
  rclcpp::shutdown();
  return 0;
}
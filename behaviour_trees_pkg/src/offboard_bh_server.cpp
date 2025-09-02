#include "behaviour_trees_pkg/offboard_bh_server.hpp"

using namespace std::chrono_literals;



OffboardServer::OffboardServer() : BehaviorServer("offboard")  
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
  for (int i = 0; i < 4; ++i) 
  {
    RCLCPP_INFO(this->get_logger(), "[OffboardBehavior] Configuring UAV %d", i);

    platform_offboard_request_[i]->data = goal->request;

    if (!platform_offboard_cli_[i]->wait_for_service(5s)) 
    {
      RCLCPP_ERROR(this->get_logger(), "[OffboardBehavior] Platform offboard service not available for UAV %d", i);
      return false;
    }

    platform_offboard_future_[i] = platform_offboard_cli_[i]->async_send_request(platform_offboard_request_[i]).future.share();
    if (platform_offboard_future_[i].wait_for(5s) != std::future_status::ready && !platform_offboard_future_[i].valid()) 
    {
      RCLCPP_ERROR(this->get_logger(), "[OffboardBehavior] Offboard service timeout for UAV %d", i);
      return false;
    }

    auto offboard_result = platform_offboard_future_[i].get();
    if (!offboard_result->success) 
    {
      RCLCPP_ERROR(this->get_logger(), "[OffboardBehavior] Failed to enable offboard mode for UAV %d", i);
      return false;
    }
  }

  RCLCPP_INFO(this->get_logger(), "[OffboardBehavior] All UAVs configured successfully");
  return true;
}



bool OffboardServer::on_modify(std::shared_ptr<const OffboardBh::Goal> /*goal*/)
{
  RCLCPP_WARN(this->get_logger(), "[OffboardBehavior] Modify not supported");
  return false;
}



bool OffboardServer::on_deactivate(const std::shared_ptr<std::string> & /*msg*/)
{
  RCLCPP_WARN(this->get_logger(), "[OffboardBehavior] Offboarding deactivation not supported");
  return false;
}



bool OffboardServer::on_pause(const std::shared_ptr<std::string> & /*msg*/)
{
  RCLCPP_WARN(this->get_logger(), "[OffboardBehavior] Cannot pause offboarding");
  return false;
}



bool OffboardServer::on_resume(const std::shared_ptr<std::string> & /*msg*/)
{
  RCLCPP_WARN(this->get_logger(), "[OffboardBehavior] Cannot resume offboarding");
  return false;
}



as2_behavior::ExecutionStatus OffboardServer::on_run(
    const std::shared_ptr<const OffboardBh::Goal> & /*goal*/,
    std::shared_ptr<OffboardBh::Feedback> & /*feedback*/,
    std::shared_ptr<OffboardBh::Result> & result)
{
  result->success = true;
  RCLCPP_INFO(this->get_logger(), "[OffboardBehavior] Offboard mode configured successfully");
  return as2_behavior::ExecutionStatus::SUCCESS;
}



void OffboardServer::on_execution_end(const as2_behavior::ExecutionStatus & state)
{
  if (state == as2_behavior::ExecutionStatus::SUCCESS) 
  {
    RCLCPP_INFO(this->get_logger(), "[OffboardBehavior] Completed successfully");
  } 
  else 
  {
    RCLCPP_ERROR(this->get_logger(), "[OffboardBehavior] Terminated with failure");
  }
}



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardServer>());
  rclcpp::shutdown();
  return 0;
}
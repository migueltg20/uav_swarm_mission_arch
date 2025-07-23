#include <motion_controller_pkg/motion_controller_client.hpp>

using namespace std::chrono_literals;



/*!*******************************************************************************************
 *  \file       motion_controller_client.cpp
 *  \brief      Controller client class implementation
 *  \authors    Miguel Tejado García
 ********************************************************************************************/



// --------------------------------------------------------------------------------------------
MotionClientNode::MotionClientNode() : Node("motion_client")
{
  /* Creating action client */
  nav_client_ = rclcpp_action::create_client<motion_controller_pkg::action::GoalPoint>(
    this, "PrimalBehaviour");  // <-- action name

  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) 
  {
    RCLCPP_ERROR(get_logger(), "NavigateToPoint action server not available.");
  }
  else 
  {
    RCLCPP_INFO(get_logger(), "NavigateToPoint action server is available.");
  }


  /* Suscription to the goal command topic */
  goal_subscriber_ = this->create_subscription<motion_controller_pkg::msg::GoalCommand>(
    "/goal", rclcpp::QoS(10),
    std::bind(&MotionClientNode::sendGoal, this, std::placeholders::_1));
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
MotionClientNode::~MotionClientNode()
{
  RCLCPP_INFO(this->get_logger(), "Destruyendo el nodo cliente...");
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
void MotionClientNode::goalResponseCallback(
  std::shared_ptr<rclcpp_action::ClientGoalHandle<motion_controller_pkg::action::GoalPoint>> goal_handle)
{
  if (!goal_handle) 
  {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server.");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Goal accepted by the server, waiting for result...");
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
void MotionClientNode::feedbackCallback(
  std::shared_ptr<rclcpp_action::ClientGoalHandle<motion_controller_pkg::action::GoalPoint>> /*goal_handle*/,
  const std::shared_ptr<const motion_controller_pkg::action::GoalPoint::Feedback> & feedback)
{
  RCLCPP_INFO(this->get_logger(), "Current position: [%.2f, %.2f, %.2f]",
    feedback->current_pose.pose.position.x,
    feedback->current_pose.pose.position.y,
    feedback->current_pose.pose.position.z);

  RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f",
    feedback->distance_remaining);
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
void MotionClientNode::resultCallback(
  const rclcpp_action::ClientGoalHandle<motion_controller_pkg::action::GoalPoint>::WrappedResult & result)
{
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) 
  {
    if (result.result->success) 
    {
        RCLCPP_INFO(this->get_logger(), "Goal succeeded and point reached!");
    } 
    else 
    {
        RCLCPP_WARN(this->get_logger(), "Goal succeeded but point NOT reached!");
    }
  } 
  else if (result.code == rclcpp_action::ResultCode::ABORTED) 
  {
    RCLCPP_ERROR(this->get_logger(), "Goal was aborted by the server.");
  } 
  else if (result.code == rclcpp_action::ResultCode::CANCELED) 
  {
    RCLCPP_WARN(this->get_logger(), "Goal was canceled.");
  } 
  else 
  {
    RCLCPP_WARN(this->get_logger(), "Unknown result code.");
  }
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
void MotionClientNode::sendGoal(const motion_controller_pkg::msg::GoalCommand::SharedPtr msg)
{
  /* Creating goal message */
  auto goal_msg = motion_controller_pkg::action::GoalPoint::Goal();
  goal_msg.goal_command = *msg;


  /* Configuring action client */
  rclcpp_action::Client<motion_controller_pkg::action::GoalPoint>::SendGoalOptions options;
  options.goal_response_callback = std::bind(&MotionClientNode::goalResponseCallback, this, std::placeholders::_1);
  options.feedback_callback = std::bind(&MotionClientNode::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  options.result_callback   = std::bind(&MotionClientNode::resultCallback, this, std::placeholders::_1);


  /* Sending goal */
  auto goal_handle_future = nav_client_->async_send_goal(goal_msg, options);
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
int main(int argc, char **argv)
{  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionClientNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
// --------------------------------------------------------------------------------------------
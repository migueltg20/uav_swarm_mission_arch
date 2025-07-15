#include <motion_controller_pkg/motion_controller_client.hpp>

using namespace std::chrono_literals;

MotionClientNode::MotionClientNode() : Node("motion_client")
{
  // create the client; use the correct action type here
  nav_client_ = rclcpp_action::create_client<motion_controller_pkg::action::GoalPoint>(
    this, "PrimalBehaviour");  // <-- action name

  // Optionally wait for the server to be available
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) 
  {
    RCLCPP_ERROR(get_logger(), "NavigateToPoint action server not available.");
  }

  goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal", rclcpp::QoS(10),
    std::bind(&MotionClientNode::sendGoal, this, std::placeholders::_1));
}


MotionClientNode::~MotionClientNode()
{
  RCLCPP_INFO(this->get_logger(), "Destruyendo el nodo cliente...");
}


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


void MotionClientNode::sendGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  auto goal_msg = motion_controller_pkg::action::GoalPoint::Goal();
  goal_msg.drone_id = 0;  
  goal_msg.point = *msg;
  goal_msg.navigation_speed = 0.0; // unused at the moment

  rclcpp_action::Client<motion_controller_pkg::action::GoalPoint>::SendGoalOptions options;
  options.feedback_callback = std::bind(&MotionClientNode::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  options.result_callback   = std::bind(&MotionClientNode::resultCallback, this, std::placeholders::_1);

  nav_client_->async_send_goal(goal_msg, options);
}


int main(int argc, char **argv)
{
  // Disable Fast RTPS shared memory transport
//   ::setenv("RMW_FASTRTPS_SHARED_MEMORY_ENABLED", "0", 1);
  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionClientNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
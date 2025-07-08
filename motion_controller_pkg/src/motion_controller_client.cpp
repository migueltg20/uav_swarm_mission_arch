#include <motion_controller_pkg/motion_controller_client.hpp>

using namespace std::chrono_literals;

MotionClientNode::MotionClientNode() : Node("motion_client")//, tf_buffer_(this->get_clock())
{
    // create the client; the string must match the server’s action name
    nav_client_ = rclcpp_action::create_client<as2_msgs::action::NavigateToPoint>(
    this, "PrimalBehaviour");  // <-- action name

    // Optionally wait for the server to be available
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_logger(), "NavigateToPoint action server not available.");
    }
}

MotionClientNode::~MotionClientNode()
{
  RCLCPP_INFO(this->get_logger(), "Destruyendo el nodo cliente...");
}

void MotionClientNode::sendNavigateGoal(double x, double y, double z)
{
  auto goal_msg = as2_msgs::action::NavigateToPoint::Goal();
  goal_msg.point.point.x = x;
  goal_msg.point.point.y = y;
  goal_msg.point.point.z = z;
  goal_msg.yaw.mode  = as2_msgs::msg::YawMode::FIXED_YAW;
  goal_msg.yaw.angle = 0.0;
  goal_msg.navigation_speed = 1.0;

  auto feedback_callback =
    [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<as2_msgs::action::NavigateToPoint>> /*goal_handle*/,
       const std::shared_ptr<const as2_msgs::action::NavigateToPoint::Feedback> & feedback)
    {
      RCLCPP_INFO(this->get_logger(), "Current position: [%.2f, %.2f, %.2f]",
        feedback.current_pose.position.x,
        feedback.current_pose.position.y,
        feedback.current_pose.position.z);
    };

    auto result_callback =
    [this](const rclcpp_action::ClientGoalHandle<as2_msgs::action::NavigateToPoint>::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        if (result.result->success) {
            RCLCPP_INFO(this->get_logger(), "Goal succeeded and point reached!");
        } else {
            RCLCPP_WARN(this->get_logger(), "Goal succeeded but point NOT reached!");
        }
        } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted by the server.");
        } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
        RCLCPP_WARN(this->get_logger(), "Goal was canceled.");
        } else {
        RCLCPP_WARN(this->get_logger(), "Unknown result code.");
        }
    };

  rclcpp_action::Client<as2_msgs::action::NavigateToPoint>::SendGoalOptions options;
  options.feedback_callback = feedback_callback;
  options.result_callback = result_callback;

  nav_client_->async_send_goal(goal_msg, options);
}

int main(int argc, char **argv)
{
  // Disable Fast RTPS shared memory transport
//   ::setenv("RMW_FASTRTPS_SHARED_MEMORY_ENABLED", "0", 1);
  
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionClientNode>();
    node->sendNavigateGoal(0.0, 0.0, 1.0);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
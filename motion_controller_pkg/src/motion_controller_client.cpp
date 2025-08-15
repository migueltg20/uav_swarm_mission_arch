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
  /* Suscription to the goal command topic */
  goal_subscriber_ = this->create_subscription<GoalCommand>(
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
  int id,
  std::shared_ptr<rclcpp_action::ClientGoalHandle<GoalPoint>> goal_handle)
{
  std::lock_guard<std::mutex> lock(goal_mutexes_[id]);

  if (!goal_handle) 
  {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server.");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Goal accepted by the server, waiting for result...");
  active_goals_[id] = goal_handle;
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
void MotionClientNode::feedbackCallback(
  int id,
  std::shared_ptr<rclcpp_action::ClientGoalHandle<GoalPoint>> /*goal_handle*/,
  const std::shared_ptr<const GoalPoint::Feedback> & feedback)
{
  RCLCPP_INFO(this->get_logger(),
    "[drone %d] Current pos: [%.2f, %.2f, %.2f] m",
    id,
    feedback->current_pose.pose.position.x,
    feedback->current_pose.pose.position.y,
    feedback->current_pose.pose.position.z);
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
void MotionClientNode::cancelGoal(int id)
{
  if (nav_clients_[id] == nullptr || active_goals_[id] == nullptr) 
  {
      RCLCPP_WARN(get_logger(), "No active goal for UAV %d", id);
      return;
  }
  
  auto client = nav_clients_[id];
  auto goal_handle = active_goals_[id];

  /* Fire off the cancel request */
  client->async_cancel_goal(goal_handle);
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
void MotionClientNode::resultCallback(
  int id,
  const rclcpp_action::ClientGoalHandle<GoalPoint>::WrappedResult & result)
{
  std::lock_guard<std::mutex> lock(goal_mutexes_[id]);

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
  active_goals_[id] = nullptr; 
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
void MotionClientNode::sendGoal(const GoalCommand::SharedPtr msg)
{
  /* Get the drone ID and action name */
  if (msg->drone_id > 3) 
  {
    RCLCPP_ERROR(get_logger(), "Invalid drone ID: %d. It must be between 0 and 3 (inclusive).", msg->drone_id);
    return;
  }
  int id = msg->drone_id;
  auto action_name = "/drone" + std::to_string(id) + "/PrimalBehaviour";

  std::lock_guard<std::mutex> lock(goal_mutexes_[id]);

  /* Creating goal message */
  auto goal_msg = GoalPoint::Goal();
  goal_msg.goal_command = *msg;


  /* Lazily create (or look up) the client */
  if (nav_clients_[id] == nullptr) 
  {
    nav_clients_[id] = rclcpp_action::create_client<GoalPoint>(this, action_name);
    if (!nav_clients_[id]->wait_for_action_server(5s)) 
    {
      RCLCPP_ERROR(get_logger(),
        "Action server '%s' not available.", action_name.c_str());
      return;
    }
  }
  auto &client = nav_clients_[id];


  /* Configure callbacks */
  rclcpp_action::Client<GoalPoint>::SendGoalOptions opts;
  opts.goal_response_callback = std::bind(&MotionClientNode::goalResponseCallback, this, id, std::placeholders::_1);
  opts.feedback_callback = std::bind(&MotionClientNode::feedbackCallback, this, id, std::placeholders::_1, std::placeholders::_2);
  opts.result_callback  = std::bind(&MotionClientNode::resultCallback, this, id, std::placeholders::_1);

  /* If this message is purely "cancel", do only that */
  if (goal_msg.goal_command.cancel)
  {
    if (active_goals_[id] != nullptr) 
    {
      auto cancel_future = client->async_cancel_goal(active_goals_[id]);
      // Wait for cancellation or implement proper state management
      if (cancel_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
        RCLCPP_WARN(get_logger(), "Cancellation timeout for drone %d", id);
      }
      active_goals_[id] = nullptr;
    }
    return;
  }

  /* Otherwise, if there’s still an active goal, cancel it first */
  if (active_goals_[id] != nullptr) 
  {
    auto cancel_future = client->async_cancel_goal(active_goals_[id]);
    /* Wait for cancellation or implement proper state management */
    if (cancel_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) 
    {
      RCLCPP_WARN(get_logger(), "Cancellation timeout for drone %d", id);
    }
    active_goals_[id] = nullptr;
  }

  /* Send the new goal */
  client->async_send_goal(goal_msg, opts);
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
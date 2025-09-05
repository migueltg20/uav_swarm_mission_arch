/* 
 * Arming Action Behavior Tree Node
 * This file defines a behavior tree action node for arming/disarming drones
 * using the AS2 framework's SetArmingState action service
 */
#ifndef AS2_BEHAVIOR_TREE__ACTION__ARMING_HPP_
#define AS2_BEHAVIOR_TREE__ACTION__ARMING_HPP_

#include <string>
#include <memory>
#include <vector>

#include "as2_behavior_tree/bt_action_node.hpp"
#include "behaviortree_cpp_v3/action_node.h"

#include "as2_core/names/actions.hpp"
#include "as2_msgs/action/set_arming_state.hpp"

/*!*******************************************************************************************
 *  \file       arming.hpp
 *  \brief      Arm client class declaration
 *  \authors    Miguel Tejado García
 ********************************************************************************************/

namespace as2_behavior_tree
{
/* 
 * ArmingAction class
 * Behavior tree action node that handles drone arming/disarming operations
 * Inherits from BtActionNode to provide ROS 2 action client functionality
 */
class ArmingAction
  : public nav2_behavior_tree::BtActionNode<as2_msgs::action::SetArmingState>
{
public:
  /* Constructor - initializes the action node with XML tag name and configuration */
  ArmingAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtActionNode<as2_msgs::action::SetArmingState>(
      xml_tag_name, "arming", conf) {} // "arming" is the action server name

  /* 
   * Called on each behavior tree tick
   * Retrieves the arming request from input port and sets the goal
   */
  void on_tick()
  {
    getInput("request", request_); // Get boolean input for arm/disarm request
    goal_.request = request_; // Set the goal request field
  }

  /* 
   * Define the input/output ports for this behavior tree node
   * Returns the list of available ports for XML configuration
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {BT::InputPort<bool>("request")}); // Input port for arm (true) or disarm (false)
  }

//   /* Commented out success handler - would process the arming result */
//   BT::NodeStatus on_success() override
//   {
//     // goal handle stores the result
//     bool ok = result_.result.armed;
//     setOutput("armed", ok);
//     return BT::NodeStatus::SUCCESS;
//   }

  /* 
   * Callback for processing feedback during action execution
   * Currently empty as no feedback processing is needed for arming
   */
  void on_wait_for_result(
    std::shared_ptr<const as2_msgs::action::SetArmingState::Feedback> feedback) {}

    
private:
  bool request_; // Stores the arming request (true = arm, false = disarm)
};

}  // namespace as2_behavior_tree

#endif  // AS2_BEHAVIOR_TREE__ACTION__ARMING_HPP_
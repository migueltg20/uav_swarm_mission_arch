/* 
 * Offboard Mode Action Behavior Tree Node
 * This file defines a behavior tree action node for enabling/disabling
 * offboard control mode in drones using the AS2 framework
 */
#ifndef AS2_BEHAVIOR_TREE__ACTION__OFFBOARD_HPP_
#define AS2_BEHAVIOR_TREE__ACTION__OFFBOARD_HPP_

#include <string>
#include <memory>
#include <vector>

#include "as2_behavior_tree/bt_action_node.hpp"
#include "behaviortree_cpp_v3/action_node.h"

#include "as2_core/names/actions.hpp"
#include "as2_msgs/action/set_offboard_mode.hpp"

/*!*******************************************************************************************
 *  \file       offboard.hpp
 *  \brief      Offboard mode client class declaration
 *  \authors    Miguel Tejado García
 ********************************************************************************************/

namespace as2_behavior_tree
{
/* 
 * OffboardingAction class
 * Behavior tree action node that handles drone offboard mode operations
 * Inherits from BtActionNode to provide ROS 2 action client functionality
 */
class OffboardingAction
  : public nav2_behavior_tree::BtActionNode<as2_msgs::action::SetOffboardMode>
{
public:
  /* Constructor - initializes the action node with XML tag name and configuration */
  OffboardingAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtActionNode<as2_msgs::action::SetOffboardMode>(
      xml_tag_name, "offboarding", conf) {} // "offboarding" is the action server name

  /* 
   * Called on each behavior tree tick
   * Retrieves the offboard mode request from input port and sets the goal
   */
  void on_tick()
  {
    getInput("request", request_); // Get boolean input for offboard mode request
    goal_.request = request_; // Set the goal request field
  }

  /* 
   * Define the input/output ports for this behavior tree node
   * Returns the list of available ports for XML configuration
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {BT::InputPort<bool>("request")}); // Input port for offboard mode (true = enable, false = disable)
  }

  /* 
   * Callback for processing feedback during action execution
   * Currently empty as no feedback processing is needed for offboard mode
   */
  void on_wait_for_result(
    std::shared_ptr<const as2_msgs::action::SetOffboardMode::Feedback> feedback) {}


private:
  bool request_; // Stores the offboard mode request (true = enable, false = disable)
};

}  // namespace as2_behavior_tree

#endif  // AS2_BEHAVIOR_TREE__ACTION__OFFBOARD_HPP_
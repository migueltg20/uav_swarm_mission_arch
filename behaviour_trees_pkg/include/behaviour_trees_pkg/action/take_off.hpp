/* 
 * Takeoff Action Behavior Tree Node
 * This file defines a behavior tree action node for commanding drones
 * to perform takeoff operations to a specified height
 */
#ifndef AS2_BEHAVIOR_TREE__ACTION__TAKE_OFF_HPP_
#define AS2_BEHAVIOR_TREE__ACTION__TAKE_OFF_HPP_

#include <string>
#include <memory>
#include <vector>

#include "as2_behavior_tree/bt_action_node.hpp"
#include "behaviortree_cpp_v3/action_node.h"

#include "behaviour_trees_pkg/action/take_off_bh.hpp"

/*!*******************************************************************************************
 *  \file       take_off.hpp
 *  \brief      Take off client class declaration
 *  \authors    Miguel Tejado García
 ********************************************************************************************/

namespace as2_behavior_tree
{
/* 
 * TakingOffAction class
 * Behavior tree action node that handles drone takeoff operations
 * Inherits from BtActionNode to provide ROS 2 action client functionality
 */
class TakingOffAction
  : public nav2_behavior_tree::BtActionNode<behaviour_trees_pkg::action::TakeOffBh>
{
public:
  /* Constructor - initializes the action node with XML tag name and configuration */
  TakingOffAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtActionNode<behaviour_trees_pkg::action::TakeOffBh>(
      xml_tag_name, "take_off", conf) {} // "take_off" is the action server name

  /* 
   * Called on each behavior tree tick
   * Retrieves takeoff parameters from input ports and sets the goal
   */
  void on_tick()
  {
    getInput("drone_id", drone_id_); // Get target drone identifier
    getInput("takeoff_height", takeoff_height_); // Get desired takeoff height
    goal_.drone_id = drone_id_; // Set the target drone ID in goal
    goal_.takeoff_height = takeoff_height_; // Set the takeoff height in goal
  }

  /* 
   * Define the input/output ports for this behavior tree node
   * Returns the list of available ports for XML configuration
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {BT::InputPort<int>("drone_id"), // Input port for drone identifier
       BT::InputPort<float>("takeoff_height")}); // Input port for takeoff height in meters
  }

  /* 
   * Callback for processing feedback during action execution
   * Could be used to provide real-time feedback about takeoff progress
   */
  void on_wait_for_result(
    std::shared_ptr<const behaviour_trees_pkg::action::TakeOffBh::Feedback> feedback) 
  {
    // Commented out feedback outputs - could be enabled to track takeoff progress
    // setOutput("drone_id", feedback->drone_id);
    // setOutput("takeoff_height", feedback->takeoff_height);
  }


private:
  int drone_id_; // Stores the target drone identifier
  float takeoff_height_; // Stores the desired takeoff height in meters
};

}  // namespace as2_behavior_tree

#endif  // AS2_BEHAVIOR_TREE__ACTION__TAKE_OFF_HPP_
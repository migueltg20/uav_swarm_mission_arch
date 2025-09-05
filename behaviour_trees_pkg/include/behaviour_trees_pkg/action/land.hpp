/* 
 * Landing Action Behavior Tree Node
 * This file defines a behavior tree action node for commanding drones
 * to perform landing operations using the AS2 framework
 */
#ifndef AS2_BEHAVIOR_TREE__ACTION__LAND_HPP_
#define AS2_BEHAVIOR_TREE__ACTION__LAND_HPP_

#include <string>
#include <memory>
#include <vector>

#include "as2_behavior_tree/bt_action_node.hpp"
#include "behaviortree_cpp_v3/action_node.h"

#include "behaviour_trees_pkg/action/land_bh.hpp"

/*!*******************************************************************************************
 *  \file       land.hpp
 *  \brief      Land client class declaration
 *  \authors    Miguel Tejado García
 ********************************************************************************************/

namespace as2_behavior_tree
{
/* 
 * LandingAction class
 * Behavior tree action node that handles drone landing operations
 * Inherits from BtActionNode to provide ROS 2 action client functionality
 */
class LandingAction
  : public nav2_behavior_tree::BtActionNode<behaviour_trees_pkg::action::LandBh>
{
public:
  /* Constructor - initializes the action node with XML tag name and configuration */
  LandingAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtActionNode<behaviour_trees_pkg::action::LandBh>(
      xml_tag_name, "land", conf) {} // "land" is the action server name

  /* 
   * Called on each behavior tree tick
   * Retrieves landing parameters from input ports and sets the goal
   */
  void on_tick()
  {
    getInput("drone_id", drone_id_); // Get target drone identifier
    getInput("land", land_); // Get landing command (true = land, false = abort)
    goal_.drone_id = drone_id_; // Set the target drone ID in goal
    goal_.land = land_; // Set the landing command in goal
  }

  /* 
   * Define the input/output ports for this behavior tree node
   * Returns the list of available ports for XML configuration
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {BT::InputPort<int>("drone_id"), // Input port for drone identifier
       BT::InputPort<bool>("land")}); // Input port for landing command
  }

  /* 
   * Callback for processing feedback during action execution
   * Currently empty as no feedback processing is needed for landing
   */
  void on_wait_for_result(
    std::shared_ptr<const behaviour_trees_pkg::action::LandBh::Feedback> feedback) {}


private:
  int drone_id_; // Stores the target drone identifier
  bool land_; // Stores the landing command (true = land, false = abort)
};

}  // namespace as2_behavior_tree

#endif  // AS2_BEHAVIOR_TREE__ACTION__LAND_HPP_
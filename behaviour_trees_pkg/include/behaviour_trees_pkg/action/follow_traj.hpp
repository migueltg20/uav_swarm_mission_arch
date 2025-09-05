/* 
 * Follow Trajectory Action Behavior Tree Node
 * This file defines a behavior tree action node for commanding drones
 * to follow predefined trajectory setpoints with specified tolerance radius
 */
#ifndef AS2_BEHAVIOR_TREE__ACTION__FOLLOW_TRAJ_HPP_
#define AS2_BEHAVIOR_TREE__ACTION__FOLLOW_TRAJ_HPP_

#include <string>
#include <memory>
#include <vector>

#include "as2_behavior_tree/bt_action_node.hpp"
#include "behaviortree_cpp_v3/action_node.h"

#include "behaviour_trees_pkg/action/follow_traj_bh.hpp"
#include "as2_msgs/msg/trajectory_setpoints.hpp"

/*!*******************************************************************************************
 *  \file       follow_traj.hpp
 *  \brief      Follow trajectory client class declaration
 *  \authors    Miguel Tejado García
 ********************************************************************************************/

namespace as2_behavior_tree
{
/* 
 * FollowingTrajAction class
 * Behavior tree action node that handles drone trajectory following operations
 * Inherits from BtActionNode to provide ROS 2 action client functionality
 */
class FollowingTrajAction
  : public nav2_behavior_tree::BtActionNode<behaviour_trees_pkg::action::FollowTrajBh>
{
public:
  /* Constructor - initializes the action node with XML tag name and configuration */
  FollowingTrajAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtActionNode<behaviour_trees_pkg::action::FollowTrajBh>(
      xml_tag_name, "follow_traj", conf) {} // "follow_traj" is the action server name

  /* 
   * Called on each behavior tree tick
   * Retrieves trajectory parameters from input ports and sets the goal
   */
  void on_tick()
  {
    getInput("drone_id", drone_id_); // Get target drone identifier
    getInput("trajectory", trajectory_); // Get trajectory setpoints to follow
    getInput("radius", radius_); // Get tolerance radius for waypoint acceptance
    goal_.drone_id = drone_id_; // Set the target drone ID in goal
    goal_.trajectory = trajectory_; // Set the trajectory setpoints in goal
    goal_.radius = radius_; // Set the acceptance radius in goal
  }

  /* 
   * Define the input/output ports for this behavior tree node
   * Returns the list of available ports for XML configuration
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {BT::InputPort<int>("drone_id"), // Input port for drone identifier
       BT::InputPort<as2_msgs::msg::TrajectorySetpoints>("trajectory"), // Input port for trajectory data
       BT::InputPort<float>("radius")}); // Input port for waypoint acceptance radius
  }

  /* 
   * Callback for processing feedback during action execution
   * Currently empty as no feedback processing is needed for trajectory following
   */
  void on_wait_for_result(
    std::shared_ptr<const behaviour_trees_pkg::action::FollowTrajBh::Feedback> feedback) {}


private:
  int drone_id_; // Stores the target drone identifier
  as2_msgs::msg::TrajectorySetpoints trajectory_; // Stores the trajectory setpoints to follow
  float radius_; // Stores the tolerance radius for waypoint acceptance
};

}  // namespace as2_behavior_tree

#endif  // AS2_BEHAVIOR_TREE__ACTION__FOLLOW_TRAJ_HPP_
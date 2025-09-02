#ifndef BEHAVIOUR_TREES_PKG__ACTION__FOLLOW_TRAJ_HPP_
#define BEHAVIOUR_TREES_PKG__ACTION__FOLLOW_TRAJ_HPP_

#include <string>
#include <memory>
#include <vector>

#include "as2_behavior_tree/bt_action_node.hpp"
#include "behaviortree_cpp_v3/action_node.h"

#include "behaviour_trees_pkg/action/follow_traj_bh.hpp"
#include "as2_msgs/msg/trajectory_setpoints.hpp"

namespace behaviour_trees_pkg
{
class FollowTrajAction
  : public nav2_behavior_tree::BtActionNode<behaviour_trees_pkg::action::FollowTrajBh>
{
public:
  FollowTrajAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtActionNode<behaviour_trees_pkg::action::FollowTrajBh>(
      xml_tag_name, "follow_traj", conf) {}

  void on_tick()
  {
    getInput("drone_id", drone_id_);
    getInput("trajectory", trajectory_);
    getInput("radius", radius_);
    goal_.drone_id = drone_id_;
    goal_.trajectory = trajectory_;
    goal_.radius = radius_;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {BT::InputPort<int>("drone_id"),
       BT::InputPort<as2_msgs::msg::TrajectorySetpoints>("trajectory"),
       BT::InputPort<double>("radius")});
  }

  void on_wait_for_result(
    std::shared_ptr<const behaviour_trees_pkg::action::FollowTrajBh::Feedback> feedback) {}

private:
  int drone_id_;
  as2_msgs::msg::TrajectorySetpoints trajectory_;
  double radius_;
};

}  // namespace behaviour_trees_pkg

#endif  // BEHAVIOUR_TREES_PKG__ACTION__FOLLOW_TRAJ_HPP_
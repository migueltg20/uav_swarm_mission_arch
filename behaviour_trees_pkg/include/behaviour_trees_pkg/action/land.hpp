#ifndef AS2_BEHAVIOR_TREE__ACTION__LAND_HPP_
#define AS2_BEHAVIOR_TREE__ACTION__LAND_HPP_

#include <string>
#include <memory>
#include <vector>

#include "as2_behavior_tree/bt_action_node.hpp"
#include "behaviortree_cpp_v3/action_node.h"

#include "behaviour_trees_pkg/action/land_bh.hpp"

namespace as2_behavior_tree
{
class LandingAction
  : public nav2_behavior_tree::BtActionNode<behaviour_trees_pkg::action::LandBh>
{
public:
  LandingAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtActionNode<behaviour_trees_pkg::action::LandBh>(
      xml_tag_name, "land", conf) {}

  void on_tick()
  {
    getInput("drone_id", drone_id_);
    getInput("land", land_);
    goal_.drone_id = drone_id_;
    goal_.land = land_;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {BT::InputPort<int>("drone_id"),
       BT::InputPort<bool>("land")});
  }

  void on_wait_for_result(
    std::shared_ptr<const behaviour_trees_pkg::action::LandBh::Feedback> feedback) {}

private:
  int drone_id_;
  bool land_;
};

}  // namespace as2_behavior_tree

#endif  // AS2_BEHAVIOR_TREE__ACTION__LAND_HPP_
#ifndef BEHAVIOUR_TREES_PKG__ACTION__LAND_HPP_
#define BEHAVIOUR_TREES_PKG__ACTION__LAND_HPP_

#include <string>
#include <memory>
#include <vector>

#include "as2_behavior_tree/bt_action_node.hpp"
#include "behaviortree_cpp_v3/action_node.h"

#include "behaviour_trees_pkg/action/land_bh.hpp"

namespace behaviour_trees_pkg
{
class LandAction
  : public nav2_behavior_tree::BtActionNode<behaviour_trees_pkg::action::LandBh>
{
public:
  LandAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtActionNode<behaviour_trees_pkg::action::LandBh>(
      xml_tag_name, "land", conf) {}

  void on_tick()
  {
    getInput("land", land_);
    goal_.land = land_;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {BT::InputPort<bool>("land")});
  }

  void on_wait_for_result(
    std::shared_ptr<const behaviour_trees_pkg::action::LandBh::Feedback> feedback) {}

private:
  bool land_;
};

}  // namespace behaviour_trees_pkg

#endif  // BEHAVIOUR_TREES_PKG__ACTION__LAND_HPP_
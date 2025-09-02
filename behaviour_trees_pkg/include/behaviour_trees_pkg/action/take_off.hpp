#ifndef BEHAVIOUR_TREES_PKG__ACTION__TAKE_OFF_HPP_
#define BEHAVIOUR_TREES_PKG__ACTION__TAKE_OFF_HPP_

#include <string>
#include <memory>
#include <vector>

#include "as2_behavior_tree/bt_action_node.hpp"
#include "behaviortree_cpp_v3/action_node.h"

#include "behaviour_trees_pkg/action/take_off_bh.hpp"

namespace behaviour_trees_pkg
{
class TakeOffAction
  : public nav2_behavior_tree::BtActionNode<behaviour_trees_pkg::action::TakeOffBh>
{
public:
  TakeOffAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtActionNode<behaviour_trees_pkg::action::TakeOffBh>(
      xml_tag_name, "take_off", conf) {}

  void on_tick()
  {
    getInput("drone_id", drone_id_);
    getInput("takeoff_height", takeoff_height_);
    goal_.drone_id = drone_id_;
    goal_.takeoff_height = takeoff_height_;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {BT::InputPort<int>("drone_id"),
       BT::InputPort<float>("takeoff_height")});
  }

  void on_wait_for_result(
    std::shared_ptr<const behaviour_trees_pkg::action::TakeOffBh::Feedback> feedback) {}

private:
  int drone_id_;
  float takeoff_height_;
};

}  // namespace behaviour_trees_pkg

#endif  // BEHAVIOUR_TREES_PKG__ACTION__TAKE_OFF_HPP_
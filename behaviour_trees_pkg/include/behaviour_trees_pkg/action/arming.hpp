#ifndef BEHAVIOUR_TREES_PKG__ACTION__ARMING_HPP_
#define BEHAVIOUR_TREES_PKG__ACTION__ARMING_HPP_

#include <string>
#include <memory>
#include <vector>

#include "as2_behavior_tree/bt_action_node.hpp"
#include "behaviortree_cpp_v3/action_node.h"

#include "as2_msgs/action/set_arming_state.hpp"

namespace behaviour_trees_pkg
{
class ArmAction
  : public nav2_behavior_tree::BtActionNode<as2_msgs::action::SetArmingState>
{
public:
  ArmAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtActionNode<as2_msgs::action::SetArmingState>(
      xml_tag_name, "arming", conf) {}

  void on_tick()
  {
    getInput("request", request_);
    goal_.request = request_;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {BT::InputPort<bool>("request")});
  }

//   BT::NodeStatus on_success() override
//   {
//     // goal handle stores the result
//     bool ok = result_.result.armed;
//     setOutput("armed", ok);
//     return BT::NodeStatus::SUCCESS;
//   }

  void on_wait_for_result(
    std::shared_ptr<const as2_msgs::action::SetArmingState::Feedback> feedback) {}

private:
  bool request_;
};

}  // namespace behaviour_trees_pkg

#endif  // BEHAVIOUR_TREES_PKG__ACTION__ARMING_HPP_
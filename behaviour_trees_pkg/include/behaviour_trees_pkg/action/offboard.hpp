#ifndef BEHAVIOUR_TREES_PKG__ACTION__OFFBOARD_HPP_
#define BEHAVIOUR_TREES_PKG__ACTION__OFFBOARD_HPP_

#include <string>
#include <memory>
#include <vector>

#include "as2_behavior_tree/bt_action_node.hpp"
#include "behaviortree_cpp_v3/action_node.h"

#include "as2_msgs/action/set_offboard_mode.hpp"

namespace behaviour_trees_pkg
{
class OffboardAction
  : public nav2_behavior_tree::BtActionNode<as2_msgs::action::SetOffboardMode>
{
public:
  OffboardAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtActionNode<as2_msgs::action::SetOffboardMode>(
      xml_tag_name, "offboard", conf) {}

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

  void on_wait_for_result(
    std::shared_ptr<const as2_msgs::action::SetOffboardMode::Feedback> feedback) {}

private:
  bool request_;
};

}  // namespace behaviour_trees_pkg

#endif  // BEHAVIOUR_TREES_PKG__ACTION__OFFBOARD_HPP_
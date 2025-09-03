#ifndef BEHAVIOUR_TREES_PKG__ARMING_BH_SERVER_HPP
#define BEHAVIOUR_TREES_PKG__ARMING_BH_SERVER_HPP

#include <array>
#include <memory>
#include <future>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "as2_behavior/behavior_server.hpp"
#include "as2_msgs/action/set_arming_state.hpp"

class ArmingServer : public as2_behavior::BehaviorServer<as2_msgs::action::SetArmingState>
{
public:
  using ArmingBh = as2_msgs::action::SetArmingState;

  ArmingServer();
  ~ArmingServer();

protected:
  bool on_activate(std::shared_ptr<const ArmingBh::Goal> goal) override;
  bool on_modify(std::shared_ptr<const ArmingBh::Goal> goal) override;
  bool on_deactivate(const std::shared_ptr<std::string>&) override;
  bool on_pause(const std::shared_ptr<std::string>&) override;
  bool on_resume(const std::shared_ptr<std::string>&) override;
  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const ArmingBh::Goal>& goal,
    std::shared_ptr<ArmingBh::Feedback>& feedback,
    std::shared_ptr<ArmingBh::Result>& result) override;
  void on_execution_end(const as2_behavior::ExecutionStatus& state) override;

private:
  rclcpp::Time activation_time_;
  std::array<rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr, 4> platform_arm_cli_;
  std::array<std::shared_ptr<std_srvs::srv::SetBool::Request>, 4> platform_arm_request_;
  std::array<std::shared_future<std_srvs::srv::SetBool::Response::SharedPtr>, 4> platform_arm_future_;
};

#endif  // BEHAVIOUR_TREES_PKG__ARMING_BH_SERVER_HPP

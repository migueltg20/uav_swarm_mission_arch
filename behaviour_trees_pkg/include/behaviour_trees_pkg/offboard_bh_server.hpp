#ifndef BEHAVIOUR_TREES_PKG__OFFBOARD_BH_SERVER_HPP
#define BEHAVIOUR_TREES_PKG__OFFBOARD_BH_SERVER_HPP

#include <array>
#include <memory>
#include <future>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "as2_behavior/behavior_server.hpp"
#include "as2_msgs/action/set_offboard_mode.hpp"

class OffboardServer : public as2_behavior::BehaviorServer<as2_msgs::action::SetOffboardMode>
{
public:
  using OffboardBh = as2_msgs::action::SetOffboardMode;

  OffboardServer();
  ~OffboardServer();

protected:
  bool on_activate(std::shared_ptr<const OffboardBh::Goal> goal) override;
  bool on_modify(std::shared_ptr<const OffboardBh::Goal> goal) override;
  bool on_deactivate(const std::shared_ptr<std::string>& msg) override;
  bool on_pause(const std::shared_ptr<std::string>& msg) override;
  bool on_resume(const std::shared_ptr<std::string>& msg) override;
  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const OffboardBh::Goal>& goal,
    std::shared_ptr<OffboardBh::Feedback>& feedback,
    std::shared_ptr<OffboardBh::Result>& result) override;
  void on_execution_end(const as2_behavior::ExecutionStatus& state) override;

private:
  rclcpp::Time activation_time_;
  std::array<rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr, 4> platform_offboard_cli_;
  std::array<std::shared_ptr<std_srvs::srv::SetBool::Request>, 4> platform_offboard_request_;
  std::array<std::shared_future<std_srvs::srv::SetBool::Response::SharedPtr>, 4> platform_offboard_future_;
};

#endif  // BEHAVIOUR_TREES_PKG__OFFBOARD_BH_SERVER_HPP

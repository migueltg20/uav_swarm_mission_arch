#ifndef BEHAVIOUR_PKG__OFFBOARD_BH_SERVER_HPP_
#define BEHAVIOUR_PKG__OFFBOARD_BH_SERVER_HPP_

#include <memory>
#include <chrono>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "as2_behavior/behavior_server.hpp"
#include "as2_msgs/action/set_offboard_mode.hpp"
#include "as2_msgs/srv/set_control_mode.hpp"


class OffboardServer : public as2_behavior::BehaviorServer<as2_msgs::action::SetOffboardMode>
{
    using OffboardBh = as2_msgs::action::SetOffboardMode;

public:
    OffboardServer();
    ~OffboardServer();

protected:
    bool on_activate(std::shared_ptr<const OffboardBh::Goal> goal) override;
    bool on_modify(std::shared_ptr<const OffboardBh::Goal> /*goal*/) override;
    bool on_deactivate(const std::shared_ptr<std::string> & /*msg*/) override;
    bool on_pause(const std::shared_ptr<std::string> & /*msg*/) override;
    bool on_resume(const std::shared_ptr<std::string> & /*msg*/) override;
    as2_behavior::ExecutionStatus on_run(
        const std::shared_ptr<const OffboardBh::Goal> & /*goal*/,
        std::shared_ptr<OffboardBh::Feedback> & /*feedback*/,
        std::shared_ptr<OffboardBh::Result> & result) override;
    void on_execution_end(const as2_behavior::ExecutionStatus & state) override;

private:
    std::array<rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr, 4> platform_offboard_cli_;
    std::array<std::shared_ptr<std_srvs::srv::SetBool::Request>, 4> platform_offboard_request_;
    std::array<std::shared_future<std::shared_ptr<std_srvs::srv::SetBool::Response>>, 4> platform_offboard_future_;

};

#endif  // BEHAVIOUR_PKG__OFFBOARD_BH_SERVER_HPP_
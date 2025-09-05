#ifndef BEHAVIOUR_TREES_PKG__TAKEOFF_BH_SERVER_HPP
#define BEHAVIOUR_TREES_PKG__TAKEOFF_BH_SERVER_HPP

#include <mutex>
#include <array>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "as2_behavior/behavior_server.hpp"
#include "behaviour_trees_pkg/action/take_off_bh.hpp"
#include "motion_controller_pkg/msg/goal_command.hpp"

/*!*******************************************************************************************
 *  \file       takeoff_bh_server.hpp
 *  \brief      Take off behaviour server class declaration for drone swarm takeoff operations
 *  \authors    Miguel Tejado García
 ********************************************************************************************/

/* TakeOffServer class: Behavior server for coordinating drone takeoff operations */
class TakeOffServer : public as2_behavior::BehaviorServer<behaviour_trees_pkg::action::TakeOffBh>
{
    /* Type aliases for cleaner code */
    using TakeOffBh = behaviour_trees_pkg::action::TakeOffBh;
    using GoalCommand = motion_controller_pkg::msg::GoalCommand;


public:
    /* Constructor and destructor */
    TakeOffServer();
    ~TakeOffServer();


protected:
    /* Behavior server lifecycle methods - override base class functionality */
    bool on_activate(std::shared_ptr<const TakeOffBh::Goal> goal) override;
    bool on_modify(std::shared_ptr<const TakeOffBh::Goal> goal) override;
    bool on_deactivate(const std::shared_ptr<std::string>& /*msg*/) override;
    bool on_pause(const std::shared_ptr<std::string>& /*msg*/) override;
    bool on_resume(const std::shared_ptr<std::string>& /*msg*/) override;
    as2_behavior::ExecutionStatus on_run(
        const std::shared_ptr<const TakeOffBh::Goal>& goal,
        std::shared_ptr<TakeOffBh::Feedback>& feedback,
        std::shared_ptr<TakeOffBh::Result>& result) override;
    void on_execution_end(const as2_behavior::ExecutionStatus& state) override;

    
private:
    /* Callback methods for drone state updates */
    void poseCallback(int drone_id, const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void twistCallback(int drone_id, const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    /* QoS profile optimized for sensor data with best effort reliability */
    rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data))
        .reliability(rclcpp::ReliabilityPolicy::BestEffort)
        .durability(rclcpp::DurabilityPolicy::Volatile); 

    /* ROS2 communication interfaces */
    std::array<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr, 4> pose_subscribers_;    // Position data subscribers for each drone
    std::array<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr, 4> twist_subscribers_;  // Velocity data subscribers for each drone
    rclcpp::Publisher<GoalCommand>::SharedPtr goal_publisher_;                                             // Publisher for takeoff commands

    /* Drone state data arrays (index = drone_id) */
    std::array<float, 4> current_height_;                              // Current Z-position for each drone
    std::array<geometry_msgs::msg::TwistStamped, 4> current_twist_;    // Current velocity data for each drone
    std::array<std::mutex, 4> drone_mutexes_;                          // Thread synchronization for each drone's data

    /* Command and timing variables */
    GoalCommand goal_command_;          // Current takeoff command being executed
    rclcpp::Time takeoff_start_time_;   // Timestamp when takeoff began (for timeout detection)

    /* Configuration parameters */
    float takeoff_threshold_ = 0.01;    // Height tolerance for determining takeoff completion (meters)
};

#endif  // BEHAVIOUR_TREES_PKG__TAKEOFF_BH_SERVER_HPP

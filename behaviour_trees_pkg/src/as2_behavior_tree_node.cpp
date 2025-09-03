// Copyright 2024 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @file as2_behavior_tree_node.hpp
 *
 * ROS2 entrypoint for launching a node to run a behavior tree
 *
 * @authors Pedro Arias Pérez
 *          Rafael Perez-Segui
 *          Miguel Fernández Cortizas
 */


// Groot connection
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "as2_behavior_tree/action/arm_service.hpp"
#include "as2_behavior_tree/action/echo.hpp"
#include "as2_behavior_tree/action/follow_path.hpp"
#include "as2_behavior_tree/action/get_origin.hpp"
#include "as2_behavior_tree/action/go_to_action.hpp"
#include "as2_behavior_tree/action/go_to_gps_action.hpp"
#include "as2_behavior_tree/action/gps_to_cartesian.hpp"
#include "as2_behavior_tree/action/land_action.hpp"
#include "as2_behavior_tree/action/offboard_service.hpp"
#include "as2_behavior_tree/action/send_event.hpp"
#include "as2_behavior_tree/action/set_origin.hpp"
#include "as2_behavior_tree/action/takeoff_action.hpp"
#include "as2_behavior_tree/condition/is_flying_condition.hpp"
#include "as2_behavior_tree/decorator/wait_for_alert.hpp"
#include "as2_behavior_tree/decorator/wait_for_event.hpp"

// ----------------------------------------------------------------
// This is not from the original code
#include <cmath>
#include "as2_behavior_tree/action/take_off.hpp"
#include "as2_behavior_tree/action/land.hpp"
#include "as2_behavior_tree/action/follow_traj.hpp"
#include "as2_behavior_tree/action/arming.hpp"
#include "as2_behavior_tree/action/offboard.hpp"
// ----------------------------------------------------------------


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_manager");

  // ----------------------------------------------------------------
  // This is not from the original code
  // rclcpp::executors::SingleThreadedExecutor exec;
  // exec.add_node(node);
  // std::thread spin_thread([&exec]() { exec.spin(); });
  // spin_thread.detach();
  // ----------------------------------------------------------------

  node->declare_parameter<std::string>("tree", "");
  node->declare_parameter<bool>("use_groot", false);
  node->declare_parameter<int>("groot_client_port", 1666);
  node->declare_parameter<int>("groot_server_port", 1667);
  node->declare_parameter<int>("server_timeout", 10000);  // miliseconds
  node->declare_parameter<int>("bt_loop_duration", 10);  // miliseconds
  node->declare_parameter<int>("wait_for_service_timeout", 5000);  // miliseconds
  std::string tree_description = node->get_parameter("tree").as_string();
  bool groot_logger = node->get_parameter("use_groot").as_bool();
  int groot_client_port = node->get_parameter("groot_client_port").as_int();
  int groot_server_port = node->get_parameter("groot_server_port").as_int();
  int server_timeout = node->get_parameter("server_timeout").as_int();
  int bt_loop_duration = node->get_parameter("bt_loop_duration").as_int();
  int wait_for_service_timeout = node->get_parameter("wait_for_service_timeout").as_int();

  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<as2_behavior_tree::ArmService>("Arm");
  factory.registerNodeType<as2_behavior_tree::DisarmService>("Disarm");
  factory.registerNodeType<as2_behavior_tree::OffboardService>("Offboard");
  factory.registerNodeType<as2_behavior_tree::TakeoffAction>("TakeOff");
  factory.registerNodeType<as2_behavior_tree::GoToAction>("GoTo");
  factory.registerNodeType<as2_behavior_tree::LandAction>("Land");
  factory.registerNodeType<as2_behavior_tree::IsFlyingCondition>("IsFlying");
  factory.registerNodeType<as2_behavior_tree::WaitForEvent>("WaitForEvent");
  factory.registerNodeType<as2_behavior_tree::WaitForAlert>("WaitForAlert");
  factory.registerNodeType<as2_behavior_tree::SendEvent>("SendEvent");
  factory.registerNodeType<as2_behavior_tree::Echo>("Echo");
  factory.registerNodeType<as2_behavior_tree::SetOrigin>("SetOrigin");
  factory.registerNodeType<as2_behavior_tree::GetOrigin>("GetOrigin");
  factory.registerNodeType<as2_behavior_tree::GpsToCartesian>("GpsToCartesian");
  factory.registerNodeType<as2_behavior_tree::GoToGpsAction>("GoToGps");
  factory.registerNodeType<as2_behavior_tree::FollowPathAction>("FollowPath");

  // ----------------------------------------------------------------------------------------------------------------------------------------
  // This is not from the original code
  factory.registerNodeType<as2_behavior_tree::ArmingAction>("Arming");
  factory.registerNodeType<as2_behavior_tree::OffboardingAction>("Offboarding");
  factory.registerNodeType<as2_behavior_tree::TakingOffAction>("TakingOff");
  factory.registerNodeType<as2_behavior_tree::LandingAction>("Landing");
  factory.registerNodeType<as2_behavior_tree::FollowingTrajAction>("FollowingTraj");

  // Define some reusable trajectories
  as2_msgs::msg::TrajectorySetpoints simple_traj_cir;
  as2_msgs::msg::TrajectorySetpoints complex_traj;
  as2_msgs::msg::TrajectorySetpoints linear_traj_cir;

  simple_traj_cir.header.frame_id = "earth";
  simple_traj_cir.setpoints.resize(4);
  // separate declarations for x and y before the loop
  float x = 30.5f;
  float y = 17.5f;
  for (int i = 0; i < 4; ++i, x -= 3.5f, y -= 3.5f)   // Solar panels are positioned equally separated
  {
    simple_traj_cir.setpoints[i].position.x = x;
    simple_traj_cir.setpoints[i].position.y = y;
    simple_traj_cir.setpoints[i].position.z = 0.0;      // Configured in the server
    simple_traj_cir.setpoints[i].twist.x = 0.0;
    simple_traj_cir.setpoints[i].twist.y = 0.0;
    simple_traj_cir.setpoints[i].twist.z = 0.0;
    simple_traj_cir.setpoints[i].acceleration.x = 0.0;
    simple_traj_cir.setpoints[i].acceleration.y = 0.0;
    simple_traj_cir.setpoints[i].acceleration.z = 0.0;
  }
  for (int i = 0; i < 4; i++)
  {
    if (i == 3)
      simple_traj_cir.setpoints[i].yaw_angle = simple_traj_cir.setpoints[i-1].yaw_angle;
    else
    {
      float x = simple_traj_cir.setpoints[i+1].position.x - simple_traj_cir.setpoints[i].position.x;
      float y = simple_traj_cir.setpoints[i+1].position.y - simple_traj_cir.setpoints[i].position.y;
      simple_traj_cir.setpoints[i].yaw_angle = std::atan2(y, x);
    }
  }

  complex_traj.header.frame_id = "earth";
  complex_traj.setpoints.resize(20);
  {
    int idx = 0;
    // x goes from 20.0 up to 20.0 + 3·3.5 = 30.5 (bottom→top)        // Look at the sdf file of the solar field
    for (int xi = 0; xi < 4; ++xi) 
    {
      float x = 20.0f + xi * 3.5f;
      // y goes from  3.5 up to 17.5 (right→left)
      for (int yi = 0; yi < 5; ++yi) 
      {
        float y = 3.5f + yi * 3.5f;
        complex_traj.setpoints[idx].position.x     = x;
        complex_traj.setpoints[idx].position.y     = y;
        complex_traj.setpoints[idx].position.z     = 0.0f;          // Configured in the server
        complex_traj.setpoints[idx].twist.x        = 0.0f;
        complex_traj.setpoints[idx].twist.y        = 0.0f;
        complex_traj.setpoints[idx].twist.z        = 0.0f;
        complex_traj.setpoints[idx].acceleration.x = 0.0f;
        complex_traj.setpoints[idx].acceleration.y = 0.0f;
        complex_traj.setpoints[idx].acceleration.z = 0.0f;
        ++idx;
      }
    }
  }
  for (int i = 0; i < 20; i++)
  {
    if (i == 19)
      complex_traj.setpoints[i].yaw_angle = complex_traj.setpoints[i-1].yaw_angle;
    else
    {
      float x = complex_traj.setpoints[i+1].position.x - complex_traj.setpoints[i].position.x;
      float y = complex_traj.setpoints[i+1].position.y - complex_traj.setpoints[i].position.y;
      complex_traj.setpoints[i].yaw_angle = std::atan2(y, x);
    }
  }

  linear_traj_cir.header.frame_id = "earth";
  linear_traj_cir.setpoints.resize(1);
  linear_traj_cir.setpoints[0].position.x = 27.0;     // Random solar panel
  linear_traj_cir.setpoints[0].position.y = 10.5;
  linear_traj_cir.setpoints[0].position.z = 0.0;      // Configured in the motion controller
  linear_traj_cir.setpoints[0].twist.x = 0.0;
  linear_traj_cir.setpoints[0].twist.y = 0.0;
  linear_traj_cir.setpoints[0].twist.z = 0.0;
  linear_traj_cir.setpoints[0].acceleration.x = 0.0;
  linear_traj_cir.setpoints[0].acceleration.y = 0.0;
  linear_traj_cir.setpoints[0].acceleration.z = 0.0;
  linear_traj_cir.setpoints[0].yaw_angle = 0.0;       // Configured in the motion controller
  // ----------------------------------------------------------------------------------------------------------------------------------------


  BT::NodeConfiguration * config = new BT::NodeConfiguration();
  // Create the blackboard that will be shared by all of the nodes in the tree
  config->blackboard = BT::Blackboard::create();
  // Put items on the blackboard
  config->blackboard->set<rclcpp::Node::SharedPtr>("node", node);
  config->blackboard->set<std::chrono::milliseconds>(
    "server_timeout", std::chrono::milliseconds(server_timeout));
  config->blackboard->set<std::chrono::milliseconds>(
    "bt_loop_duration", std::chrono::milliseconds(bt_loop_duration));
  config->blackboard->set<std::chrono::milliseconds>(
    "wait_for_service_timeout", std::chrono::milliseconds(wait_for_service_timeout));

  // -------------------------------------------------------------------------------------
  // This is not from the original code
  config->blackboard->set<as2_msgs::msg::TrajectorySetpoints>(
    "simple_traj_cir", simple_traj_cir);
  config->blackboard->set<as2_msgs::msg::TrajectorySetpoints>(
    "complex_traj", complex_traj);
  config->blackboard->set<as2_msgs::msg::TrajectorySetpoints>(
    "linear_traj_cir", linear_traj_cir);
  // -------------------------------------------------------------------------------------
  
  auto tree = factory.createTreeFromFile(tree_description, config->blackboard);

  // LOGGERS
  BT::StdCoutLogger logger_cout(tree);
  std::shared_ptr<BT::PublisherZMQ> groot_pub = nullptr;

  if (groot_logger) {
    groot_pub = std::make_shared<BT::PublisherZMQ>(
      tree, 25U, groot_client_port,
      groot_server_port);
  }

  // to keep track of the number of ticks it took to reach a terminal result
  int ticks = 0;

  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  // BT loop execution rate
  rclcpp::WallRate loopRate(std::chrono::milliseconds(static_cast<int>(bt_loop_duration)));

  // main BT execution loop
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    result = tree.tickRoot();
    ticks++;
    loopRate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

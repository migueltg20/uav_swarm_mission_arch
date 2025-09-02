// Groot connection
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "behaviour_trees_pkg/action/take_off.hpp"
#include "behaviour_trees_pkg/action/land.hpp"
#include "behaviour_trees_pkg/action/follow_traj.hpp"
#include "behaviour_trees_pkg/action/arming.hpp"
#include "behaviour_trees_pkg/action/offboard.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_manager");

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

  factory.registerNodeType<behaviour_trees_pkg::ArmAction>("Arm");
  factory.registerNodeType<behaviour_trees_pkg::OffboardAction>("Offboard");
  factory.registerNodeType<behaviour_trees_pkg::TakeOffAction>("TakeOff");
  factory.registerNodeType<behaviour_trees_pkg::LandAction>("Land");
  factory.registerNodeType<behaviour_trees_pkg::FollowTrajAction>("FollowTraj");

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
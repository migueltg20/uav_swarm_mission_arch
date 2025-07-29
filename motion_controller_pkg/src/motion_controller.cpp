#include "motion_controller_pkg/motion_controller.hpp"  

using namespace std::chrono_literals;



/*!*******************************************************************************************
 *  \file       motion_controller_client.cpp
 *  \brief      Controller client class implementation
 *  \authors    Miguel Tejado García
 ********************************************************************************************/



// --------------------------------------------------------------------------------------------
/* Node initialization */
PidControllerNode::PidControllerNode() : Node("motion_controller"), tf_buffer_(this->get_clock())
{
  /* Creating action server */
  nav_server_ = rclcpp_action::create_server<motion_controller_pkg::action::GoalPoint>(
      this,
      "PrimalBehaviour",
      std::bind(&PidControllerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PidControllerNode::handleCancel, this, std::placeholders::_1),
      std::bind(&PidControllerNode::handleAccepted, this, std::placeholders::_1));


  /* Creating plugin loader and plugin instance */
  RCLCPP_INFO(this->get_logger(), "Inicializando el cargador de plugins...");
  plugin_loader_ = std::make_unique<pluginlib::ClassLoader<as2_motion_controller_plugin_base::ControllerBase>>(
    "as2_motion_controller", "as2_motion_controller_plugin_base::ControllerBase");
  RCLCPP_INFO(this->get_logger(), "Cargador de plugins inicializado correctamente.");

  RCLCPP_INFO(this->get_logger(), "Creando instancia del plugin...");
  controller_plugin_ = plugin_loader_->createSharedInstance("pid_speed_controller::Plugin");
  if (!controller_plugin_) 
  {
    RCLCPP_ERROR(this->get_logger(), "Error: No se pudo crear la instancia del plugin.");
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Instancia del plugin creada correctamente.");


  /* Configuring plugin parameters */
  const std::vector<std::pair<std::string, bool>> bool_params
  {
    {"proportional_limitation", false},
    {"use_bypass", false},
    {"trajectory_control.reset_integral", false},
    {"yaw_control.reset_integral", false},
  };
  const std::vector<std::pair<std::string, double>> double_params
  {
    {"trajectory_control.antiwindup_cte", 0.0},
    {"trajectory_control.alpha", 0.0},
    {"yaw_control.antiwindup_cte", 0.0},
    {"yaw_control.alpha", 0.0},
    {"yaw_control.kp", 0.0},
    {"yaw_control.kd", 0.0},
    {"yaw_control.ki", 0.0},
  };
  for (auto &p : bool_params)   this->declare_parameter<bool>(p.first,  p.second);
  for (auto &p : double_params) this->declare_parameter<double>(p.first, p.second);

  axes_ = {"x","y","z"};
  for (auto &axis : axes_) 
  {
    for (auto &coef : {"kp","kd","ki"}) 
    {
      this->declare_parameter<double>(
        "trajectory_control."+ std::string(coef) +"."+ axis, 0.0);
    }
  }


  /* Initializing plugin */
  RCLCPP_INFO(this->get_logger(), "Llamando al método initialize() del plugin...");
  controller_plugin_->initialize(this);
  RCLCPP_INFO(this->get_logger(), "Método initialize() del plugin llamado correctamente.");


  /* Loading parameters */
  RCLCPP_INFO(this->get_logger(), "Llamando al método updateParams() del plugin...");
  params_ = this->get_parameters(this->list_parameters({}, 10).names);
  if(controller_plugin_ -> updateParams(params_))
  {
    RCLCPP_INFO(this->get_logger(), "Método updateParams() del plugin llamado correctamente.");      
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Error al actualizar parámetros en el plugin.");
    rclcpp::shutdown();
    return;
  }


  /* Configuring control modes (for calculations) */
  /* Input: ENU local frame trajectory (input's config is the more important, which plugin uses
     to know how to perform the calculations, for a trajectory in this case) */
  input_mode_.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;               // or YAW_SPEED or NONE
  input_mode_.control_mode = as2_msgs::msg::ControlMode::TRAJECTORY;
  input_mode_.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;

  /* Output: ENU local frame speed control (only used to indicate the command's output frame) */
  output_mode_.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;              // or YAW_SPEED or NONE
  output_mode_.control_mode = as2_msgs::msg::ControlMode::SPEED;
  output_mode_.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;

  RCLCPP_INFO(this->get_logger(), "Llamando al método setMode() del plugin...");
  if(controller_plugin_ -> setMode(input_mode_, output_mode_))
  {
    RCLCPP_INFO(this->get_logger(), "Modos de control establecidos correctamente.");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Error al establecer los modos de control.");
    rclcpp::shutdown();
    return;
  }


  /* Subscribers, publishers and timer */
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data))
                .reliability(rclcpp::ReliabilityPolicy::BestEffort)
                .durability(rclcpp::DurabilityPolicy::Volatile);

  pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/drone0/self_localization/pose", qos,
    std::bind(&PidControllerNode::poseCallback, this, std::placeholders::_1));

  twist_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/drone0/self_localization/twist", qos,
    std::bind(&PidControllerNode::twistCallback, this, std::placeholders::_1));

  // traj_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
  //   "/goal", rclcpp::QoS(10),
  //   std::bind(&PidControllerNode::trajCallback, this, std::placeholders::_1));
  // traj_goal_defined_ = false;

  twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/drone0/actuator_command/twist", qos);

  timer_ = this->create_wall_timer(100ms, std::bind(&PidControllerNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Todo configurados correctamente.");
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
/* Creating TF listener for base transform */
void PidControllerNode::initTfListener() 
{
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, shared_from_this());
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
/* Node destruction */
PidControllerNode::~PidControllerNode()
{
  RCLCPP_INFO(this->get_logger(), "Destruyendo el nodo y liberando el plugin...");
  controller_plugin_.reset();                                   // First destroy the plugin instance
  plugin_loader_.reset();                                       // Then destroy the loader to allow the library to unload
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
/* Goal handling */
rclcpp_action::GoalResponse PidControllerNode::handleGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const motion_controller_pkg::action::GoalPoint::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received new goal request");
  RCLCPP_WARN(this->get_logger(), "Received goal command: \ndrone_id: %d \nposition: [%f, %f, %f]",
    goal->goal_command.drone_id,
    goal->goal_command.point.pose.position.x,
    goal->goal_command.point.pose.position.y,
    goal->goal_command.point.pose.position.z);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
/* Goal cancellation */
rclcpp_action::CancelResponse PidControllerNode::handleCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_controller_pkg::action::GoalPoint>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
/* Goal acceptance */
void PidControllerNode::handleAccepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_controller_pkg::action::GoalPoint>> goal_handle)
{
  /* Spinning off the execution in another thread to return quickly */
  std::thread{std::bind(&PidControllerNode::executeGoal, this, goal_handle)}.detach();
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
/* Goal execution part 1 */
void PidControllerNode::executeGoal(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<motion_controller_pkg::action::GoalPoint>> goal_handle)
{
  /* Get the goal from the goal handle */
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<motion_controller_pkg::action::GoalPoint::Result>();

  /* Create a PoseStamped from the goal point to use with trajCallback */
  auto point_msg = std::make_shared<geometry_msgs::msg::PoseStamped>(goal->goal_command.point);

  /* Call the trajectory callback with the goal point */
  trajCallback(point_msg);

  /* Set the result and mark the goal as succeeded */
  result->success = true;
  goal_handle->succeed(result);
  
  RCLCPP_INFO(this->get_logger(), "Goal completed successfully");
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
/* Pose callback (it tracks the current pose to update trajectory) */
void PidControllerNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  /* Update the current pose */
  current_pose_ = *msg;
  

  /* Check if trajectories are already defined */
  if (!traj_goal_defined_ && circular_traj_.setpoints.empty()) {
    return;
  }
  

  /* If a trajectory is defined, check if the current pose is close to the desired trajectory point to update it */
  if(traj_goal_defined_)
  {
    if(current_pose_.pose.position.x >= desired_traj_.setpoints[0].position.x - distance_threshold_ &&
        current_pose_.pose.position.x <= desired_traj_.setpoints[0].position.x + distance_threshold_ &&
        current_pose_.pose.position.y >= desired_traj_.setpoints[0].position.y - distance_threshold_ &&
        current_pose_.pose.position.y <= desired_traj_.setpoints[0].position.y + distance_threshold_ &&
        current_pose_.pose.position.z >= desired_traj_.setpoints[0].position.z - distance_threshold_ &&
        current_pose_.pose.position.z <= desired_traj_.setpoints[0].position.z + distance_threshold_)
    {
      /* If the current pose is close to the desired trajectory point, update it */
      desired_traj_.setpoints.erase(desired_traj_.setpoints.begin());

      /* Check if the trajectory is empty */
      if(desired_traj_.setpoints.empty())
      {
        traj_goal_defined_ = false;
        RCLCPP_INFO(this->get_logger(), "La trayectoria ha terminado.");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "El siguiente punto de la trayectoria ha sido alcanzado.");
      }
    }
  }
  else
  {
    if(current_pose_.pose.position.x >= circular_traj_.setpoints[0].position.x - circular_threshold_ &&
        current_pose_.pose.position.x <= circular_traj_.setpoints[0].position.x + circular_threshold_ &&
        current_pose_.pose.position.y >= circular_traj_.setpoints[0].position.y - circular_threshold_ &&
        current_pose_.pose.position.y <= circular_traj_.setpoints[0].position.y + circular_threshold_ &&
        current_pose_.pose.position.z >= circular_traj_.setpoints[0].position.z - circular_threshold_ &&
        current_pose_.pose.position.z <= circular_traj_.setpoints[0].position.z + circular_threshold_)
    {
      /* If the current pose is close to the desired circular trajectory point, update it */
      as2_msgs::msg::TrajectoryPoint aux = circular_traj_.setpoints[0];
      circular_traj_.setpoints.erase(circular_traj_.setpoints.begin());
      circular_traj_.setpoints.push_back(aux); 
    }
  }
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
/* Twist callback (it tracks the current twist) */
void PidControllerNode::twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  current_twist_ = *msg;
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
/* Goal execution part 2 (the important one) */
void PidControllerNode::trajCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  /* Create a goal pose (it'll be the final point of the trajectory)*/
  geometry_msgs::msg::PoseStamped goal = *msg;


  /* Transform the goal pose to the UAV's reference frame using the tf_listener_ */
  geometry_msgs::msg::TransformStamped transform;
  try
  {
    transform = tf_buffer_.lookupTransform(current_pose_.header.frame_id, goal.header.frame_id,
                                            rclcpp::Time(0), rclcpp::Duration(2s));
    tf2::doTransform(goal, goal, transform);
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Error al transformar la pose de destino: %s", ex.what());
    return;
  }


  /* Create a circular trajectory around the goal */
  int circular_points = static_cast<int>(std::round(4 * M_PI * radius_));
  circular_traj_.setpoints.resize(circular_points);
  for (int i = 0; i < circular_points; ++i)
  {
    double theta = 2.0 * M_PI * static_cast<double>(i) / circular_points;
    circular_traj_.setpoints[i].position.x    = goal.pose.position.x + radius_ * std::cos(theta);
    circular_traj_.setpoints[i].position.y    = goal.pose.position.y + radius_ * std::sin(theta);
    circular_traj_.setpoints[i].position.z    = goal.pose.position.z;
    double v = 1.0;
    circular_traj_.setpoints[i].twist.x       = v * std::cos(theta);
    circular_traj_.setpoints[i].twist.y       = v * std::sin(theta);
    circular_traj_.setpoints[i].twist.z       = 0.0;
    circular_traj_.setpoints[i].acceleration.x = 0.0;
    circular_traj_.setpoints[i].acceleration.y = 0.0;
    circular_traj_.setpoints[i].acceleration.z = 0.0;
    circular_traj_.setpoints[i].yaw_angle     = theta + M_PI;  
  }
  RCLCPP_INFO(this->get_logger(), "Trayectoria circular definida.");


  /* Linear trajectory */
  /* Start point (keep or adjust height if necessary) */
  geometry_msgs::msg::PoseStamped start = current_pose_;
  start.pose.position.z = 5.0;

  /* Change to Eigen vectors for calculations */
  Eigen::Vector3d pi(start.pose.position.x, start.pose.position.y, start.pose.position.z);
  Eigen::Vector3d pf(goal.pose.position.x,    goal.pose.position.y,    goal.pose.position.z);

  /* Trajectory size definition */
  as2_msgs::msg::TrajectorySetpoints traj;
  traj.setpoints.resize(num_points_);

  /* Displacement vector, normalized direction and fixed yaw */
  Eigen::Vector3d delta = pf - pi;
  Eigen::Vector3d dir   = delta.normalized();
  double yaw_const = std::atan2(delta.y(), delta.x());

  /* Create linear trajectory waypoints */
  for (int i = 0; i < num_points_; ++i) 
  {
    double t = static_cast<double>(i) / (num_points_ - 1);
    Eigen::Vector3d pos = pi + delta * t;

    /* Interpolate position */
    traj.setpoints[i].position.x = pos.x();
    traj.setpoints[i].position.y = pos.y();
    traj.setpoints[i].position.z = pos.z();

    /* Velocity: zero at start/end, constant direction elsewhere */
    if (i == 0 || i == num_points_ - 1) 
    {
      traj.setpoints[i].twist.x = 0.0;
      traj.setpoints[i].twist.y = 0.0;
      traj.setpoints[i].twist.z = 0.0;
    } 
    else 
    {
      traj.setpoints[i].twist.x = dir.x();
      traj.setpoints[i].twist.y = dir.y();
      traj.setpoints[i].twist.z = dir.z();
    }

    /* Acceleration: zero everywhere */
    traj.setpoints[i].acceleration.x = 0.0;
    traj.setpoints[i].acceleration.y = 0.0;
    traj.setpoints[i].acceleration.z = 0.0;

    /* Yaw fixed */
    traj.setpoints[i].yaw_angle = yaw_const;
  }

  desired_traj_ = traj;
  traj_goal_defined_ = true;
  RCLCPP_INFO(this->get_logger(), "Trayectoria deseada definida.");
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
/* Timer callback (for trajectory updates) */
void PidControllerNode::timerCallback()
{
  if(!traj_goal_defined_ && circular_traj_.setpoints.empty())
  {
    RCLCPP_WARN(this->get_logger(), "No se ha definido la trayectoria deseada.");
    return;
  }
  else
  {
    /* Update reference */
    RCLCPP_INFO(this->get_logger(), "Llamando al método updateReference() del plugin...");
    if(traj_goal_defined_)
      controller_plugin_ -> updateReference(desired_traj_);
    else
      controller_plugin_ -> updateReference(circular_traj_);
    RCLCPP_INFO(this->get_logger(), "Método updateReference() del plugin llamado correctamente.");


    /* Update state */
    RCLCPP_INFO(this->get_logger(), "Llamando al método updateState() del plugin...");
    controller_plugin_ -> updateState(current_pose_, current_twist_);
    RCLCPP_INFO(this->get_logger(), "Método updateState() del plugin llamado correctamente.");


    /* Calculate output */
    RCLCPP_INFO(this->get_logger(), "Llamando al método computeOutput() del plugin...");
    if(controller_plugin_ -> computeOutput(0.100, unused_pose_, command_twist_, unused_thrust_))
    {
      RCLCPP_INFO(this->get_logger(), "Método computeOutput() del plugin llamado correctamente.");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Error al calcular la salida del plugin.");
      rclcpp::shutdown();
      return;
    }


    /* Transform the output TwistStamped to the drone's base_link frame */
    geometry_msgs::msg::TransformStamped transform;
    try
    {
      transform = tf_buffer_.lookupTransform("drone0/base_link", command_twist_.header.frame_id,
                                              rclcpp::Time(0), rclcpp::Duration(2s));

      /* Transform the linear velocity */
      tf2::Vector3 linear_velocity(
        command_twist_.twist.linear.x,
        command_twist_.twist.linear.y,
        command_twist_.twist.linear.z
      );
      tf2::Vector3 transformed_linear_velocity = tf2::Transform(
        tf2::Quaternion(
          transform.transform.rotation.x,
          transform.transform.rotation.y,
          transform.transform.rotation.z,
          transform.transform.rotation.w
        )
      ) * linear_velocity;

      /* Transform the angular velocity */
      tf2::Vector3 angular_velocity(
        command_twist_.twist.angular.x,
        command_twist_.twist.angular.y,
        command_twist_.twist.angular.z
      );
      tf2::Vector3 transformed_angular_velocity = tf2::Transform(
        tf2::Quaternion(
          transform.transform.rotation.x,
          transform.transform.rotation.y,
          transform.transform.rotation.z,
          transform.transform.rotation.w
        )
      ) * angular_velocity;

      /* Update the TwistStamped message with the transformed velocities */
      command_twist_.twist.linear.x = transformed_linear_velocity.x();
      command_twist_.twist.linear.y = transformed_linear_velocity.y();
      command_twist_.twist.linear.z = transformed_linear_velocity.z();

      command_twist_.twist.angular.x = transformed_angular_velocity.x();
      command_twist_.twist.angular.y = transformed_angular_velocity.y();
      command_twist_.twist.angular.z = transformed_angular_velocity.z();

      command_twist_.header.frame_id = "drone0/base_link";

      RCLCPP_INFO(this->get_logger(), "'command_twist_' transformada.");
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Error al transformar la salida del plugin: %s", ex.what());
      return;
    }

    /* Publish velocity command */
    twist_publisher_->publish(command_twist_);
    RCLCPP_INFO(this->get_logger(), "Comando de velocidad publicado.");
  }
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  // Disable Fast RTPS shared memory transport
  ::setenv("RMW_FASTRTPS_SHARED_MEMORY_ENABLED", "0", 1);
  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PidControllerNode>();

  node->initTfListener();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
// --------------------------------------------------------------------------------------------
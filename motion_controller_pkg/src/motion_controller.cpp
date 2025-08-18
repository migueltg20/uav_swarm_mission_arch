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
  /* Creating action servers */
  for (int i = 0; i < 4; ++i) 
  {
    nav_servers_[i] = rclcpp_action::create_server<GoalPoint>(
        this,
        "/drone" + std::to_string(i) + "/PrimalBehaviour",
        std::bind(&PidControllerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PidControllerNode::handleCancel, this, std::placeholders::_1),
        std::bind(&PidControllerNode::handleAccepted, this, std::placeholders::_1));
  }


  /* Creating plugin loader */
  RCLCPP_INFO(this->get_logger(), "Inicializando el cargador de plugins...");
  plugin_loader_ = std::make_unique<pluginlib::ClassLoader<as2_motion_controller_plugin_base::ControllerBase>>(
    "as2_motion_controller", "as2_motion_controller_plugin_base::ControllerBase");
  RCLCPP_INFO(this->get_logger(), "Cargador de plugins inicializado correctamente.");


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

  auto axes = {"x","y","z"};
  for (auto &axis : axes) 
  {
    for (auto &coef : {"kp","kd","ki"}) 
    {
      this->declare_parameter<double>(
        "trajectory_control."+ std::string(coef) +"."+ axis, 0.0);
    }
  }

  for (int i = 0; i < 4; ++i) 
  {
    /* Subscribers, publishers and timer */
    pose_subscribers_[i] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/drone" + std::to_string(i) + "/self_localization/pose", qos_,
      [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        this->poseCallback(i, msg);
      });

    twist_subscribers_[i] = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/drone" + std::to_string(i) + "/self_localization/twist", qos_,
      [this, i](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        this->twistCallback(i, msg);
      });

    twist_publishers_[i] = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/drone" + std::to_string(i) + "/actuator_command/twist", qos_);

    traj_goal_defined_[i] = false;

    timers_[i] = this->create_wall_timer(100ms, [this, i]() { this->timerCallback(i); });
  }
  RCLCPP_INFO(this->get_logger(), "Subscribers, publishers and timers initialized");
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
/* Creating TF listener for base transform */
void PidControllerNode::initTfListener() 
{
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, shared_from_this());
  RCLCPP_INFO(this->get_logger(), "TF listener initialized");
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
/* Node destruction */
PidControllerNode::~PidControllerNode()
{
  RCLCPP_INFO(this->get_logger(), "Destruyendo el nodo y liberando plugins...");
  for (int i = 0; i < 4; ++i) 
  {
    if (controllers_[i]) 
    {
      controllers_[i].reset();  // Reset each individual shared_ptr
    }
  }
  plugin_loader_.reset();    // Then unload the class‐loader so libraries can unload
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
/* Goal handling */
rclcpp_action::GoalResponse PidControllerNode::handleGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const GoalPoint::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received new goal request");
  RCLCPP_WARN(this->get_logger(), "Received goal command: \ndrone_id: %d \nposition: [%f, %f, %f] \ncircular = %d \nradius = %f \nControl mode: [%d, %d, %d]",
    goal->goal_command.drone_id,
    goal->goal_command.point.pose.position.x,
    goal->goal_command.point.pose.position.y,
    goal->goal_command.point.pose.position.z,
    goal->goal_command.circular,
    goal->goal_command.radius,
    goal->goal_command.control_mode.control_mode,
    goal->goal_command.control_mode.yaw_mode,
    goal->goal_command.control_mode.reference_frame);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
void PidControllerNode::cleanupGoal(int drone_id, bool cleanup_controller)
{
  /* Reset trajectory state */
  traj_goal_defined_[drone_id] = false;
  desired_traj_[drone_id].setpoints.clear();
  circular_traj_[drone_id].setpoints.clear();
  
  /* Remove from active goals */
  active_goals_[drone_id] = nullptr;
  
  /* Optional: Clean up controller instance */
  if (cleanup_controller) 
  {
    controllers_[drone_id] = nullptr;
    params_[drone_id].clear();
    input_mode_[drone_id] = as2_msgs::msg::ControlMode();
    RCLCPP_INFO(this->get_logger(), "Controller instance for drone %d cleaned up", drone_id);
  }
  
  RCLCPP_INFO(this->get_logger(), "Goal resources for drone %d cleaned up", drone_id);
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
/* Goal cancellation */
rclcpp_action::CancelResponse PidControllerNode::handleCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoalPoint>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

  /* Find and clean up the cancelled goal */
  for (int i = 0; i < 4; ++i) 
  {
    std::lock_guard<std::mutex> lock(drone_mutexes_[i]);

    if (active_goals_[i] == goal_handle) 
    {
        RCLCPP_INFO(this->get_logger(), "Cleaning up cancelled goal for drone %d", i);
        cleanupGoal(i, false);
        break;
    }
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
/* Goal acceptance */
void PidControllerNode::handleAccepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoalPoint>> goal_handle)
{
  RCLCPP_WARN(this->get_logger(), "Goal accepted, starting execution");
  /* Spinning off the execution in another thread to return quickly */
  std::thread{std::bind(&PidControllerNode::executeGoal, this, goal_handle)}.detach();
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
/* Goal execution part 1 */
void PidControllerNode::executeGoal(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoalPoint>> goal_handle)
{
  /* Get the goal from the goal handle */
  const auto goal = goal_handle->get_goal();
  const auto drone_id = goal->goal_command.drone_id;

  RCLCPP_WARN(this->get_logger(), "Received goal command: \ndrone_id: %d \nposition: [%f, %f, %f] \ncircular = %d \nradius = %f \nControl mode: [%d, %d, %d]",
    goal->goal_command.drone_id,
    goal->goal_command.point.pose.position.x,
    goal->goal_command.point.pose.position.y,
    goal->goal_command.point.pose.position.z,
    goal->goal_command.circular,
    goal->goal_command.radius,
    goal->goal_command.control_mode.control_mode,
    goal->goal_command.control_mode.yaw_mode,
    goal->goal_command.control_mode.reference_frame);

  std::lock_guard<std::mutex> lock(drone_mutexes_[drone_id]);

  active_goals_[drone_id] = goal_handle;


  /* Create a new controller instance if it doesn't exist */
  if (!controllers_[drone_id]) 
  {
    controllers_[drone_id] = plugin_loader_->createSharedInstance("pid_speed_controller::Plugin");

    if (!controllers_[drone_id]) 
    {
      RCLCPP_ERROR(this->get_logger(), "Error: No se pudo crear la instancia del plugin para el UAV %d.", drone_id);
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Instancia del plugin del drone %d creada correctamente.", drone_id);

    /* Initializing plugin */
    RCLCPP_INFO(this->get_logger(), "Llamando al método initialize() del plugin %d...", drone_id);
    controllers_[drone_id]->initialize(this);
    RCLCPP_INFO(this->get_logger(), "Método initialize() del plugin %d llamado correctamente.", drone_id);
  }


  /* Loading parameters */
  RCLCPP_INFO(this->get_logger(), "Llamando al método updateParams() del plugin %d...", drone_id);
  params_[drone_id] = this->get_parameters(this->list_parameters({}, 10).names);
  if(controllers_[drone_id]->updateParams(params_[drone_id]))
  {
    RCLCPP_INFO(this->get_logger(), "Método updateParams() del plugin %d llamado correctamente.", drone_id);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Error al actualizar parámetros en el plugin %d.", drone_id);
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Parámetros: ");
  for (const auto &param : params_[drone_id])
  {
    RCLCPP_WARN(this->get_logger(), "  %s: %s", param.get_name().c_str(), param.value_to_string().c_str());
  }

  /* Configuring control modes (for calculations) */
  /* Input: ENU local frame trajectory (the input's config is the most important, as it determines how the plugin
     performs the calculations; in this case, for a trajectory) */
  input_mode_[drone_id] = goal->goal_command.control_mode;

  /* Output: ENU local frame speed control (used only to indicate the frame of the command's output) */
  output_mode_.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;              // or YAW_SPEED or NONE (not functional with YAW_SPEED)
  output_mode_.control_mode = as2_msgs::msg::ControlMode::SPEED;
  output_mode_.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;

  RCLCPP_INFO(this->get_logger(), "Llamando al método setMode() del plugin %d...", drone_id);
  if(controllers_[drone_id]->setMode(input_mode_[drone_id], output_mode_))
  {
    RCLCPP_INFO(this->get_logger(), "Modos de control para el UAV %d establecidos correctamente.", drone_id);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Error al establecer los modos de control para el UAV %d.", drone_id);
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Modos de control para el UAV %d establecidos correctamente.", drone_id);


  /* Call the trajectory callback with the goal */
  trajCallback(goal->goal_command);
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
/* Pose callback (it tracks the current pose to update trajectory) */
void PidControllerNode::poseCallback(int drone_id, const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(drone_mutexes_[drone_id]);

  /* Update the current pose */
  current_pose_[drone_id] = *msg;

  /* Check if the goal is active */
  if (active_goals_[drone_id] == nullptr) 
  {
    return;  // No active goal
  }

  if (!active_goals_[drone_id] || !active_goals_[drone_id]->is_active()) 
  {
    return; // Goal handle is invalid or not active
  }
  auto result = std::make_shared<GoalPoint::Result>();


  /* Check if trajectories are already defined */
  if (!traj_goal_defined_[drone_id] && circular_traj_[drone_id].setpoints.empty()) 
  {
    return;
  }
  

  /* If a trajectory is defined, check if the current pose is close to the desired trajectory point to update it */
  if(traj_goal_defined_[drone_id])
  {
    if(current_pose_[drone_id].pose.position.x >= desired_traj_[drone_id].setpoints[0].position.x - distance_threshold_ &&
        current_pose_[drone_id].pose.position.x <= desired_traj_[drone_id].setpoints[0].position.x + distance_threshold_ &&
        current_pose_[drone_id].pose.position.y >= desired_traj_[drone_id].setpoints[0].position.y - distance_threshold_ &&
        current_pose_[drone_id].pose.position.y <= desired_traj_[drone_id].setpoints[0].position.y + distance_threshold_ &&
        current_pose_[drone_id].pose.position.z >= desired_traj_[drone_id].setpoints[0].position.z - distance_threshold_ &&
        current_pose_[drone_id].pose.position.z <= desired_traj_[drone_id].setpoints[0].position.z + distance_threshold_)
    {
      /* If the current pose is close to the desired trajectory point, update it */
      desired_traj_[drone_id].setpoints.erase(desired_traj_[drone_id].setpoints.begin());

      /* Check if the trajectory is empty */
      if(desired_traj_[drone_id].setpoints.empty())
      {
        traj_goal_defined_[drone_id] = false;
        RCLCPP_INFO(this->get_logger(), "La trayectoria del UAV %d ha terminado.", drone_id);

        /* Set the result and mark the goal as succeeded */
        result->success = true;
        active_goals_[drone_id]->succeed(result);

        RCLCPP_INFO(this->get_logger(), "Goal for UAV %d completed successfully", drone_id);

        /* Clean up completed goal */
        active_goals_[drone_id] = nullptr;
      }
    }
  }
  else
  {
    if(current_pose_[drone_id].pose.position.x >= circular_traj_[drone_id].setpoints[0].position.x - circular_threshold_ &&
        current_pose_[drone_id].pose.position.x <= circular_traj_[drone_id].setpoints[0].position.x + circular_threshold_ &&
        current_pose_[drone_id].pose.position.y >= circular_traj_[drone_id].setpoints[0].position.y - circular_threshold_ &&
        current_pose_[drone_id].pose.position.y <= circular_traj_[drone_id].setpoints[0].position.y + circular_threshold_ &&
        current_pose_[drone_id].pose.position.z >= circular_traj_[drone_id].setpoints[0].position.z - circular_threshold_ &&
        current_pose_[drone_id].pose.position.z <= circular_traj_[drone_id].setpoints[0].position.z + circular_threshold_)
    {
      /* If the current pose is close to the desired circular trajectory point, update it */
      as2_msgs::msg::TrajectoryPoint aux = circular_traj_[drone_id].setpoints[0];
      circular_traj_[drone_id].setpoints.erase(circular_traj_[drone_id].setpoints.begin());
      circular_traj_[drone_id].setpoints.push_back(aux);
    }
  }
}
// --------------------------------------------------------------------------------------------



// TODO: Change feedback and where it is sent if needed
// --------------------------------------------------------------------------------------------
/* Twist callback (it tracks the current twist and send feedback) */
void PidControllerNode::twistCallback(int drone_id, const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(drone_mutexes_[drone_id]);

  /* Check if there's an active goal for this drone */
  if (active_goals_[drone_id] == nullptr) 
  {
    return; // No active goal
  }

  if (!active_goals_[drone_id] || !active_goals_[drone_id]->is_active()) 
  {
    return; // Goal handle is invalid or not active
  }


  current_twist_[drone_id] = *msg;


  /* Send feedback */
  auto feedback = std::make_shared<GoalPoint::Feedback>();
  feedback->current_pose  = current_pose_[drone_id];
  feedback->current_speed = current_twist_[drone_id];
  active_goals_[drone_id]->publish_feedback(feedback);
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
/* Goal execution part 2 (the important one) */
void PidControllerNode::trajCallback(const GoalCommand & msg)
{
  auto result = std::make_shared<GoalPoint::Result>();
  const auto goal = msg;
  uint8_t drone_id = goal.drone_id;

  RCLCPP_WARN(this->get_logger(), "Received goal command: \ndrone_id: %d \nposition: [%f, %f, %f] \ncircular = %d \nradius = %f \nControl mode: [%d, %d, %d]",
    goal.drone_id,
    goal.point.pose.position.x,
    goal.point.pose.position.y,
    goal.point.pose.position.z,
    goal.circular,
    goal.radius,
    goal.control_mode.control_mode,
    goal.control_mode.yaw_mode,
    goal.control_mode.reference_frame);

  /* Check if goal is still active before proceeding */
  if (active_goals_[drone_id] == nullptr || !active_goals_[drone_id]->is_active()) 
  {
    RCLCPP_WARN(this->get_logger(), "Goal for drone %d is no longer active", drone_id);
    return;
  }


  /* Transform the goal pose to the UAV's reference frame using the tf_listener_ */
  geometry_msgs::msg::TransformStamped transform;
  try
  {
    transform = tf_buffer_.lookupTransform(
      current_pose_[drone_id].header.frame_id,
      goal.point.header.frame_id,
      rclcpp::Time(0), rclcpp::Duration(2s)
    );

    tf2::doTransform(goal.point, goal.point, transform);
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Error al transformar la pose de destino: %s", ex.what());
    return;
  }


  switch (goal.circular)
  {
  case 0:   // If the goal includes a circular trajectory
    if (goal.trajectory.setpoints.empty() && !goal.point.header.frame_id.empty())    // If linear trajectory from initial point to goal
    {
      /* Start point (keep or adjust height if necessary) */
      geometry_msgs::msg::PoseStamped start = current_pose_[drone_id];
      start.pose.position.z = 5.0;

      /* Change to Eigen vectors for calculations */
      Eigen::Vector3d pi(start.pose.position.x, start.pose.position.y, start.pose.position.z);
      Eigen::Vector3d pf(goal.point.pose.position.x,    goal.point.pose.position.y,    goal.point.pose.position.z);

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

      desired_traj_[drone_id] = traj;
      traj_goal_defined_[drone_id] = true;
    }
    else if (!goal.trajectory.setpoints.empty())    // If a custom trajectory is provided
    {
      desired_traj_[drone_id] = goal.trajectory;
      traj_goal_defined_[drone_id] = true;
    }
    else if (goal.point.header.frame_id.empty())    // If the goal is not defined
    {
      RCLCPP_ERROR(this->get_logger(), "Error: El objetivo no está definido para el UAV %d.", drone_id);

      /* Set the result and mark the goal as succeeded */
      result->success = false;
      active_goals_[drone_id]->abort(result);

      /* Clean up resources */
      cleanupGoal(drone_id, false);

      return;
    }

    break;

  case 1:  // If the goal doesn't include a circular trajectory
    if (goal.radius <= 0.0) 
    {
      RCLCPP_ERROR(this->get_logger(), "Error: El radio de la trayectoria circular del UAV %d debe ser mayor que cero.", drone_id);

      /* Set the result and mark the goal as succeeded */
      result->success = false;
      active_goals_[drone_id]->abort(result);

      /* Clean up resources */
      cleanupGoal(drone_id, false);

      return;
    }


    /* Calculate the number of points for the circular trajectory */
    int circular_points = static_cast<int>(std::round(4 * M_PI * goal.radius));

    if (goal.trajectory.setpoints.empty() && !goal.point.header.frame_id.empty())    // If linear trajectory from initial point to goal + circular trajectory around goal
    {
      /* Circular trajectory */
      circular_traj_[drone_id].setpoints.resize(circular_points);
      for (int i = 0; i < circular_points; ++i)
      {
        double theta = 2.0 * M_PI * static_cast<double>(i) / circular_points;
        circular_traj_[drone_id].setpoints[i].position.x    = goal.point.pose.position.x + goal.radius * std::cos(theta);
        circular_traj_[drone_id].setpoints[i].position.y    = goal.point.pose.position.y + goal.radius * std::sin(theta);
        circular_traj_[drone_id].setpoints[i].position.z    = goal.point.pose.position.z;
        double v = 1.0;
        circular_traj_[drone_id].setpoints[i].twist.x       = v * std::cos(theta);
        circular_traj_[drone_id].setpoints[i].twist.y       = v * std::sin(theta);
        circular_traj_[drone_id].setpoints[i].twist.z       = 0.0;
        circular_traj_[drone_id].setpoints[i].acceleration.x = 0.0;
        circular_traj_[drone_id].setpoints[i].acceleration.y = 0.0;
        circular_traj_[drone_id].setpoints[i].acceleration.z = 0.0;
        circular_traj_[drone_id].setpoints[i].yaw_angle     = theta + M_PI;
      }


      /* Linear trajectory */
      /* Start point (keep or adjust height if necessary) */
      geometry_msgs::msg::PoseStamped start = current_pose_[drone_id];
      start.pose.position.z = 5.0;

      /* Change to Eigen vectors for calculations */
      Eigen::Vector3d pi(start.pose.position.x, start.pose.position.y, start.pose.position.z);
      Eigen::Vector3d pf(goal.point.pose.position.x,    goal.point.pose.position.y,    goal.point.pose.position.z);

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

      desired_traj_[drone_id] = traj;
      traj_goal_defined_[drone_id] = true;
    }
    else if (goal.trajectory.setpoints.empty() && goal.point.header.frame_id.empty())   // If circular trajectory around actual position only
    {
      /* Circular trajectory */
      circular_traj_[drone_id].setpoints.resize(circular_points);
      for (int i = 0; i < circular_points; ++i)
      {
        double theta = 2.0 * M_PI * static_cast<double>(i) / circular_points;
        circular_traj_[drone_id].setpoints[i].position.x    = current_pose_[drone_id].pose.position.x + goal.radius * std::cos(theta);
        circular_traj_[drone_id].setpoints[i].position.y    = current_pose_[drone_id].pose.position.y + goal.radius * std::sin(theta);
        circular_traj_[drone_id].setpoints[i].position.z    = 5.0;    // Keep or adjust height if necessary
        double v = 1.0;
        circular_traj_[drone_id].setpoints[i].twist.x       = v * std::cos(theta);
        circular_traj_[drone_id].setpoints[i].twist.y       = v * std::sin(theta);
        circular_traj_[drone_id].setpoints[i].twist.z       = 0.0;
        circular_traj_[drone_id].setpoints[i].acceleration.x = 0.0;
        circular_traj_[drone_id].setpoints[i].acceleration.y = 0.0;
        circular_traj_[drone_id].setpoints[i].acceleration.z = 0.0;
        circular_traj_[drone_id].setpoints[i].yaw_angle     = theta + M_PI;
      }
    }
    else if (!goal.trajectory.setpoints.empty())    // If a custom trajectory is provided + circular trajectory around goal
    {
      desired_traj_[drone_id] = goal.trajectory;
      traj_goal_defined_[drone_id] = true;

      
      /* Circular trajectory */
      circular_traj_[drone_id].setpoints.resize(circular_points);
      for (int i = 0; i < circular_points; ++i)
      {
        double theta = 2.0 * M_PI * static_cast<double>(i) / circular_points;
        circular_traj_[drone_id].setpoints[i].position.x    = goal.trajectory.setpoints.back().position.x + goal.radius * std::cos(theta);
        circular_traj_[drone_id].setpoints[i].position.y    = goal.trajectory.setpoints.back().position.y + goal.radius * std::sin(theta);
        circular_traj_[drone_id].setpoints[i].position.z    = goal.trajectory.setpoints.back().position.z;
        double v = 1.0;
        circular_traj_[drone_id].setpoints[i].twist.x       = v * std::cos(theta);
        circular_traj_[drone_id].setpoints[i].twist.y       = v * std::sin(theta);
        circular_traj_[drone_id].setpoints[i].twist.z       = 0.0;
        circular_traj_[drone_id].setpoints[i].acceleration.x = 0.0;
        circular_traj_[drone_id].setpoints[i].acceleration.y = 0.0;
        circular_traj_[drone_id].setpoints[i].acceleration.z = 0.0;
        circular_traj_[drone_id].setpoints[i].yaw_angle     = theta + M_PI;
      }
    }

    break;
  }
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
/* Timer callback (for trajectory updates) */
void PidControllerNode::timerCallback(int drone_id)
{
  std::lock_guard<std::mutex> lock(drone_mutexes_[drone_id]);

  /* Check if there's an active goal for this drone */
  if (active_goals_[drone_id] == nullptr) 
  {
    /* Clean up unused controller instances */
    if (controllers_[drone_id] != nullptr) {
      RCLCPP_INFO(this->get_logger(), "Cleaning up unused controller for drone %d", drone_id);
      cleanupGoal(drone_id, true);  // Clean up controller too
    }
    return; // No active goal
  }

  if (!active_goals_[drone_id] || !active_goals_[drone_id]->is_active()) 
  {
    /* Goal is no longer active, clean up */
    RCLCPP_WARN(this->get_logger(), "Detected inactive goal for drone %d, cleaning up", drone_id);
    cleanupGoal(drone_id, false);
    return;
  }

  if(!traj_goal_defined_[drone_id] && circular_traj_[drone_id].setpoints.empty())
  {
    RCLCPP_WARN(this->get_logger(), "No se ha definido la trayectoria deseada para el UAV %d.", drone_id);
    return;
  }
  else
  {
    if (!controllers_[drone_id]) 
    {
      RCLCPP_WARN(this->get_logger(), "No controller for UAV %d", drone_id);
      return;
    }

    /* Update reference */
    RCLCPP_INFO(this->get_logger(), "Llamando al método updateReference() del plugin %d...", drone_id);
    if (traj_goal_defined_[drone_id]) 
    {
      controllers_[drone_id]->updateReference(desired_traj_[drone_id]);
    } 
    else 
    {
      controllers_[drone_id]->updateReference(circular_traj_[drone_id]);
    }
    RCLCPP_INFO(this->get_logger(), "Método updateReference() del plugin %d llamado correctamente.", drone_id);


    /* Update state */
    RCLCPP_INFO(this->get_logger(), "Llamando al método updateState() del plugin %d...", drone_id);
    controllers_[drone_id]->updateState(current_pose_[drone_id], current_twist_[drone_id]);
    RCLCPP_INFO(this->get_logger(), "Método updateState() del plugin %d llamado correctamente.", drone_id);

    geometry_msgs::msg::PoseStamped unused_pose;
    geometry_msgs::msg::TwistStamped command_twist;
    as2_msgs::msg::Thrust unused_thrust;


    /* Calculate output */
    RCLCPP_INFO(this->get_logger(), "Llamando al método computeOutput() del plugin %d...", drone_id);
    if(controllers_[drone_id]->computeOutput(0.100, unused_pose, command_twist, unused_thrust))
    {
      RCLCPP_INFO(this->get_logger(), "Método computeOutput() del plugin %d llamado correctamente.", drone_id);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Error al calcular la salida del plugin %d.", drone_id);
      rclcpp::shutdown();
      return;
    }


    /* Transform the output TwistStamped to the drone's base_link frame */
    geometry_msgs::msg::TransformStamped transform;
    try
    {
      transform = tf_buffer_.lookupTransform("drone" + std::to_string(drone_id) + "/base_link", command_twist.header.frame_id,
                                              rclcpp::Time(0), rclcpp::Duration(2s));

      /* Transform the linear velocity */
      tf2::Vector3 linear_velocity(
        command_twist.twist.linear.x,
        command_twist.twist.linear.y,
        command_twist.twist.linear.z
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
        command_twist.twist.angular.x,
        command_twist.twist.angular.y,
        command_twist.twist.angular.z
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
      command_twist.twist.linear.x = transformed_linear_velocity.x();
      command_twist.twist.linear.y = transformed_linear_velocity.y();
      command_twist.twist.linear.z = transformed_linear_velocity.z();

      command_twist.twist.angular.x = transformed_angular_velocity.x();
      command_twist.twist.angular.y = transformed_angular_velocity.y();
      command_twist.twist.angular.z = transformed_angular_velocity.z();

      command_twist.header.frame_id = "drone" + std::to_string(drone_id) + "/base_link";

      RCLCPP_INFO(this->get_logger(), "'command_twist' transformada.");
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Error al transformar la salida del plugin: %s", ex.what());
      return;
    }

    /* Publish velocity command */
    twist_publishers_[drone_id]->publish(command_twist);
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
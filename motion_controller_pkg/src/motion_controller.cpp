#include "motion_controller_pkg/motion_controller.hpp"  

using namespace std::chrono_literals;

/*!*******************************************************************************************
 *  \file       motion_controller.cpp
 *  \brief      Controller class implementation + action servers
 *  \authors    Miguel Tejado García
 ********************************************************************************************/

// --------------------------------------------------------------------------------------------
/* Node initialization - sets up action servers, plugin loader, and subscribers/publishers for 4 drones */
PidControllerNode::PidControllerNode() : Node("motion_controller"), tf_buffer_(this->get_clock())
{
  /* Creating action servers for each drone (0-3) */
  for (int i = 0; i < 4; ++i) 
  {
    nav_servers_[i] = rclcpp_action::create_server<GoalPoint>(
        this,
        "/drone" + std::to_string(i) + "/PrimalBehaviour", // Action server topic per drone
        std::bind(&PidControllerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PidControllerNode::handleCancel, this, std::placeholders::_1),
        std::bind(&PidControllerNode::handleAccepted, this, std::placeholders::_1));
  }

  /* Creating plugin loader for motion controller plugins */
  plugin_loader_ = std::make_unique<pluginlib::ClassLoader<as2_motion_controller_plugin_base::ControllerBase>>(
    "as2_motion_controller", "as2_motion_controller_plugin_base::ControllerBase");

  /* Configuring plugin parameters - boolean parameters for controller behavior */
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

  /* Configuring plugin parameters - double parameters for PID gains and limits */
  auto axes = {"x","y","z"};
  for (auto &axis : axes) 
  {
    for (auto &coef : {"kp","kd","ki"}) // PID coefficients
    {
      this->declare_parameter<double>(
        "trajectory_control."+ std::string(coef) +"."+ axis, 0.0);
    }
  }

  /* Initialize communication interfaces for each drone */
  for (int i = 0; i < 4; ++i) 
  {
    /* Pose subscriber - receives current position and orientation */
    pose_subscribers_[i] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/drone" + std::to_string(i) + "/self_localization/pose", qos_,
      [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        this->poseCallback(i, msg);
      });

    /* Twist subscriber - receives current linear and angular velocities */
    twist_subscribers_[i] = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/drone" + std::to_string(i) + "/self_localization/twist", qos_,
      [this, i](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        this->twistCallback(i, msg);
      });

    /* Twist publisher - sends velocity commands to drone actuators */
    twist_publishers_[i] = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/drone" + std::to_string(i) + "/actuator_command/twist", qos_);

    traj_goal_defined_[i] = false; // Initialize trajectory state

    /* Timer for periodic control loop execution (10Hz) */
    timers_[i] = this->create_wall_timer(100ms, [this, i]() { this->timerCallback(i); });
  }
}
// --------------------------------------------------------------------------------------------



/* Creating TF listener for coordinate frame transformations */
void PidControllerNode::initTfListener() 
{
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, shared_from_this());
}
// --------------------------------------------------------------------------------------------



/* Node destruction - properly cleanup controllers and plugin loader */
PidControllerNode::~PidControllerNode()
{
  RCLCPP_INFO(this->get_logger(), "Destroying PID controller node...");
  /* Reset each controller instance before unloading plugins */
  for (int i = 0; i < 4; ++i) 
  {
    if (controllers_[i]) 
    {
      controllers_[i].reset();  // Reset each individual shared_ptr
    }
  }
  plugin_loader_.reset();    // Then unload the class loader so libraries can unload
}
// --------------------------------------------------------------------------------------------



/* Goal handling - decides whether to accept or reject incoming action goals */
rclcpp_action::GoalResponse PidControllerNode::handleGoal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const GoalPoint::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received new goal request");
  /* Log goal details for debugging */
  RCLCPP_WARN(this->get_logger(), "Received goal command: \ndrone_id: %d \ntake off: %f \nland: %d\nposition: [%f, %f, %f] \ncircular = %d \nradius = %f",
    goal->goal_command.drone_id,
    goal->goal_command.takeoff,
    goal->goal_command.land,
    goal->goal_command.point.pose.position.x,
    goal->goal_command.point.pose.position.y,
    goal->goal_command.point.pose.position.z,
    goal->goal_command.circular,
    goal->goal_command.radius);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
// --------------------------------------------------------------------------------------------



/* Clean up goal-related data and reset trajectory state for a specific drone */
void PidControllerNode::cleanupGoal(int drone_id)
{
  /* Reset trajectory state */
  traj_goal_defined_[drone_id] = false;
  desired_traj_[drone_id].setpoints.clear();
  circular_traj_[drone_id].setpoints.clear();
  
  /* Remove from active goals */
  active_goals_[drone_id] = nullptr;
}
// --------------------------------------------------------------------------------------------



/* Goal cancellation - handle requests to cancel active goals */
rclcpp_action::CancelResponse PidControllerNode::handleCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoalPoint>> goal_handle)
{
  /* Find and clean up the cancelled goal */
  for (int i = 0; i < 4; ++i) 
  {
    std::lock_guard<std::mutex> lock(drone_mutexes_[i]);

    if (active_goals_[i] == goal_handle) 
    {
        RCLCPP_INFO(this->get_logger(), "Cleaning up cancelled goal for drone %d", i);
        cleanupGoal(i);
        break;
    }
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}
// --------------------------------------------------------------------------------------------



/* Goal acceptance - start execution of accepted goals in separate thread */
void PidControllerNode::handleAccepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoalPoint>> goal_handle)
{
  /* Spinning off the execution in another thread to return quickly */
  std::thread{std::bind(&PidControllerNode::executeGoal, this, goal_handle)}.detach();
}
// --------------------------------------------------------------------------------------------



/* Goal execution part 1 - initialize controller and set up control modes */
void PidControllerNode::executeGoal(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoalPoint>> goal_handle)
{
  /* Get the goal from the goal handle */
  const auto goal = goal_handle->get_goal();
  const auto drone_id = goal->goal_command.drone_id;

  std::lock_guard<std::mutex> lock(drone_mutexes_[drone_id]);

  active_goals_[drone_id] = goal_handle;

  /* Create a new controller instance if it doesn't exist */
  if (!controllers_[drone_id]) 
  {
    /* Load PID speed controller plugin */
    controllers_[drone_id] = plugin_loader_->createSharedInstance("pid_speed_controller::Plugin");

    if (!controllers_[drone_id]) 
    {
      RCLCPP_ERROR(this->get_logger(), "Error: Cannot create plugin instance for UAV %d.", drone_id);
      rclcpp::shutdown();
      return;
    }

    /* Initialize plugin with node reference */
    controllers_[drone_id]->initialize(this);
  }

  /* Load and update controller parameters */
  params_[drone_id] = this->get_parameters(this->list_parameters({}, 10).names);
  if(!controllers_[drone_id]->updateParams(params_[drone_id]))
  {
    RCLCPP_ERROR(this->get_logger(), "Error updating parameters in plugin %d.", drone_id);
    rclcpp::shutdown();
    return;
  }

  /* Configure control modes for input and output */
  /* Input: ENU local frame trajectory (determines how plugin performs calculations) */
  input_mode_.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;              // Fixed yaw angle control
  input_mode_.control_mode = as2_msgs::msg::ControlMode::TRAJECTORY;         // Trajectory following mode
  input_mode_.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME; // East-North-Up coordinate frame

  /* Output: ENU local frame speed control (indicates frame of command output) */
  output_mode_.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;              // Fixed yaw angle output
  output_mode_.control_mode = as2_msgs::msg::ControlMode::SPEED;              // Velocity command output
  output_mode_.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME; // East-North-Up coordinate frame

  /* Set control modes in the controller plugin */
  if(!controllers_[drone_id]->setMode(input_mode_, output_mode_))
  {
    RCLCPP_ERROR(this->get_logger(), "Error setting control modes for UAV %d.", drone_id);
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Control modes for UAV %d set successfully.", drone_id);

  /* Process the goal command to generate trajectory */
  processGoal(goal->goal_command);
}
// --------------------------------------------------------------------------------------------



/* Pose callback - tracks current pose and updates trajectory progress */
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
  
  /* Check trajectory progress for linear trajectories */
  if(traj_goal_defined_[drone_id])
  {
    /* Check if drone is within threshold distance of current waypoint */
    if(current_pose_[drone_id].pose.position.x >= desired_traj_[drone_id].setpoints[0].position.x - distance_threshold_ &&
        current_pose_[drone_id].pose.position.x <= desired_traj_[drone_id].setpoints[0].position.x + distance_threshold_ &&
        current_pose_[drone_id].pose.position.y >= desired_traj_[drone_id].setpoints[0].position.y - distance_threshold_ &&
        current_pose_[drone_id].pose.position.y <= desired_traj_[drone_id].setpoints[0].position.y + distance_threshold_ &&
        current_pose_[drone_id].pose.position.z >= desired_traj_[drone_id].setpoints[0].position.z - height_threshold_ &&
        current_pose_[drone_id].pose.position.z <= desired_traj_[drone_id].setpoints[0].position.z + height_threshold_)
    {
      /* Advance to next waypoint if available */
      if (desired_traj_[drone_id].setpoints.size() > 1)
        desired_traj_[drone_id].setpoints.erase(desired_traj_[drone_id].setpoints.begin());

      /* Check if linear trajectory is complete */
      if(desired_traj_[drone_id].setpoints.size() <= 1 && not_circular_[drone_id])
      {
        RCLCPP_INFO(this->get_logger(), "Trajectory for UAV %d completed.", drone_id);

        /* Mark goal as successful but keep position control active */
        result->success = true;
        active_goals_[drone_id]->succeed(result);
      }
      else if (desired_traj_[drone_id].setpoints.size() <= 1 && !not_circular_[drone_id])
      {
        /* Switch to circular trajectory mode */
        traj_goal_defined_[drone_id] = false;  
      }
    }
  }
  else
  {
    /* Check circular trajectory progress */
    if(current_pose_[drone_id].pose.position.x >= circular_traj_[drone_id].setpoints[0].position.x - circular_threshold_ &&
        current_pose_[drone_id].pose.position.x <= circular_traj_[drone_id].setpoints[0].position.x + circular_threshold_ &&
        current_pose_[drone_id].pose.position.y >= circular_traj_[drone_id].setpoints[0].position.y - circular_threshold_ &&
        current_pose_[drone_id].pose.position.y <= circular_traj_[drone_id].setpoints[0].position.y + circular_threshold_ &&
        current_pose_[drone_id].pose.position.z >= circular_traj_[drone_id].setpoints[0].position.z - height_threshold_ &&
        current_pose_[drone_id].pose.position.z <= circular_traj_[drone_id].setpoints[0].position.z + height_threshold_)
    {
      /* Cycle through circular trajectory points (move first to end) */
      as2_msgs::msg::TrajectoryPoint aux = circular_traj_[drone_id].setpoints[0];
      circular_traj_[drone_id].setpoints.erase(circular_traj_[drone_id].setpoints.begin());
      circular_traj_[drone_id].setpoints.push_back(aux);
    }
  }
}
// --------------------------------------------------------------------------------------------



/* Twist callback - tracks current velocity and sends feedback to action client */
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

  /* Send feedback to action client with current state */
  auto feedback = std::make_shared<GoalPoint::Feedback>();
  feedback->current_pose  = current_pose_[drone_id];
  feedback->current_speed = current_twist_[drone_id];
  active_goals_[drone_id]->publish_feedback(feedback);
}
// --------------------------------------------------------------------------------------------



/* Goal execution part 2 - process different types of trajectory commands */
void PidControllerNode::processGoal(const GoalCommand & msg)
{
  auto result = std::make_shared<GoalPoint::Result>();
  GoalCommand goal = msg;
  uint8_t drone_id = goal.drone_id;

  /* Check if goal is still active before proceeding */
  if (active_goals_[drone_id] == nullptr || !active_goals_[drone_id]->is_active()) 
  {
    RCLCPP_WARN(this->get_logger(), "Goal for drone %d is no longer active", drone_id);
    return;
  }

  /* Handle landing command */
  if (goal.land)
  {
    not_circular_[drone_id] = true;

    /* Create landing trajectory to ground level */
    as2_msgs::msg::TrajectorySetpoints land_point;
    land_point.setpoints.resize(1);
    land_point.setpoints[0].position.x = current_pose_[drone_id].pose.position.x;
    land_point.setpoints[0].position.y = current_pose_[drone_id].pose.position.y;
    land_point.setpoints[0].position.z = 0.0; // Ground level
    land_point.header.frame_id = current_pose_[drone_id].header.frame_id;
    desired_traj_[drone_id] = land_point;

    traj_goal_defined_[drone_id] = true;
  }
  /* Handle takeoff command */
  else if (goal.takeoff != 0)
  {
    not_circular_[drone_id] = true;

    /* Create takeoff trajectory to specified height */
    as2_msgs::msg::TrajectorySetpoints takeoff_point;
    takeoff_point.setpoints.resize(1);
    takeoff_point.setpoints[0].position.x = current_pose_[drone_id].pose.position.x;
    takeoff_point.setpoints[0].position.y = current_pose_[drone_id].pose.position.y;
    takeoff_point.setpoints[0].position.z = goal.takeoff; // Desired takeoff height
    takeoff_point.header.frame_id = current_pose_[drone_id].header.frame_id;
    desired_traj_[drone_id] = takeoff_point;

    traj_goal_defined_[drone_id] = true;
  }
  else
  {
    /* Handle different trajectory types based on circular flag */
    switch (goal.circular)
    {
    case 0:   // Linear trajectory only
      not_circular_[drone_id] = true;

      /* Generate linear trajectory from current position to goal point */
      if (goal.trajectory.setpoints.empty() && !goal.point.header.frame_id.empty())
      {
        /* Transform the goal pose to the UAV's reference frame */
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
          RCLCPP_ERROR(this->get_logger(), "Error transforming goal pose: %s", ex.what());
          return;
        }

        /* Define start point with adjusted height */
        geometry_msgs::msg::PoseStamped start = current_pose_[drone_id];
        start.pose.position.z = 5.0; // Standard flight height

        /* Calculate trajectory using Eigen vectors */
        Eigen::Vector3d pi(start.pose.position.x, start.pose.position.y, start.pose.position.z);
        Eigen::Vector3d pf(goal.point.pose.position.x, goal.point.pose.position.y, goal.point.pose.position.z);

        /* Calculate horizontal distance for waypoint density */
        double xy_distance = std::sqrt((pf.x() - pi.x()) * (pf.x() - pi.x()) + 
                                        (pf.y() - pi.y()) * (pf.y() - pi.y()));

        int nums_points_factor = 3;  // Waypoint density factor
        int num_points = static_cast<int>(xy_distance / nums_points_factor);

        /* Generate trajectory with interpolated waypoints */
        as2_msgs::msg::TrajectorySetpoints traj;
        traj.header.frame_id = goal.point.header.frame_id;
        traj.setpoints.resize(num_points);

        /* Calculate displacement vector and direction */
        Eigen::Vector3d delta = pf - pi;
        Eigen::Vector3d dir   = delta.normalized();
        double yaw_const = std::atan2(delta.y(), delta.x()); // Fixed yaw towards goal

        /* Create linear trajectory waypoints */
        for (int i = 0; i < num_points; ++i) 
        {
          double t = static_cast<double>(i) / (num_points - 1); // Interpolation parameter
          Eigen::Vector3d pos = pi + delta * t;

          /* Set interpolated position */
          traj.setpoints[i].position.x = pos.x();
          traj.setpoints[i].position.y = pos.y();
          traj.setpoints[i].position.z = pos.z();

          /* Set velocity profile: zero at endpoints, constant in middle */
          if (i == 0 || i == num_points - 1) 
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

          /* Zero acceleration everywhere */
          traj.setpoints[i].acceleration.x = 0.0;
          traj.setpoints[i].acceleration.y = 0.0;
          traj.setpoints[i].acceleration.z = 0.0;

          traj.setpoints[i].yaw_angle = yaw_const; // Fixed yaw towards goal
        }

        desired_traj_[drone_id] = traj;
        traj_goal_defined_[drone_id] = true;
      }
      /* Use provided custom trajectory */
      else if (!goal.trajectory.setpoints.empty())
      {
        desired_traj_[drone_id] = goal.trajectory;
        traj_goal_defined_[drone_id] = true;
      }
      /* Handle undefined goal error */
      else if (goal.point.header.frame_id.empty())
      {
        RCLCPP_ERROR(this->get_logger(), "Error: Goal is not defined for UAV %d.", drone_id);

        result->success = false;
        active_goals_[drone_id]->abort(result);
        cleanupGoal(drone_id);
        return;
      }

      break;

    case 1:  // Linear trajectory followed by circular trajectory
      not_circular_[drone_id] = false;

      /* Validate circular trajectory radius */
      if (goal.radius <= 0.0) 
      {
        RCLCPP_ERROR(this->get_logger(), "Error: Circular trajectory radius for UAV %d must be greater than zero.", drone_id);

        result->success = false;
        active_goals_[drone_id]->abort(result);
        cleanupGoal(drone_id);
        return;
      }

      /* Calculate number of points for circular trajectory based on circumference */
      int circular_points = static_cast<int>(std::round(4 * M_PI * goal.radius));

      /* Linear trajectory to goal + circular trajectory around goal */
      if (goal.trajectory.setpoints.empty() && !goal.point.header.frame_id.empty())
      {
        /* Transform goal pose to UAV reference frame */
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
          RCLCPP_ERROR(this->get_logger(), "Error transforming goal pose: %s", ex.what());
          return;
        }

        /* Generate circular trajectory around goal point */
        circular_traj_[drone_id].setpoints.resize(circular_points);
        circular_traj_[drone_id].header.frame_id = goal.point.header.frame_id;
        for (int i = 0; i < circular_points; ++i)
        {
          double theta = 2.0 * M_PI * static_cast<double>(i) / circular_points; // Angular position
          /* Position on circle around goal */
          circular_traj_[drone_id].setpoints[i].position.x    = goal.point.pose.position.x + goal.radius * std::cos(theta);
          circular_traj_[drone_id].setpoints[i].position.y    = goal.point.pose.position.y + goal.radius * std::sin(theta);
          circular_traj_[drone_id].setpoints[i].position.z    = goal.point.pose.position.z;
          /* Tangential velocity for circular motion */
          double v = 1.0; // Circular velocity magnitude
          circular_traj_[drone_id].setpoints[i].twist.x       = -v * std::sin(theta);
          circular_traj_[drone_id].setpoints[i].twist.y       = v * std::cos(theta);
          circular_traj_[drone_id].setpoints[i].twist.z       = 0.0;
          /* Zero acceleration */
          circular_traj_[drone_id].setpoints[i].acceleration.x = 0.0;
          circular_traj_[drone_id].setpoints[i].acceleration.y = 0.0;
          circular_traj_[drone_id].setpoints[i].acceleration.z = 0.0;
          circular_traj_[drone_id].setpoints[i].yaw_angle     = theta + M_PI; // Face inward
        }

        /* Generate linear trajectory to circle entry point */
        /* Start point (keep or adjust height if necessary) */
        geometry_msgs::msg::PoseStamped start = current_pose_[drone_id];
        start.pose.position.z = 5.0;

        /* Change to Eigen vectors for calculations */
        Eigen::Vector3d pi(start.pose.position.x, start.pose.position.y, start.pose.position.z);
        Eigen::Vector3d pf(goal.point.pose.position.x,    goal.point.pose.position.y,    goal.point.pose.position.z);

        double xy_distance = std::sqrt((pf.x() - pi.x()) * (pf.x() - pi.x()) + 
                                        (pf.y() - pi.y()) * (pf.y() - pi.y()));

        int nums_points_factor = 3;  
        int num_points = static_cast<int>(xy_distance / nums_points_factor);

        /* Trajectory size definition */
        as2_msgs::msg::TrajectorySetpoints traj;
        traj.header.frame_id = goal.point.header.frame_id;
        traj.setpoints.resize(num_points);

        /* Displacement vector, normalized direction and fixed yaw */
        Eigen::Vector3d delta = pf - pi;
        Eigen::Vector3d dir   = delta.normalized();
        double yaw_const = std::atan2(delta.y(), delta.x());

        /* Create linear trajectory waypoints */
        for (int i = 0; i < num_points; ++i) 
        {
          double t = static_cast<double>(i) / (num_points - 1);
          Eigen::Vector3d pos = pi + delta * t;

          /* Interpolate position */
          traj.setpoints[i].position.x = pos.x();
          traj.setpoints[i].position.y = pos.y();
          traj.setpoints[i].position.z = pos.z();

          /* Velocity: zero at start/end, constant direction elsewhere */
          if (i == 0 || i == num_points - 1) 
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
      /* Circular trajectory around current position only */
      else if (goal.trajectory.setpoints.empty() && goal.point.header.frame_id.empty())
      {
        /* Generate circular trajectory around current position */
        circular_traj_[drone_id].setpoints.resize(circular_points);
        circular_traj_[drone_id].header.frame_id = current_pose_[drone_id].header.frame_id;
        for (int i = 0; i < circular_points; ++i)
        {
          double theta = 2.0 * M_PI * static_cast<double>(i) / circular_points;
          /* Position on circle around current position */
          circular_traj_[drone_id].setpoints[i].position.x    = current_pose_[drone_id].pose.position.x + goal.radius * std::cos(theta);
          circular_traj_[drone_id].setpoints[i].position.y    = current_pose_[drone_id].pose.position.y + goal.radius * std::sin(theta);
          circular_traj_[drone_id].setpoints[i].position.z    = 5.0; // Standard flight height
          /* Tangential velocity for circular motion */
          double v = 1.0;
          circular_traj_[drone_id].setpoints[i].twist.x       = -v * std::sin(theta);
          circular_traj_[drone_id].setpoints[i].twist.y       = v * std::cos(theta);
          circular_traj_[drone_id].setpoints[i].twist.z       = 0.0;
          /* Zero acceleration */
          circular_traj_[drone_id].setpoints[i].acceleration.x = 0.0;
          circular_traj_[drone_id].setpoints[i].acceleration.y = 0.0;
          circular_traj_[drone_id].setpoints[i].acceleration.z = 0.0;
          circular_traj_[drone_id].setpoints[i].yaw_angle     = theta + M_PI; // Face inward
        }
      }
      /* Custom trajectory + circular trajectory around end point */
      else if (!goal.trajectory.setpoints.empty())
      {
        desired_traj_[drone_id] = goal.trajectory;
        traj_goal_defined_[drone_id] = true;

        /* Generate circular trajectory around trajectory end point */
        circular_traj_[drone_id].setpoints.resize(circular_points);
        circular_traj_[drone_id].header.frame_id = goal.trajectory.header.frame_id;
        for (int i = 0; i < circular_points; ++i)
        {
          double theta = 2.0 * M_PI * static_cast<double>(i) / circular_points;
          /* Position on circle around trajectory end */
          circular_traj_[drone_id].setpoints[i].position.x    = goal.trajectory.setpoints.back().position.x + goal.radius * std::cos(theta);
          circular_traj_[drone_id].setpoints[i].position.y    = goal.trajectory.setpoints.back().position.y + goal.radius * std::sin(theta);
          circular_traj_[drone_id].setpoints[i].position.z    = goal.trajectory.setpoints.back().position.z;
          /* Tangential velocity for circular motion */
          double v = 1.0;
          circular_traj_[drone_id].setpoints[i].twist.x       = -v * std::sin(theta);
          circular_traj_[drone_id].setpoints[i].twist.y       = v * std::cos(theta);
          circular_traj_[drone_id].setpoints[i].twist.z       = 0.0;
          /* Zero acceleration */
          circular_traj_[drone_id].setpoints[i].acceleration.x = 0.0;
          circular_traj_[drone_id].setpoints[i].acceleration.y = 0.0;
          circular_traj_[drone_id].setpoints[i].acceleration.z = 0.0;
          circular_traj_[drone_id].setpoints[i].yaw_angle     = theta + M_PI; // Face inward
        }
      }

      break;
    }
  }
}
// --------------------------------------------------------------------------------------------



/* Timer callback - main control loop that runs periodically for each drone */
void PidControllerNode::timerCallback(int drone_id)
{
  std::lock_guard<std::mutex> lock(drone_mutexes_[drone_id]);

  /* Check if there's a trajectory to control */
  if(!traj_goal_defined_[drone_id] && circular_traj_[drone_id].setpoints.empty())
  {
    /* Clean up unused controller instances when no trajectory is active */
    if (controllers_[drone_id] != nullptr && active_goals_[drone_id] == nullptr) 
    {
      RCLCPP_INFO(this->get_logger(), "Cleaning up unused controller for drone %d", drone_id);
      cleanupGoal(drone_id);
    }
    return; // No trajectory to control
  }

  if (!controllers_[drone_id]) 
  {
    RCLCPP_WARN(this->get_logger(), "No controller for UAV %d", drone_id);
    return;
  }

  /* Update controller reference based on trajectory type */
  if (traj_goal_defined_[drone_id]) 
  {
    controllers_[drone_id]->updateReference(desired_traj_[drone_id]); // Linear trajectory
  } 
  else 
  {
    controllers_[drone_id]->updateReference(circular_traj_[drone_id]); // Circular trajectory
  }

  /* Update controller with current drone state */
  controllers_[drone_id]->updateState(current_pose_[drone_id], current_twist_[drone_id]);

  /* Variables for controller output */
  geometry_msgs::msg::PoseStamped unused_pose;
  geometry_msgs::msg::TwistStamped command_twist;
  as2_msgs::msg::Thrust unused_thrust;

  /* Compute control output from PID controller */
  if(!controllers_[drone_id]->computeOutput(0.100, unused_pose, command_twist, unused_thrust)) // 100ms control period
  {
    RCLCPP_ERROR(this->get_logger(), "Error computing output for plugin %d.", drone_id);
    rclcpp::shutdown();
    return;
  }

  /* Transform the output TwistStamped to the drone's base_link frame */
  geometry_msgs::msg::TransformStamped transform;
  try
  {
    /* Get transform from command frame to drone base_link */
    transform = tf_buffer_.lookupTransform("drone" + std::to_string(drone_id) + "/base_link", command_twist.header.frame_id,
                                            rclcpp::Time(0), rclcpp::Duration(2s));

    /* Transform linear velocity vector */
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

    /* Transform angular velocity vector */
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

    /* Update command with transformed velocities */
    command_twist.twist.linear.x = transformed_linear_velocity.x();
    command_twist.twist.linear.y = transformed_linear_velocity.y();
    command_twist.twist.linear.z = transformed_linear_velocity.z();

    command_twist.twist.angular.x = transformed_angular_velocity.x();
    command_twist.twist.angular.y = transformed_angular_velocity.y();
    command_twist.twist.angular.z = transformed_angular_velocity.z();

    command_twist.header.frame_id = "drone" + std::to_string(drone_id) + "/base_link";
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Error transforming plugin output: %s", ex.what());
    return;
  }

  /* Publish velocity command to drone actuators */
  twist_publishers_[drone_id]->publish(command_twist);
}
// --------------------------------------------------------------------------------------------



/* Main function - entry point for the motion controller node */
int main(int argc, char **argv)
{
  // Disable Fast RTPS shared memory transport for better compatibility
  ::setenv("RMW_FASTRTPS_SHARED_MEMORY_ENABLED", "0", 1);
  
  rclcpp::init(argc, argv); // Initialize ROS 2
  auto node = std::make_shared<PidControllerNode>(); // Create controller node

  node->initTfListener(); // Initialize transform listener
  
  rclcpp::spin(node); // Start processing callbacks
  rclcpp::shutdown(); // Clean shutdown
  return 0;
}
// --------------------------------------------------------------------------------------------
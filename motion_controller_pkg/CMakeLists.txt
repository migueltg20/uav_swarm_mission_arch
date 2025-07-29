cmake_minimum_required(VERSION 3.8)
project(motion_controller_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# ------------------------------------------------------------------------------------
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(as2_motion_controller REQUIRED)
find_package(as2_msgs REQUIRED)
find_package(as2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(motion_controller_pkg REQUIRED)
# ------------------------------------------------------------------------------------



# ------------------------------------------------------------------------------------
# Generate the action and message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GoalCommand.msg"
  "action/GoalPoint.action"
  DEPENDENCIES
    geometry_msgs
    as2_msgs
)
# ------------------------------------------------------------------------------------



# ------------------------------------------------------------------------------------
# Action server node (low level controller node)
add_executable(motion_controller
  src/motion_controller.cpp
)

# ROS2 libraries/packages
ament_target_dependencies(motion_controller
  rclcpp
  as2_motion_controller
  as2_msgs
  as2_core
  geometry_msgs
  pluginlib
  motion_controller_pkg
)

# Non-ROS2 libraries/packages
target_link_libraries(motion_controller
  Threads::Threads
  Eigen3::Eigen
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)
# ------------------------------------------------------------------------------------



# ------------------------------------------------------------------------------------
# Action client node
add_executable(motion_controller_client
  src/motion_controller_client.cpp
)

# ROS2 libraries/packages
ament_target_dependencies(motion_controller_client
  rclcpp
  rclcpp_action
  as2_core
  motion_controller_pkg  
)
# ------------------------------------------------------------------------------------



# ------------------------------------------------------------------------------------
# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Install targets
install(TARGETS
  motion_controller
  motion_controller_client
  DESTINATION lib/${PROJECT_NAME}
)

# Install configuration files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install rviz files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)
# ------------------------------------------------------------------------------------



# ------------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
# ------------------------------------------------------------------------------------
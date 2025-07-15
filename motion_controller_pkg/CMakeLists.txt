cmake_minimum_required(VERSION 3.8)
project(motion_controller_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
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
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# add nodes
add_executable(motion_controller
  src/motion_controller.cpp
)

# Link ROS libraries using ament_target_dependencies (this auto-configures include dirs and flags)
ament_target_dependencies(motion_controller
  rclcpp
  as2_motion_controller
  as2_msgs
  as2_core
  geometry_msgs
  pluginlib
)

# Explicitly link non-ROS libraries (here: TF2)
target_link_libraries(motion_controller
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)

# add_executable(motion_controller_client
#   src/motion_controller_client.cpp
# )

# ament_target_dependencies(motion_controller_client
#   rclcpp
#   rclcpp_action
#   as2_msgs
#   as2_core
#   geometry_msgs
#   pluginlib
# )

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoalPoint.action"
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${Eigen3_INCLUDE_DIRS}
)

install(TARGETS motion_controller
  DESTINATION lib/${PROJECT_NAME})

# Install configuration files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

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
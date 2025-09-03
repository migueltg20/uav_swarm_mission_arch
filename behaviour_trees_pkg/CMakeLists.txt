cmake_minimum_required(VERSION 3.8)
project(behaviour_trees_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# ------------------------------------------------------------------------------------
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(as2_msgs REQUIRED)
find_package(as2_core REQUIRED)
find_package(as2_behavior REQUIRED)
# find_package(as2_behavior_tree REQUIRED)
# find_package(behaviortree_cpp_v3 REQUIRED)
# find_package(nav2_behavior_tree REQUIRED)
find_package(motion_controller_pkg REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
# ------------------------------------------------------------------------------------



# ------------------------------------------------------------------------------------
ament_export_dependencies(rosidl_default_runtime)
# -------------------------------------------------------------------------------------



# ------------------------------------------------------------------------------------
# ament_python_install_package(${PROJECT_NAME})
# ------------------------------------------------------------------------------------



# ------------------------------------------------------------------------------------
# Generate the action and message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/TakeOffBh.action"
  "action/LandBh.action"
  "action/FollowTrajBh.action"
  DEPENDENCIES
    geometry_msgs
    as2_msgs
)
# ------------------------------------------------------------------------------------



# ------------------------------------------------------------------------------------
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# ------------------------------------------------------------------------------------



# ------------------------------------------------------------------------------------
# Action server node (low level controller node)
add_executable(takeoff_server
  src/takeoff_bh_server.cpp
)

# ROS2 libraries/packages
ament_target_dependencies(takeoff_server
  rclcpp
  rclcpp_action
  as2_msgs
  as2_behavior
  geometry_msgs
  motion_controller_pkg
  rosidl_default_runtime
)

# Non-ROS2 libraries/packages
target_link_libraries(takeoff_server
  ${cpp_typesupport_target}
)
# ------------------------------------------------------------------------------------



# ------------------------------------------------------------------------------------
# Action server node (low level controller node)
add_executable(land_server
  src/land_bh_server.cpp
)

# ROS2 libraries/packages
ament_target_dependencies(land_server
  rclcpp
  rclcpp_action
  as2_msgs
  as2_behavior
  geometry_msgs
  motion_controller_pkg
  rosidl_default_runtime
)

# Non-ROS2 libraries/packages
target_link_libraries(land_server
  ${cpp_typesupport_target}
)
# ------------------------------------------------------------------------------------



# ------------------------------------------------------------------------------------
# Action server node (low level controller node)
add_executable(follow_traj_server
  src/followtraj_bh_server.cpp
)

# ROS2 libraries/packages
ament_target_dependencies(follow_traj_server
  rclcpp
  rclcpp_action
  as2_msgs
  as2_behavior
  geometry_msgs
  motion_controller_pkg
  rosidl_default_runtime
)

# Non-ROS2 libraries/packages
target_link_libraries(follow_traj_server
  ${cpp_typesupport_target}
)
# ------------------------------------------------------------------------------------



# ------------------------------------------------------------------------------------
# Action server node (low level controller node)
add_executable(arming_server
  src/arming_bh_server.cpp
)

# ROS2 libraries/packages
ament_target_dependencies(arming_server
  rclcpp
  rclcpp_action
  std_srvs
  as2_msgs
  as2_behavior
  rosidl_default_runtime
)

# Non-ROS2 libraries/packages
target_link_libraries(arming_server
  ${cpp_typesupport_target}
)
# ------------------------------------------------------------------------------------



# ------------------------------------------------------------------------------------
# Action server node (low level controller node)
add_executable(offboard_server
  src/offboard_bh_server.cpp
)

# ROS2 libraries/packages
ament_target_dependencies(offboard_server
  rclcpp
  rclcpp_action
  std_srvs
  as2_msgs
  as2_behavior
  rosidl_default_runtime
)

# Non-ROS2 libraries/packages
target_link_libraries(offboard_server
  ${cpp_typesupport_target}
)
# ------------------------------------------------------------------------------------



# # ------------------------------------------------------------------------------------
# # Action manager node
# add_executable(bt_manager
#   src/bt_manager.cpp
# )

# ament_target_dependencies(bt_manager
#   rclcpp
#   behaviortree_cpp_v3
#   nav2_behavior_tree
#   as2_behavior_tree
#   rosidl_default_runtime
# )

# # Link the generated typesupport library
# target_link_libraries(bt_manager
#   ${cpp_typesupport_target}
# )
# # ------------------------------------------------------------------------------------



# ------------------------------------------------------------------------------------
# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install targets
install(TARGETS
  takeoff_server
  land_server
  follow_traj_server
  arming_server
  offboard_server
  # bt_manager
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

# Install Python scripts
install(PROGRAMS
  scripts/takeoff_bh_client.py
  scripts/land_bh_client.py
  scripts/followtraj_bh_client.py
  scripts/arming_bh_client.py
  scripts/offboard_bh_client.py
  DESTINATION lib/${PROJECT_NAME}
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
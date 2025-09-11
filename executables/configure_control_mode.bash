#!/bin/bash

cd ~/aerostack2_ws

source install/setup.bash

ros2 service call /drone0/set_platform_control_mode as2_msgs/srv/SetControlMode "control_mode:
  yaw_mode: 2
  control_mode: 3
  reference_frame: 1"
  
ros2 service call /drone0/controller/set_control_mode as2_msgs/srv/SetControlMode "control_mode:
  yaw_mode: 2
  control_mode: 3
  reference_frame: 1"
  
ros2 service call /drone1/set_platform_control_mode as2_msgs/srv/SetControlMode "control_mode:
  yaw_mode: 2
  control_mode: 3
  reference_frame: 1"
  
ros2 service call /drone1/controller/set_control_mode as2_msgs/srv/SetControlMode "control_mode:
  yaw_mode: 2
  control_mode: 3
  reference_frame: 1"
  
ros2 service call /drone2/set_platform_control_mode as2_msgs/srv/SetControlMode "control_mode:
  yaw_mode: 2
  control_mode: 3
  reference_frame: 1"
  
ros2 service call /drone2/controller/set_control_mode as2_msgs/srv/SetControlMode "control_mode:
  yaw_mode: 2
  control_mode: 3
  reference_frame: 1"
  
ros2 service call /drone3/set_platform_control_mode as2_msgs/srv/SetControlMode "control_mode:
  yaw_mode: 2
  control_mode: 3
  reference_frame: 1"
  
ros2 service call /drone3/controller/set_control_mode as2_msgs/srv/SetControlMode "control_mode:
  yaw_mode: 2
  control_mode: 3
  reference_frame: 1"

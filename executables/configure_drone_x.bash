#!/bin/bash

cd ~/aerostack2_ws

source install/setup.bash

ros2 service call /drone0/set_arming_state std_srvs/srv/SetBool "{data: true}"

ros2 service call /drone0/set_offboard_mode std_srvs/srv/SetBool "{data: true}"

ros2 service call /drone0/set_platform_control_mode as2_msgs/srv/SetControlMode "control_mode:
  yaw_mode: 2
  control_mode: 3
  reference_frame: 1"
  
ros2 service call /drone0/controller/set_control_mode as2_msgs/srv/SetControlMode "control_mode:
  yaw_mode: 2
  control_mode: 3
  reference_frame: 1"

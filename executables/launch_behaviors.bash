#!/bin/bash

cd ~/aerostack2_ws

source install/setup.bash

ros2 launch motion_controller_pkg motion_controller_launch.py

ros2 launch behaviour_trees_pkg mission_02.py				# or mission_13.py

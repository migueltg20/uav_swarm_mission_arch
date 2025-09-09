#!/bin/bash

cd ~/aerostack2_ws

source install/setup.bash

ros2 launch motion_controller_pkg motion_controller_launch.py

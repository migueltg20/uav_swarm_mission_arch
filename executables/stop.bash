#!/bin/bash

cd ~/simulaciones/TB2_Panel_Inspection_Simulation
./stop.bash

pkill -f as2
pkill -f ros_gz_bridge
pkill -f ros2
pkill -f tf2_ros/static_transform_publisher
pkill -f "python3 /opt/ros/humble/bin/ros2 launch"

ros2 daemon stop

rm -rf /tmp/gazebo*
rm -rf /tmp/tmux*
rm -rf ~/.ros/log
rm -rf ~/.ros/tmp

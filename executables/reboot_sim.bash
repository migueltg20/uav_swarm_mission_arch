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

sleep 2

ros2 daemon start

source /opt/ros/humble/setup.bash

cd ~/simulaciones/TB2_Panel_Inspection_Simulation
./launch_as2.bash -n 4 -p 5 -r 4

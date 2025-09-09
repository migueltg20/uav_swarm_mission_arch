#!/bin/bash

cd ~/aerostack2_ws

source install/setup.bash

ros2 topic pub /goal motion_controller_pkg/msg/GoalCommand '{
  drone_id: 2,
  takeoff: 5.0,
  land: false,
  point: {
    header: {
      stamp: {sec: 0, nanosec: 0},
      frame_id: "earth"
    },
    pose: {
      position: {x: 0.0, y: 0.0, z: 5.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  },
  trajectory: {
    header: {
      stamp: {sec: 0, nanosec: 0},
      frame_id: ""
    },
    setpoints: []
  },
  circular: false,
  radius: 3.0,
  cancel: false
}' --once

#!/bin/bash

cd ~/aerostack2_ws

source install/setup.bash

ros2 topic pub /goal motion_controller_pkg/msg/GoalCommand '{
  drone_id: 0,
  takeoff: 0,
  land: false,
  point: {
    header: {
      stamp: {sec: 0, nanosec: 0},
      frame_id: "earth"
    },
    pose: {
      position: {x: 20.0, y: 3.5, z: 5.0},
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
  radius: 3.0,
  cancel: false
}' --once

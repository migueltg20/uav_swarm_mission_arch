#!/bin/bash

cd ~/aerostack2_ws

source install/setup.bash

ros2 topic pub /goal motion_controller_pkg/msg/GoalCommand '{
  drone_id: 2,
  takeoff: 0,
  land: false,
  point: {
    header: {
      stamp: {sec: 0, nanosec: 0},
      frame_id: ""
    },
    pose: {
      position: {x: 0.0, y: 10.0, z: 5.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  },
  trajectory: {
    header: {
      stamp: {sec: 0, nanosec: 0},
      frame_id: "earth"
    },
    setpoints: [
      {
        position: {x: 20.0, y: 3.5, z: 5.0},
        twist: {x: 0.0, y: 0.0, z: 0.0},
        acceleration: {x: 0.0, y: 0.0, z: 0.0},
        yaw_angle: 0.0
      },
      {
        position: {x: 23.5, y: 3.5, z: 5.0},
        twist: {x: 1.0, y: 2.0, z: 0.0},
        acceleration: {x: 0.0, y: 0.0, z: 0.0},
        yaw_angle: 0.0
      },
      {
        position: {x: 27.0, y: 3.5, z: 5.0},
        twist: {x: 0.0, y: 0.0, z: 0.0},
        acceleration: {x: 0.0, y: 0.0, z: 0.0},
        yaw_angle: 0.0
      },
      {
        position: {x: 30.5, y: 3.5, z: 5.0},
        twist: {x: 0.0, y: 0.0, z: 0.0},
        acceleration: {x: 0.0, y: 0.0, z: 0.0},
        yaw_angle: 0.0
      }
    ]
  },
  circular: false,
  radius: 3.0,
  cancel: false
}' --once

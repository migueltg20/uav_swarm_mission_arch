# Mission Execution Architecture for Multi-UAV Teams

## Overview
This repository contains two packages for controlling a UAV swarm, mainly for inspection tasks. Both packages were created based on a previous architecture that used ROS (Robot Operating System) and the Behaviour Trees technique, but have been updated to use ROS2 and Aerostack2, an aerial platform framework designed to ease the creation of systems and architectures like this.

## Packages
As mentioned, this repo contains two ROS2 packages for UAV mission execution: one for simple motion control, and one for Behaviour Trees application (the latter package uses the motion controller one as a base).

### motion_controller_pkg

### behaviour_trees_pkg
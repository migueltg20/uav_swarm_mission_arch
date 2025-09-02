#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler
from behaviour_trees_pkg.action import FollowTrajBh
from as2_msgs.msg import TrajectorySetpoints, TrajectoryPoint
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header



class FollowTrajClient(BehaviorHandler):
    """Follow Trajectory Behavior Client"""

    def __init__(self, node: Node) -> None:
        self.__node = node

        try:
            super().__init__(node, FollowTrajBh, 'follow_traj')
        except self.BehaviorNotAvailable as err:
            self.__node.get_logger().warn(str(err))

    def start(self, goal_msg: FollowTrajBh.Goal, wait_result: bool = True) -> bool:
        try:
            return super().start(goal_msg, wait_result)
        except self.GoalRejected as err:
            self.__node.get_logger().warn(str(err))
        return False

    def modify(self, goal_msg: FollowTrajBh.Goal) -> bool:
        return super().modify(goal_msg)
    

# def main():
#     rclpy.init()
    
#     node = Node('follow_traj_test')
#     follow_traj_client = FollowTrajClient(node)

#     goal = FollowTrajBh.Goal()
#     goal.drone_id = 0
#     goal.radius = 1.0

#     # Create trajectory
#     trajectory = TrajectorySetpoints()
    
#     # Set header
#     trajectory.header = Header()
#     trajectory.header.stamp = node.get_clock().now().to_msg()
#     trajectory.header.frame_id = "earth"
    
#     # Create setpoints
#     setpoints = []
    
#     # Setpoint 1
#     point1 = TrajectoryPoint()
#     point1.position = Vector3(x=20.0, y=3.5, z=5.0)
#     point1.twist = Vector3(x=0.0, y=0.0, z=0.0)
#     point1.acceleration = Vector3(x=0.0, y=0.0, z=0.0)
#     point1.yaw_angle = 0.0
#     setpoints.append(point1)
    
#     # Setpoint 2
#     point2 = TrajectoryPoint()
#     point2.position = Vector3(x=23.5, y=3.5, z=5.0)
#     point2.twist = Vector3(x=1.0, y=2.0, z=0.0)
#     point2.acceleration = Vector3(x=0.0, y=0.0, z=0.0)
#     point2.yaw_angle = 0.0
#     setpoints.append(point2)
    
#     # Setpoint 3
#     point3 = TrajectoryPoint()
#     point3.position = Vector3(x=27.0, y=3.5, z=5.0)
#     point3.twist = Vector3(x=0.0, y=0.0, z=0.0)
#     point3.acceleration = Vector3(x=0.0, y=0.0, z=0.0)
#     point3.yaw_angle = 0.0
#     setpoints.append(point3)
    
#     # Setpoint 4
#     point4 = TrajectoryPoint()
#     point4.position = Vector3(x=30.5, y=3.5, z=5.0)
#     point4.twist = Vector3(x=0.0, y=0.0, z=0.0)
#     point4.acceleration = Vector3(x=0.0, y=0.0, z=0.0)
#     point4.yaw_angle = 0.0
#     setpoints.append(point4)
    
#     # Assign setpoints to trajectory
#     trajectory.setpoints = setpoints
#     goal.trajectory = trajectory
    
#     success = follow_traj_client.start(goal, wait_result=True)
    
#     if success:
#         node.get_logger().info("Trajectory following successful!")
#     else:
#         node.get_logger().error("Trajectory following failed!")

#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
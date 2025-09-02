#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler
from behaviour_trees_pkg.action import LandBh



class LandClient(BehaviorHandler):
    """Land Behavior Client"""

    def __init__(self, node: Node) -> None:
        self.__node = node

        try:
            super().__init__(node, LandBh, 'land')
        except self.BehaviorNotAvailable as err:
            self.__node.get_logger().warn(str(err))

    def start(self, goal_msg: LandBh.Goal, wait_result: bool = True) -> bool:
        try:
            return super().start(goal_msg, wait_result)
        except self.GoalRejected as err:
            self.__node.get_logger().warn(str(err))
        return False

    def modify(self, goal_msg: LandBh.Goal) -> bool:
        return super().modify(goal_msg)
    

# def main():
#     rclpy.init()

#     node = Node('land_test')
#     land_client = LandClient(node)

#     goal = LandBh.Goal()
#     goal.drone_id = 0
#     goal.land = True
#     success = land_client.start(goal, wait_result=True)
    
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
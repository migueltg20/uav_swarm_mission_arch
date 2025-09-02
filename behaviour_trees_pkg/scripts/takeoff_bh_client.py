#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler
from behaviour_trees_pkg.action import TakeOffBh



class TakeOffClient(BehaviorHandler):
    """Takeoff Behavior Client"""

    def __init__(self, node: Node) -> None:
        self.__node = node

        try:
            super().__init__(node, TakeOffBh, 'take_off')
        except self.BehaviorNotAvailable as err:
            self.__node.get_logger().warn(str(err))

    def start(self, goal_msg: TakeOffBh.Goal, wait_result: bool = True) -> bool:
        try:
            return super().start(goal_msg, wait_result)
        except self.GoalRejected as err:
            self.__node.get_logger().warn(str(err))
        return False

    def modify(self, goal_msg: TakeOffBh.Goal) -> bool:
        return super().modify(goal_msg)



def main():
    rclpy.init()
    
    node = Node('takeoff_test')
    takeoff_client = TakeOffClient(node)
    
    goal = TakeOffBh.Goal()
    goal.drone_id = 0
    goal.takeoff_height = 5.0
    success = takeoff_client.start(goal, wait_result=True)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
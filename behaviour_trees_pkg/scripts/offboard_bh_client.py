#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler
from as2_msgs.action import SetOffboardMode



class OffboardClient(BehaviorHandler):
    """Offboard Behavior Client"""

    def __init__(self, node: Node) -> None:
        self.__node = node

        try:
            super().__init__(node, SetOffboardMode, 'offboard')
        except self.BehaviorNotAvailable as err:
            self.__node.get_logger().warn(str(err))

    def start(self, goal_msg: SetOffboardMode.Goal, wait_result: bool = True) -> bool:
        try:
            return super().start(goal_msg, wait_result)
        except self.GoalRejected as err:
            self.__node.get_logger().warn(str(err))
        return False

    def modify(self, goal_msg: SetOffboardMode.Goal) -> bool:
        return super().modify(goal_msg)
    

def main():
    rclpy.init()

    node = Node('offboard_test')
    offboard_client = OffboardClient(node)

    goal = SetOffboardMode.Goal()
    goal.request = True
    success = offboard_client.start(goal, wait_result=True)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
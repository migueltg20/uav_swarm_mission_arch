#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler
from as2_msgs.action import SetArmingState



class ArmingClient(BehaviorHandler):
    """Arming Behavior Client"""

    def __init__(self, node: Node) -> None:
        self.__node = node

        try:
            super().__init__(node, SetArmingState, 'arming')
        except self.BehaviorNotAvailable as err:
            self.__node.get_logger().warn(str(err))

    def start(self, goal_msg: SetArmingState.Goal, wait_result: bool = True) -> bool:
        try:
            return super().start(goal_msg, wait_result)
        except self.GoalRejected as err:
            self.__node.get_logger().warn(str(err))
        return False

    def modify(self, goal_msg: SetArmingState.Goal) -> bool:
        return super().modify(goal_msg)
    

def main():
    rclpy.init()

    node = Node('arming_test')
    arming_client = ArmingClient(node)

    goal = SetArmingState.Goal()
    goal.request = True
    success = arming_client.start(goal, wait_result=True)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
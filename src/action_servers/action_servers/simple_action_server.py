import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from custom_action_interfaces.action import Simpleaction

class SimpleActionServer(Node):

    def __init__(self):
        super().__init__('simple_action_server')
        self._action_server = ActionServer(
            self,
            Simpleaction,
            'simple_action',
            self.execute_callback)

    # From sample Fibonacci action server
    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Request received: {goal_handle.request.simple_request}. Executing...')

        feedback_msg = Simpleaction.Feedback()
        feedback_msg.simple_feedback = [0, 1]

        for i in range(1, goal_handle.request.simple_request):
            feedback_msg.simple_feedback.append(
                feedback_msg.simple_feedback[i] + feedback_msg.simple_feedback[i-1])
            self.get_logger().info(f'Feedback: {feedback_msg.simple_feedback}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = Simpleaction.Result()
        result.simple_result = feedback_msg.simple_feedback
        print(f'result.simple_result: {result.simple_result}')
        return result


def main(args=None):
    rclpy.init(args=args)

    simple_action_server = SimpleActionServer()

    try:
        rclpy.spin(simple_action_server)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
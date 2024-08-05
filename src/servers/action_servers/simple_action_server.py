import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from custom_interfaces.action import Simpleactiontype


class SimpleActionServer(Node):

    def __init__(self):
        super().__init__('simple_action_node')
        self._action_server = ActionServer(
            self,
            Simpleactiontype,
            'simple_action_name',
            self.execute_callback)

    # From sample Fibonacci action server
    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Request received: {goal_handle.request.simple_request}. Executing...')

        feedback_msg = Simpleactiontype.Feedback()
        feedback_msg.simple_feedback = [0, 1]

        for i in range(1, goal_handle.request.simple_request):
            feedback_msg.simple_feedback.append(
                feedback_msg.simple_feedback[i] + feedback_msg.simple_feedback[i-1])
            self.get_logger().info(f'Feedback: {feedback_msg.simple_feedback}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = Simpleactiontype.Result()
        result.simple_result = feedback_msg.simple_feedback
        self.get_logger().info(f'Goal ({goal_handle.request.simple_request}) reached, result: {result.simple_result}')
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
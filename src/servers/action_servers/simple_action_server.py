import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node

from custom_interfaces.action import Simpleactiontype


class SimpleActionServer(Node):

    def __init__(self):
        super().__init__('simple_action_node')
        self._action_server = ActionServer(
            self,
            action_type=Simpleactiontype,
            action_name='simple_action_name',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback
            )

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

    def cancel_callback(self, goal_handle):
        """Custom callback to allow or reject goal cancellation. (Default ROS 2 action servers reject all cancellations.)"""

        self.get_logger().info(f"Cancel request received.")

        # Example logic: Allow cancellation only if the order is above a threshold
        if goal_handle.request.order >=10:
            self.get_logger().info(f"Cancel request accepted.")
            return CancelResponse.ACCEPT
        
        self.get_logger().info(f"Cancel request rejected as order {goal_handle.request.order} was less than 10.")
        return CancelResponse.REJECT


def main(args=None):
    rclpy.init(args=args)

    simple_action_server = SimpleActionServer()

    try:
        rclpy.spin(simple_action_server)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
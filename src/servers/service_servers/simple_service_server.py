import time

import rclpy
from rclpy.node import Node

from custom_interfaces.srv import Simpleservice


class SimpleService(Node):

    def __init__(self):
        super().__init__('simple_service_server')
        self.srv = self.create_service(
            Simpleservice,
            'simple_service',
            self.service_callback)

    # From sample add three ints service server
    def service_callback(self, request, response):
        self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (request.a, request.b, request.c))

        response.sum = request.a + request.b + request.c

        return response


def main(args=None):
    rclpy.init(args=args)

    simple_service = SimpleService()

    try:
        rclpy.spin(simple_service)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
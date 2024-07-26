#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import argparse

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(String, 'ros_sm/event', 10)

    def publish_command(self, command, goal=None):
        msg = String()
        if goal is not None:
            msg.data = f"{command} {goal}"
        else:
            msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing to ros_sm/event topic: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)

    command_publisher = CommandPublisher()

    try:
        while rclpy.ok():
            command = input("Enter command to publish:")
            if command.lower() in ['exit', 'quit']:
                break
            goal = input("Enter goal value (or leave blank):")
            goal = goal if goal else None
            command_publisher.publish_command(command, goal)
    except KeyboardInterrupt:
        pass
    finally:
        command_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

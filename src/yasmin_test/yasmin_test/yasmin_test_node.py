#!/usr/bin/env python3


import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub


# define state Foo
class FooState(State):
    def __init__(self, node) -> None:
        super().__init__(["outcome1", "outcome2"])
        self.node = node
        self.counter = 0

    def execute(self, blackboard: Blackboard) -> str:
        print("Executing state FOO")
        time.sleep(3)

        if self.counter < 3:
            self.counter += 1
            blackboard["foo_str"] = f"Counter: {self.counter}"
            return "outcome1"
        else:
            return "outcome2"

    # TODO: externally trigger the state transition to Bar

# define state Bar
class BarState(State):
    def __init__(self, node) -> None:
        super().__init__(outcomes=["outcome3"])
        self.node = node

    def execute(self, blackboard: Blackboard) -> str:
        print("Executing state BAR")
        time.sleep(3)

        print(blackboard["foo_str"])
        return "outcome3"


# define StateMachineNode
class StateMachineNode(Node):
    def __init__(self):
        super().__init__('ros_state_machine')
        self.event_sub = self.create_subscription(
            String,
            'ros_sm/event',
            self.event_callback,
            10
        )
        self.current_event = {'event':None, 'value':None}

    def event_callback(self, msg):
        try:
            parts = msg.data.split()
            self.current_event['event'] = parts[0]
            self.current_event['value'] = float(parts[1]) if len(parts) > 1 else None
        except (IndexError, ValueError):
            self.get_logger().error(f'Invalid event message format: {msg.data}')
            self.current_event = {'event':None, 'value':None}

    def wait_for_event(self):
        print('waiting for event')
        while rclpy.ok() and self.current_event == {'event':None, 'value':None}:
            rclpy.spin_once(self)
        event = self.current_event
        self.current_event = {'event':None, 'value':None}
        return event


# main
def main():

    print("yasmin_test_node running")

    # init ROS 2
    rclpy.init()

    node = StateMachineNode()

    # create a FSM
    sm = StateMachine(outcomes=["outcome4"])

    # add states
    sm.add_state(
        "FOO",
        FooState(node),
        transitions={
            "outcome1": "BAR",
            "outcome2": "outcome4"
        }
    )
    sm.add_state(
        "BAR",
        BarState(node),
        transitions={
            "outcome3": "FOO"
        }
    )

    # pub FSM info
    YasminViewerPub("yasmin_demo", sm)

    try:
        # execute FSM
        outcome = sm()
        print(outcome)
    except Exception as e:
        print(e)

    rclpy.spin(node)

    # shutdown ROS 2
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

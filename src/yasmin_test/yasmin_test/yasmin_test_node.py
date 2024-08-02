#!/usr/bin/env python3

# Imports for basic yasmin demo
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub

# Imports for action client demo
from custom_action_interfaces.action import Simpleaction
from yasmin import CbState
from yasmin_ros import ActionState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL


# define state Foo
class FooState(State):
    def __init__(self, node) -> None:
        super().__init__(["outcome1", "outcome2", "do_Action"])
        self.node = node
        self.counter = 0

    def execute(self, blackboard: Blackboard) -> str:
        self.node.get_logger().info("Executing state FOO")
        event = self.node.wait_for_event()

        time.sleep(3)

        if event['event'] == 'outcome1':
            blackboard["event"] = f"outcome1"
            return "outcome1"
        elif event['event'] == "do_action":
            blackboard["event"] = f"do_action"
            blackboard["action_goal"] = event["value"]
            return "do_Action"
        elif event['event'] == 'outcome2':
            # blackboard["event"] = f"outcome2"
            return "outcome2"


# define state Bar
class BarState(State):
    def __init__(self, node) -> None:
        super().__init__(outcomes=["outcome3"])
        self.node = node

    def execute(self, blackboard: Blackboard) -> str:
        self.node.get_logger().info("Executing state BAR")
        event = self.node.wait_for_event()

        time.sleep(3)

        if event['event'] == 'outcome3':
            blackboard["event"] = f"outcome3"
            return "outcome3"
        elif event['event'] == 'outcome2':
            # blackboard["event"] = f"outcome2"
            return "outcome2"


# define state SimpleAction
class SimpleActionState(ActionState):
    def __init__(self, node) -> None:
        super().__init__(
            action_type=Simpleaction,  # action type
            action_name="/simple_action",  # action name
            create_goal_handler=self.create_goal_handler,  # cb to create the goal
            outcomes=None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            result_handler=self.response_handler,  # cb to process the response
            feedback_handler=self.print_feedback  # cb to process the feedback
        )
        self.node = node

    def create_goal_handler(self, blackboard: Blackboard) -> Simpleaction.Goal:
        goal = Simpleaction.Goal()
        goal.simple_request = int(blackboard["action_goal"])
        self.node.get_logger().info("Executing action SIMPLEACTION with goal: {goal.simple_request}")
        return goal

    def response_handler(
        self,
        blackboard: Blackboard,
        response: Simpleaction.Result
    ) -> str:

        blackboard["action_result"] = response.simple_result
        self.node.get_logger().info(f"Action completed with result: {blackboard["action_result"]}")
        return SUCCEED

    def print_feedback(
        self,
        blackboard: Blackboard,
        feedback: Simpleaction.Feedback
    ) -> None:
        print(f"Received feedback: {list(feedback.simple_feedback)}")


# define StateMachineNode
class StateMachineNode(Node):
    def __init__(self):
        super().__init__('yasmin_test_sm')
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
    print(f"sm final outcome is: outcome4")

    # add states
    sm.add_state(
        "FOO",
        FooState(node),
        transitions={
            "outcome1": "BAR",
            "outcome2": "outcome4",
            "do_Action": "DOING_ACTION"
        }
    )
    sm.add_state(
        "DOING_ACTION",
        SimpleActionState(node),
        transitions={
            SUCCEED: "BAR",
            CANCEL: "outcome4",
            ABORT: "FOO"
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
    YasminViewerPub("yasmin_test", sm)

    # create an initial blackboard
    blackboard = Blackboard()

    try:
        # execute FSM
        outcome = sm(blackboard)
        print(f"Reached final outcome: {outcome}")
    except Exception as e:
        print(e)

    rclpy.spin(node)

    # shutdown ROS 2
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

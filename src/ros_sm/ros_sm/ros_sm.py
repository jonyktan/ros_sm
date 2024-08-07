#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node

# Import std msg String for publishing to ros_sm/event topic to trigger state transitions
from std_msgs.msg import String

# yasmin imports to setup state machine
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub
from yasmin import CbState
from yasmin_ros import ActionState, ServiceState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL

# custom interface imports for actions & services
from custom_interfaces.action import Simpleactiontype
from custom_interfaces.srv import Simpleservicetype



# define state Init
class InitState(State):
    def __init__(self, node) -> None:
        super().__init__(["do_Action", "call_Service", "failed"])
        self.node = node

    def execute(self, blackboard: Blackboard) -> str:
        self.node.get_logger().info("Executing state INIT")
        event = self.node.wait_for_event()

        time.sleep(3)

        if event['event'] == "do_action":
            blackboard["event"] = f"do_action"
            try:
            # Ensure that action goal matches request definition in .action file (integer)
                blackboard["action_goal"] = int(float(event["value"]))
                self.node.get_logger().info(f"Action goal is: {blackboard["action_goal"]}")                
                return "do_Action"
            except TypeError as e:
                print(e)
        elif event['event'] == 'call_service':
            blackboard["event"] = f"call_service"
            # Ensure that service request matches definition in .srv file (3 integers)
            try:
                request = event["value"].split(',')
                if len(request) == 3:
                    blackboard["service_request"] = [int(request[0]),
                                                     int(request[1]),
                                                     int(request[2])]
                    self.node.get_logger().info(f"Service request is: {blackboard["service_request"]}")
                    return "call_Service"
                else:
                    raise Exception("Wrong service request format")
            except Exception as e:
                print(e)
        else:
            self.node.get_logger().info(f"Unrecognized outcome, state transition failed")
            return "failed"


# define state DoingAction
class DoingActionState(ActionState):
    def __init__(self, node) -> None:
        super().__init__(
            action_type=Simpleactiontype,  # action type
            action_name="/simple_action_name",  # action name
            create_goal_handler=self.create_goal_handler,  # cb to create the goal
            outcomes=None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            result_handler=self.response_handler,  # cb to process the response
            feedback_handler=self.print_feedback  # cb to process the feedback
        )
        self.node = node

    def create_goal_handler(self, blackboard: Blackboard) -> Simpleactiontype.Goal:
        goal = Simpleactiontype.Goal()
        goal.simple_request = int(blackboard["action_goal"])
        self.node.get_logger().info("Executing SIMPLEACTION with goal: {goal.simple_request}")
        return goal

    def response_handler(
        self,
        blackboard: Blackboard,
        response: Simpleactiontype.Result
    ) -> str:

        blackboard["action_result"] = response.simple_result
        self.node.get_logger().info(f"Action completed with result: {blackboard["action_result"]}")
        return SUCCEED

    def print_feedback(
        self,
        blackboard: Blackboard,
        feedback: Simpleactiontype.Feedback
    ) -> None:
        print(f"Received feedback: {list(feedback.simple_feedback)}")


# define state RunningService
class RunningServiceState(ServiceState):
    def __init__(self, node) -> None:
        super().__init__(
            srv_type=Simpleservicetype,  # srv type
            srv_name="/simple_service_name",  # service name
            create_request_handler=self.create_request_handler,  # cb to create the request
            outcomes=None,  # outcomes. Includes (SUCCEED, ABORT)
            response_handler=self.response_handler  # cb to process the response
        )
        self.node = node

    def create_request_handler(self, blackboard: Blackboard) -> Simpleservicetype.Request:

        req = Simpleservicetype.Request()
        self.node.get_logger().info(f"Service request received: {req}")
        try:
            service_request = blackboard["service_request"]
            req.a = int(service_request[0])
            req.b = int(service_request[1])
            req.c = int(service_request[2])
            self.node.get_logger().info(f"Calling SIMPLESERVICE with request: {req}")
        except Exception as e:
            print(f'{e}')
            return ABORT
        return req

    def response_handler(
        self,
        blackboard: Blackboard,
        response: Simpleservicetype.Response
    ) -> str:

        blackboard["service_result"] = response.sum
        self.node.get_logger().info(f"Service completed with result: {blackboard["service_result"]}")
        return SUCCEED


# define state Standby
class StandbyState(State):
    def __init__(self, node) -> None:
        super().__init__(["do_Action", "call_Service", "re_Init", "failed", "end"])
        self.node = node

    def execute(self, blackboard: Blackboard) -> str:
        self.node.get_logger().info("Executing state STANDBY")
        event = self.node.wait_for_event()

        time.sleep(3)

        if event['event'] == "do_action":
            blackboard["event"] = f"do_action"
            try:
            # Ensure that action goal matches request definition in .action file (integer)
                blackboard["action_goal"] = int(float(event["value"]))
                self.node.get_logger().info(f"Action goal is: {blackboard["action_goal"]}")
                return "do_Action"
            except TypeError as e:
                print(e)
        elif event['event'] == 'call_service':
            blackboard["event"] = f"call_service"
            # Ensure that service request matches definition in .srv file (3 integers)
            try:
                request = event["value"].split(',')
                if len(request) == 3:
                    blackboard["service_request"] = [int(request[0]),
                                                     int(request[1]),
                                                     int(request[2])]
                    self.node.get_logger().info(f"Service request is: {blackboard["service_request"]}")
                    return "call_Service"
                else:
                    raise Exception("Wrong service request format")
            except Exception as e:
                print(e)
        elif event['event'] == 're_init':
            self.node.get_logger().info(f"Re-enter state INIT")
            return "re_Init"
        elif event['event'] == "end":
            self.node.get_logger().info(f"End State Machine")
            return "end"
        else:
            self.node.get_logger().info(f"Unrecognized outcome, state transition failed")
            return "failed"


# define StateMachineNode
class StateMachineNode(Node):
    def __init__(self):
        super().__init__('ros_sm_node')
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
            self.current_event['value'] = parts[1] if len(parts) > 1 else None
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

    print("ros_sm node running")

    # init ROS 2
    rclpy.init()

    node = StateMachineNode()

    # create a FSM
    sm = StateMachine(outcomes=["END"])
    print(f"sm final outcome is: END")

    # add states
    sm.add_state(
        "INIT",
        InitState(node),
        transitions={
            "do_Action": "DOING_ACTION",
            "call_Service": "CALLING_SERVICE",
            "failed": "INIT"
        }
    )
    sm.add_state(
        "DOING_ACTION",
        DoingActionState(node),
        transitions={
            SUCCEED: "STANDBY",
            CANCEL: "END",
            ABORT: "STANDBY", # TODO: return to previous state
        }
    )
    sm.add_state(
        "CALLING_SERVICE",
        RunningServiceState(node),
        transitions={
            SUCCEED: "STANDBY",
            ABORT: "STANDBY" # TODO: return to previous state
        }
    )
    sm.add_state(
        "STANDBY",
        StandbyState(node),
        transitions={
            "do_Action": "DOING_ACTION",
            "call_Service": "CALLING_SERVICE",
            "re_Init": "INIT",
            "failed": "STANDBY",
            "end": "END"
        }
    )

    # pub FSM info
    YasminViewerPub("ROS_SM using YASMIN", sm)

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
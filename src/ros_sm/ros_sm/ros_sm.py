#!/usr/bin/env python3

import time
import rclpy
from yasmin import State, StateMachine, CbState
from yasmin import Blackboard
from yasmin_viewer import YasminViewerPub
from yasmin_ros import ActionState, MonitorState, ServiceState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL, TIMEOUT


from std_msgs.msg import String
from custom_action_interfaces.action import Simpleaction


# Define state Init
class InitState(State):
    def __init__(self) -> None:
        super().__init__(["do_action", "call_service", "failed"])
        self.counter = 0

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

    def execute(self, blackboard: Blackboard) -> str:
        print("Executing state INIT")
        time.sleep(3)

        if self.counter < 3:
            self.counter += 1
            blackboard["foo_str"] = f"Counter: {self.counter}"
            return "outcome1"
        else:
            return "outcome2"
# class InitState(smach.State):
#     def __init__(self, node):
#         smach.State.__init__(self, outcomes=['do_action', 'call_service', 'failed'], output_keys=['action_goal'])
#         self.node = node

#     def execute(self, userdata):
#         self.node.get_logger().info('Executing state InitState')
#         event = self.node.wait_for_event()
#         if event['event'] == 'do_action':
#             userdata.action_goal = event['value']
#             return 'do_action'
#         elif event['event'] == 'call_service':
#             return 'call_service'
#         elif event['event'] == 'failed':
#             return 'failed'

# Define state DoingAction
class DoingActionState(ActionState):
    def __init__(self) -> None:
        super().__init__(
            action_type=Simpleaction,
            action_name='/simple_action',
            create_goal_handler=self.create_goal_handler,
            outcomes=['action_complete', 'failed'],
            result_handler=self.response_handler,
            feedback_handler=self.print_feedback
            )

    def create_goal_handler(self, blackboard: Blackboard) -> Simpleaction.Goal:

        goal = Simpleaction.Goal()
        goal.order = blackboard["n"]
        return goal    

    def response_handler(
        self,
        blackboard: Blackboard,
        response: Simpleaction.Result
    ) -> str:

        blackboard["simpleaction_result"] = response.simple_result
        self.print_result()

    def print_feedback(
        self,
        blackboard: Blackboard,
        feedback: Simpleaction.Feedback
    ) -> None:
        print(f"Received feedback: {list(feedback.simple_feedback)}")    

    def print_result(blackboard: Blackboard) -> str:
        print(f"Result: {blackboard['simpleaction_result']}")
        return SUCCEED

# Define state RunningService
class RunningServiceState(ServiceState):
    def __init__(self) -> None:
        super().__init__(
            srv_type=SimpleService,
            srv_name="/simple_service",
            create_request_handler=self.create_request_handler,
            outcomes=["service_ended", "failed"],
            response_handler=self.response_handler
        )

    def create_request_handler(self, blackboard: Blackboard) -> SimpleService.Request:

        req = SimpleService.Request()
        req.a = blackboard["a"]
        req.b = blackboard["b"]
        return req

    def response_handler(
        self,
        blackboard: Blackboard,
        response: SimpleService.Response
    ) -> str:

        blackboard["sum"] = response.sum
        return SUCCEED
    
# class RunningServiceState(smach_ros.ServiceState):
#     def __init__(self, node):
#         smach_ros.ServiceState.__init__(self, outcomes=['service_ended', 'failed'])
#         self.node = node

#     def execute(self, userdata):
#         self.node.get_logger().info('Executing state RunningServiceState')
#         event, _ = self.node.wait_for_event()
#         if event == 'service_ended':
#             return 'service_ended'
#         elif event == 'failed':
#             return 'failed'

# # Define state Standby
# class StandbyState(smach.State):
#     def __init__(self, node):
#         smach.State.__init__(self, outcomes=['do_action', 'call_service', 're_init', 'failed'])
#         self.node = node

#     def execute(self, userdata):
#         self.node.get_logger().info('Executing state Standby')
#         event = self.node.wait_for_event()
#         if event['event'] == 'do_action':
#             return 'do_action'
#         elif event == 'call_service':
#             return 'call_service'
#         elif event == 're_init':
#             return 're_init'
#         elif event == 'failed':
#             return 'failed'

# class StateMachineNode(Node):
#     def __init__(self):
#         super().__init__('ros_state_machine')
#         self.event_sub = self.create_subscription(
#             String,
#             'ros_sm/event',
#             self.event_callback,
#             10
#         )
#         self.current_event = {'event':None, 'value':None}

#     def event_callback(self, msg):
#         try:
#             parts = msg.data.split()
#             self.current_event['event'] = parts[0]
#             self.current_event['value'] = float(parts[1]) if len(parts) > 1 else None
#         except (IndexError, ValueError):
#             self.get_logger().error(f'Invalid event message format: {msg.data}')
#             self.current_event = {'event':None, 'value':None}

#     def wait_for_event(self):
#         print('waiting for event')
#         while rclpy.ok() and self.current_event == {'event':None, 'value':None}:
#             rclpy.spin_once(self)
#         event = self.current_event
#         self.current_event = {'event':None, 'value':None}
#         return event

# main
def main(args=None):
    rclpy.init(args=args)

    # Create a FSM
    sm = StateMachine(outcomes=['FAILED'])

    # Add states
    sm.add_state(
        "INIT",
        InitState(),
        transitions={
            "do_action":"ACTION",
            "call_service":"SERVICE",
            "standby":"STANDBY",
            "failed":"FAILED"
        }
    )
    sm.add_state(
        "ACTION",
        DoingActionState(),
        transitions={
            SUCCEED:"STANDBY",
            CANCEL:"ACTION",
            ABORT:"FAILED"
        }
    )
    sm.add_state(
        "SERVICE",
        RunningServiceState(),
        transitions={
            SUCCEED:"STANDBY",
            ABORT:"FAILED"
        }
    )
    sm.add_state(
        "STANDBY",
        StandbyState(),
        transitions={
            "do_action":"ACTION",
            "call_service":"SERVICE",
            "re_init":"INIT",
            "failed":"FAILED"
        }
    )

    # Pub FSM info
    YasminViewerPub("ROS_SM using YASMIN", sm)

    # Create an initial blackboard
    blackboard = Blackboard()
    blackboard["n"] = 10

    try:
        # Execute FSFM
        outcome = sm(blackboard)
        print(outcome)
    except Exception as e:
        print(e)

    # Shutdown ROS 2
    rclpy.shutdown()

if __name__ == '__main__':
    main()


#     # Create and start the introspection server
#     sis = smach_ros.IntrospectionServer('ros_sm', sm, '/ROS_SM')
#     sis.start()

#     try:
#         # Execute SMACH plan
#         outcome = sm.execute()
#     except Exception as e:
#         print(e)

#     # Wait for ctrl-c to stop the application
#     rclpy.spin(node)
#     sis.stop()

#     node.destroy_node()
#     rclpy.shutdown()
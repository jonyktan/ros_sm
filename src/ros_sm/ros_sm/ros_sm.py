#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import smach
import smach_ros

from custom_action_interfaces.action import Simpleaction
# from action_servers.simple_action_server import SimpleActionServer

# Define state Docked
class InitState(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['do_action', 'call_service', 'failed'], output_keys=['action_goal'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info('Executing state InitState')
        event = self.node.wait_for_event()
        if event['event'] == 'do_action':
            userdata.action_goal = event['value']
            return 'do_action'
        elif event['event'] == 'call_service':
            return 'call_service'
        elif event['event'] == 'failed':
            return 'failed'

# Define state DoingAction
class DoingActionState(smach_ros.SimpleActionState):
    def __init__(self, node):
        smach_ros.SimpleActionState.__init__(self, node=node, action_name='simple_action', action_spec=Simpleaction, goal_cb=self.goal_callback, input_keys=['action_goal'], result_key='action_result', result_cb=self.result_callback, outcomes=['action_complete', 'failed'])

    @staticmethod
    def goal_callback(userdata, goal):
        # Specify the action goal from userdata, ensuring that the type matches the goal type specified in the action. 
        # Did not use goal_key as userdata.goal_key would have been float though action goal expecting int32. 
        goal.simple_request = int(userdata.action_goal)
        print(f'action goal is: {goal.simple_request}')
    
    # TODO: debug why result_callback does not seem to be called. Printout does not appear in ros_sm node terminal. 

    @staticmethod
    def result_callback(result):
        print(f'result is: {result.action_result}')
    
    # TODO: define method to return event to trigger state transition

    # def execute(self, userdata):
    #     self.node.get_logger().info('Executing state DoingActionState')
    #     print(f'userdata is {userdata.action_goal}')
    #     event = self.node.wait_for_event()
    #     if event['event'] == 'action_complete':
    #         return 'action_complete'
    #     elif event['event'] == 'failed':
    #         return 'failed'

# Define state RunningService
class RunningServiceState(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['service_ended', 'failed'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info('Executing state RunningServiceState')
        event, _ = self.node.wait_for_event()
        if event == 'service_ended':
            return 'service_ended'
        elif event == 'failed':
            return 'failed'

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

# Define state Standby
class StandbyState(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['do_action', 'call_service', 're_init', 'failed'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info('Executing state Standby')
        event = self.node.wait_for_event()
        if event['event'] == 'do_action':
            return 'do_action'
        elif event == 'call_service':
            return 'call_service'
        elif event == 're_init':
            return 're_init'
        elif event == 'failed':
            return 'failed'

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

def main(args=None):
    rclpy.init(args=args)

    node = StateMachineNode()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['failed'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('INIT', InitState(node), 
                            transitions={'do_action':'ACTION', 
                                         'call_service':'SERVICE', 
                                         'failed':'failed'},
                            remapping={'action_goal':'action_goal'})
        #  transitions={'aborted','preempted','succeeded'} are default in SimpleActionState
        smach.StateMachine.add('ACTION', DoingActionState(node), 
                            transitions={'action_complete':'STANDBY', 
                                        'failed':'failed',
                                        'succeeded':'STANDBY',
                                        'aborted':'failed',
                                        'preempted':'ACTION'},
                            remapping={'action_goal':'action_goal'})
        smach.StateMachine.add('SERVICE', RunningServiceState(node), 
                            transitions={'service_ended':'STANDBY',
                                         'failed':'failed'})
        smach.StateMachine.add('STANDBY', StandbyState(node), 
                            transitions={'do_action':'ACTION',
                                            'call_service':'SERVICE', 
                                            're_init':'INIT',
                                            'failed':'failed'})    

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('ros_sm', sm, '/ROS_SM')
    sis.start()

    try:
        # Execute SMACH plan
        outcome = sm.execute()
    except Exception as e:
        print(e)

    # Wait for ctrl-c to stop the application
    rclpy.spin(node)
    sis.stop()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

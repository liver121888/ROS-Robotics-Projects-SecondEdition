#!/usr/bin/env python
import rospy
import smach
import time
from random import randint
from smach import State, StateMachine
from smach_ros import SimpleActionState, IntrospectionServer
from geometry_msgs.msg import Twist
from math import  pi
import actionlib
from actionlib import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from tf.transformations import quaternion_from_euler
from collections import OrderedDict
from std_msgs.msg import Float32, Bool
from smach_example.msg import battery_simAction, battery_simGoal, battery_simResult


low_percentage = 50.0
ready_percentage = 100.0
percentage = 0.0

def callback(msg):
    global percentage
    percentage = msg.data
    rospy.loginfo(f'capacity: {percentage}')

def lowPercentage():
    global percentage, low_percentage
    if percentage < low_percentage:
        return True
    else:
        return False

pub = rospy.Publisher('isCharging', Bool, queue_size=10)
sub = rospy.Subscriber('battery_percentage', Float32, callback, queue_size=10)

class PowerOnRobot(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo("Powering ON robot...")
        time.sleep(2)
        return 'succeeded' 

class Charging(State):    
    def __init__(self):
        State.__init__(self, outcomes=['full','charging'])

    def execute(self, userdata):
        global percentage
        rospy.loginfo("Checking remaining power...")
        time.sleep(2)
        if percentage >= ready_percentage:
            # full so discharging, ready to work
            pub.publish(False)
            return 'full'    
        else:
            pub.publish(True)
            return 'charging'    

class CheckButtonState(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        if not lowPercentage():
            if randint(0, 1) == 1:
                rospy.loginfo("Got order...")
                return 'succeeded'
            else:
                rospy.loginfo("Waiting for order...")
                return 'aborted'
        else:
            return 'preempted'


class OrderConfirmation(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        if not lowPercentage():
            rospy.loginfo("Confirmation order...")
            time.sleep(2)
            if randint(0, 1) == 1:
                rospy.loginfo("Order confirmed...")
                return 'succeeded'
            else:
                rospy.loginfo("Cannot confirm order...")
                return 'aborted'
        else:
            return 'preempted'

class SpeakOut(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        if not lowPercentage():
            rospy.sleep(1)
            rospy.loginfo ("Please confirm the order")
            rospy.sleep(5)
            if randint(0, 1) == 1:
                rospy.loginfo ("Chief confirmed the order")
                return 'succeeded'
            else:
                rospy.loginfo ("Chief did not confirm the order, retrying...")
                return 'aborted'
        else:
            return 'preempted'

class main():
    def __init__(self):
        rospy.init_node('waiter_robot', anonymous=False)
        rospy.on_shutdown(self.shutdown)        
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(15))
        rospy.loginfo("Connected to move_base action server")

        
        quaternions = list()
        euler_angles = (pi/2, pi, 3*pi/2, 0)
        for angle in euler_angles:
                q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
                q = Quaternion(*q_angle)
                quaternions.append(q)
    
        # Create a list to hold the waypoint poses
        self.waypoints = list()
        self.waypoints.append(Pose(Point(0.0, 0.0, 0.0), quaternions[3]))
        self.waypoints.append(Pose(Point(-1.0, -1.5, 0.0), quaternions[0]))
        self.waypoints.append(Pose(Point(1.5, 1.0, 0.0), quaternions[1]))
        self.waypoints.append(Pose(Point(2.0, -2.0, 0.0), quaternions[1]))
        self.waypoints.append(Pose(Point(1.0, -2.5, 0.0), quaternions[0]))
        room_locations = (('table', self.waypoints[0]),
                      ('delivery_area', self.waypoints[1]),
                      ('kitchen', self.waypoints[2]),
                      ('home', self.waypoints[3]),
                      ('charging_area', self.waypoints[4]))
    
        # Store the mapping as an ordered dictionary so we can visit the rooms in sequence
        self.room_locations = OrderedDict(room_locations)
        nav_states = {}
        
        for room in self.room_locations:
            nav_goal = MoveBaseGoal()
            nav_goal.target_pose.header.frame_id = 'map'
            nav_goal.target_pose.pose = self.room_locations[room]
            move_base_state = SimpleActionState('move_base', MoveBaseAction, goal=nav_goal, result_cb=self.move_base_result_cb, 
                                                exec_timeout=rospy.Duration(6.0),
                                                server_wait_timeout=rospy.Duration(6.0))
            nav_states[room] = move_base_state


        sm_order_confirmation = StateMachine(outcomes=['succeeded','aborted','preempted'])
        with sm_order_confirmation:
            StateMachine.add('ORDER_CONFIRMATION_PROCESS', OrderConfirmation(), transitions={'succeeded':'','aborted':'','preempted':''})

        sm_restaurant = StateMachine(outcomes=['succeeded','aborted','preempted'])
        with sm_restaurant:            
            StateMachine.add('POWER_ON', PowerOnRobot(), transitions={'succeeded':'GO_TO_CHARGING_AREA'})
            StateMachine.add('GO_TO_CHARGING_AREA', Charging(), transitions={'full':'CHECK_BUTTON_STATE', 'charging':'GO_TO_CHARGING_AREA'})        
            StateMachine.add('CHECK_BUTTON_STATE', CheckButtonState(), transitions={'succeeded':'GO_TO_CUSTOMER_TABLE','aborted':'CHECK_BUTTON_STATE','preempted':'GO_TO_CHARGING_AREA'})
            StateMachine.add('GO_TO_CUSTOMER_TABLE', nav_states['table'], transitions={'succeeded':'ORDER_CONFIRMATION','aborted':'ORDER_CONFIRMATION','preempted':'GO_TO_CHARGING_AREA'})
            StateMachine.add('ORDER_CONFIRMATION', sm_order_confirmation, transitions={'succeeded':'GO_TO_DELIVERY_AREA','aborted':'GO_TO_KITCHEN','preempted':'GO_TO_CHARGING_AREA'})
            StateMachine.add('GO_TO_KITCHEN', nav_states['kitchen'], transitions={'succeeded':'SPEAK_OUT','aborted':'GO_TO_KITCHEN','preempted':'GO_TO_CHARGING_AREA'})
            StateMachine.add('GO_TO_DELIVERY_AREA', nav_states['delivery_area'], transitions={'succeeded':'DELIVER_FOOD','aborted':'DELIVER_FOOD','preempted':'GO_TO_CHARGING_AREA'})
            StateMachine.add('SPEAK_OUT', SpeakOut(), transitions={'succeeded':'GO_TO_DELIVERY_AREA','aborted':'SPEAK_OUT','preempted':'GO_TO_CHARGING_AREA'})
            StateMachine.add('DELIVER_FOOD', nav_states['table'], transitions={'succeeded':'GO_TO_HOME','aborted':'GO_TO_HOME','preempted':'GO_TO_CHARGING_AREA'})
            StateMachine.add('GO_TO_HOME', nav_states['home'], transitions={'succeeded':'CHECK_BUTTON_STATE','aborted':'GO_TO_HOME','preempted':'GO_TO_CHARGING_AREA'})

        intro_server = IntrospectionServer('restaurant', sm_restaurant, '/SM_ROOT')
        intro_server.start()
        
        # Execute the state machine
        sm_outcome = sm_restaurant.execute()      
        intro_server.stop()

    def move_base_result_cb(self, userdata, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            pass
        else:
            self.wait_for_result()
            
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Restaurant robot test finished.")

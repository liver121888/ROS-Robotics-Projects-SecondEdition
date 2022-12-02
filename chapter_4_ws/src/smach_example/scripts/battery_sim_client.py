#! /usr/bin/env python

import sys
import rospy
from std_msgs.msg import Int32, Bool, Float32
import actionlib
from battery_simulator.msg import battery_simAction, battery_simGoal, battery_simResult, battery_simFeedback


class Client():
	def __init__(self):
		self.ac = actionlib.SimpleActionClient('battery_simulator', battery_simAction)
		self.pub = rospy.Publisher('battery_percentage', Float32, queue_size=10)
		self.charging = False
		self.sub = rospy.Subscriber('isCharging', Bool, self.charging_callback, queue_size=10)
		self.ac.wait_for_server()
		self.send_battery_state(0)
		self.ac.wait_for_result()

	def charging_callback(self, msg):
		rospy.loginfo('self.charging {}, charging {}'.format(self.charging, msg.data))
		if self.charging != msg.data:
			self.charging = msg.data
			if self.charging == False:
				self.send_battery_state(0)
			else:
				self.send_battery_state(1)


	def send_battery_state(self, charge_condition):
		goal = battery_simGoal()
		goal.charge_state = charge_condition
		self.ac.send_goal(goal, feedback_cb=self.read_percentage)

	def read_percentage(self, feedback):
		percent = feedback.battery_percentage
		rospy.loginfo(percent)
		self.pub.publish(percent)

if __name__ == '__main__':
	rospy.init_node('BatterySimClient')
	Client()
	rospy.spin()
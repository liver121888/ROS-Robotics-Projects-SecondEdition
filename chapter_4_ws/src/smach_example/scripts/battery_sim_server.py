#! /usr/bin/env python

import time
import rospy
from multiprocessing import Process
from std_msgs.msg import Int32, Bool
import actionlib
from battery_simulator.msg import battery_simAction, battery_simGoal, battery_simResult, battery_simFeedback


battery_capacity = 100

class Server():
	def __init__(self):
		self._as = actionlib.SimpleActionServer('battery_simulator', battery_simAction, self.goalFun, False)
		self._as.start()
		self.battery_level = battery_capacity

	def goalFun(self, goal):
		if goal.charge_state == 0:
			rospy.set_param("/MyRobot/BatteryStatus",goal.charge_state)
		elif goal.charge_state == 1:
			rospy.set_param("/MyRobot/BatteryStatus",goal.charge_state)
		result = battery_simResult()
		feedback = battery_simFeedback()
		while not rospy.is_shutdown():
			if self._as.is_preempt_requested():
				rospy.loginfo('%s: Preempted' % rospy.get_name())
				self._as.set_preempted()
				break

			if rospy.has_param("/MyRobot/BatteryStatus"):
				time.sleep(1)
				param = rospy.get_param("/MyRobot/BatteryStatus")
				if param == 1:
					if self.battery_level == 100:
						result.battery_status = "Full"
						self._as.set_succeeded(result)
					else:
						self.battery_level += 1
						rospy.loginfo("Charging...currently, %s", self.battery_level)
						feedback.battery_percentage = self.battery_level
						self._as.publish_feedback(feedback)	
						time.sleep(4)	
				elif param == 0:
					self.battery_level -= 1
					rospy.logwarn("Discharging...currently, %s", self.battery_level)
					feedback.battery_percentage = self.battery_level
					self._as.publish_feedback(feedback)	
					time.sleep(2)

if __name__ == '__main__':	
	rospy.init_node('BatterySimServer')
	Server();	
	rospy.spin()


#! /usr/bin/env python
import rospy
import actionlib
import actionlib_tutorials.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback

class ActionServer(object):
    # create messages that are used to publish feedback/result

    def __init__(self):
        self._action_name = "move_base"
        self._as = actionlib.SimpleActionServer(self._action_name, MoveBaseAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
    def execute_cb(self, goal):
        # assume the action succeeds
        success = True
        rospy.loginfo('Executing')
        rospy.loginfo(goal)

        # start executing the action
        # check that preempt has not been requested by the client
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
        # no feedback for client
        # self._feedback.sequence.append("")
        # publish the feedback
        # self._as.publish_feedback(self._feedback)
        rospy.sleep(5)
        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            # if msg.goal == 0:
            self._as.set_succeeded()
            # elif msg.goal == 1:
            #     self._as.set_aborted()
            # elif msg.goal == 2:
            #     self._as.set_preempted()
        
if __name__ == '__main__':
    rospy.init_node('action_server')
    server = ActionServer()
    rospy.spin()
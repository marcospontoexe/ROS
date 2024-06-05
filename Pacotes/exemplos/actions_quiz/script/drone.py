#!/usr/bin/env python

import rospy
import actionlib
from actions_quiz.msg import CustomActionMsgAction, CustomActionMsgFeedback, CustomActionMsgResult
from std_msgs.msg import Empty

class DroneActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('/action_custom_msg_as', CustomActionMsgAction, self.execute, False)
        self.feedback = CustomActionMsgFeedback() # create messages that are used to publish feedback/result
        self.result = CustomActionMsgResult()
        self.takeoff_pub = rospy.Publisher('/drone/takeoff', Empty, queue_size=10)
        self.land_pub = rospy.Publisher('/drone/land', Empty, queue_size=10)
        self.server.start()

    def execute(self, goal): # this callback is called when the action server is called. 
        rate = rospy.Rate(1)
        # check that preempt (cancelation) has not been requested by the action client
        if self.server.is_preempt_requested():
            rospy.loginfo('The goal has been cancelled/preempted')
            # the following line, sets the client in preempted state (goal cancelled)
            self.server.set_preempted()
            success = False
        if goal.goal == "TAKEOFF":
            self.takeoff_pub.publish(Empty())
            self.feedback.feedback = "Taking off"
            rospy.loginfo("Drone is taking off")
        elif goal.goal == "LAND":
            self.land_pub.publish(Empty())
            self.feedback.feedback = "Landing"
            rospy.loginfo("Drone is landing")
        else:
            rospy.loginfo("Invalid goal")
            self.server.set_aborted()
            return
        
        # the sequence is computed at 1 Hz frequency
        rate.sleep()

        # at this point, either the goal has been achieved (success==true)
        # or the client preempted the goal (success==false)
        # If success, then we publish the final result
        # If not success, we do not publish anything in the result
        if success:
            self._result = Empty()
            self.server.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('move_drone_node')
    server = DroneActionServer()
    rospy.spin()
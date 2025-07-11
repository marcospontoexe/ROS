#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
txt = ""
def callback(msg):
    global txt
    rospy.loginfo(f"Callback: {msg.data}")
    txt = msg.data
    
    # rospy.sleep(0.5)  # Simula um processo lento

def listener():
    rospy.loginfo(f"Main: {txt}")
    # rospy.spin()

if __name__ == '__main__':
    rospy.init_node('slow_listener_1', anonymous=True)
    rospy.Subscriber('chatter', String, callback)
    rate = rospy.Rate(2)  
    
    while not rospy.is_shutdown():
        listener()
        rate.sleep()

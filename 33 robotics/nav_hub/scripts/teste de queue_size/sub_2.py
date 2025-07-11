#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(msg):
    rospy.loginfo(f"Recebido: {msg.data}")
    # rospy.sleep(0.5)  # Simula um processo lento

def listener():
    rospy.init_node('slow_listener_3', anonymous=True)
    rospy.Subscriber('chatter', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

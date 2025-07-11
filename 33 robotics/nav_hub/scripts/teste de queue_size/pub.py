#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    pub = rospy.Publisher('chatter', String, queue_size=1, latch=True)  # Mude aqui para testar diferentes tamanhos
    rospy.init_node('fast_talker', anonymous=True)
    rate = rospy.Rate(100)  # 100 Hz
    count = 0
    msg = f"Mensagem {count}"
    pub.publish(msg)
    rospy.loginfo(f"Publicado: {msg}")
    count += 1
    while not rospy.is_shutdown():
        
        rate.sleep()
   

#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from nav_hub.msg import CustomJoy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import os

class Vel:
    def __init__(self):
        """
        Essa classe recebe uma velocidade do tópico /cmd_vel_controller
        e publica novamnete no tópico /cmd_vel de forma a suavizar os trancos
        """
        rospy.init_node("VelocidadeSuave_node", anonymous=True)

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
 
        rospy.Subscriber("/cmd_vel_controller", Twist, self.vel_callback, queue_size=1)
        # rospy.Subscriber("/cmd_vel", Twist, self.vel_callback)

        self.velocity = Twist()
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0

        self.lastVel = Twist()
        self.lastVel.linear.x = 0
        self.lastVel.angular.z = 0
        
        
    def vel_callback(self, data):
        self.velocity = data


    def main(self):
        
        # para controlar transissões brutas
        self.lastVel.linear.x = self.lastVel.linear.x + ((self.velocity.linear.x - self.lastVel.linear.x) * 0.25)
        self.lastVel.angular.z = self.lastVel.angular.z + ((self.velocity.angular.z - self.lastVel.angular.z) * 0.25)
            
        self.cmd_vel_pub.publish(self.lastVel)


if __name__ == '__main__':
    try:
        objt = Vel()
        rate = rospy.Rate(5)
    except rospy.ROSInterruptException:
        rospy.loginfo("Erro ao iniciar o objeto Vel()")
        pass
    
    while not rospy.is_shutdown():
        objt.main()
        rate.sleep()
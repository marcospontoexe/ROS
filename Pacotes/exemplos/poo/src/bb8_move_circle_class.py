#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class MoveBB8():
    
    def __init__(self): # construtor
        self.bb8_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()
        self.ctrl_c = False
        self.rate = rospy.Rate(1) # 1hz
        rospy.on_shutdown(self.shutdownhook)    # quando o nó for encerrado
        
    def publish_once_in_cmd_vel(self):
        """
        Isso ocorre porque publicar em tópicos às vezes falha na primeira vez que você publica. 
        Em sistemas de publicação contínua, isso não é um grande problema, 
        mas em sistemas que publicam apenas uma vez, ISSO é muito importante.
        """
        while not self.ctrl_c:
            connections = self.bb8_vel_publisher.get_num_connections()
            if connections > 0:
                self.bb8_vel_publisher.publish(self.cmd)
                rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()
        
    def shutdownhook(self): # método chamado quando o nó for encerrado
        # works better than the rospy.is_shutdown()
        self.stop_bb8()
        self.ctrl_c = True

    def stop_bb8(self):
        rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    def move_bb8(self, moving_time, linear_speed=0.2, angular_speed=0.2):
        self.cmd.linear.x = linear_speed
        self.cmd.angular.z = angular_speed
        i = 0
        
        rospy.loginfo("Moving BB8!")
        while not self.ctrl_c and i <= moving_time: # enquanto não apertar "ctrl+c" ou acabar o tempo definido
            self.publish_once_in_cmd_vel()
            i = i+1
            self.rate.sleep()
            
        self.stop_bb8()
            
if __name__ == '__main__':
    rospy.init_node('move_bb8_test', anonymous=True)
    movebb8_object = MoveBB8() # instanciando um objeto "movebb8_object" da classe  MoveBB8() 
    try:
        movebb8_object.move_bb8()
    except rospy.ROSInterruptException:
        pass
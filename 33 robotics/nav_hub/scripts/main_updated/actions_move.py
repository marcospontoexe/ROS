#!/usr/bin/env python3

import time
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import math
import tf

class Actions_Robot:

    def __init__(self):
        self.current_yaw = None

        self.vel = Twist()

        #subscribers
        rospy.Subscriber("/keyence_pose", PoseWithCovarianceStamped, self.poseCallBack, queue_size=1)  # Para verificar a posição do robô

        #publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
   
    
    def poseCallBack(self, data):
        try:
            # Converte quaternion para yaw
            orientation_q = data.pose.pose.orientation
            _, _, yaw = tf.transformations.euler_from_quaternion([
                orientation_q.x,
                orientation_q.y,
                orientation_q.z,
                orientation_q.w
            ])
            self.current_yaw = yaw
        except:
            rospy.logwarn("Não foi possível obter a pose do robo!")


    def normalize_angle(self, angle):
        """Normaliza o ângulo para o intervalo [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))

    def girar(self):
        # Aguarda leitura inicial da orientação
        rospy.loginfo("Aguardando orientação inicial...")
        while self.current_yaw is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        # Salva orientação inicial
        orientation = self.current_yaw
        rospy.loginfo(f"Orientação inicial salva: {math.degrees(orientation):.2f} graus")

        self.vel.linear.x = 0.0  
        self.vel.angular.z = -0.3

        duration = 3.0  
        
        tempo_inicio = time.time()
        while (time.time() - tempo_inicio) < duration:
            self.cmd_vel_pub.publish(self.vel)
                    
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.vel)

            # Diferença angular entre atual e inicial
            angle_diff = self.normalize_angle(self.current_yaw - orientation)

            # Se completou 360 graus (ou quase)
            if abs(angle_diff) < 0.05 and abs(angle_diff) > 0:
                rospy.loginfo("Orientação inicial alcançada novamente. Parando...")
                break

        # Para o robô
        self.vel.angular.z = 0.0
        self.cmd_vel_pub.publish(self.vel)

    
if __name__ == '__main__':
    rospy.init_node("actions_move_node", anonymous=True)
    obj = Actions_Robot()

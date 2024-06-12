#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def laser_callback(data):   # Callback para lidar com os dados recebidos do laser

    distanciaFront = data.ranges[360]
    distanciaRight = data.ranges[719]
    print(f"DistÂncia a frente do robo: {distanciaFront}.")
    print(f"DistÂncia à direita do robo: {distanciaRight}.")
    
    # Twist para enviar comandos de velocidade
    vel_msg = Twist()
    
    if distanciaFront < 0.5:    # obstáculo a frente (virar pa à esquerda)
        vel_msg.linear.x = 0.1        
        vel_msg.angular.z = -1
    elif distanciaRight > 0.3: # vira para à direita (se aproximama da parede)
        vel_msg.linear.x = 0.1        
        vel_msg.angular.z = 0.5
    elif distanciaRight < 0.3: # vira para à esquerda (se afasta da parede)
        vel_msg.linear.x = 0.1        
        vel_msg.angular.z = -0.5
    else:                       # anda em linha reta
        vel_msg.linear.x = 0.5
        vel_msg.angular.z = 0.0

    
    vel_pub.publish(vel_msg)    # Publica a mensagem de velocidade
    

def laser_subscriber():    
    rospy.init_node('wall_following_subscriber_node', anonymous=True) # Inicializa o nó ROS
    
    # Cria um subscriber para o tópico '/kobuki/laser/scan' com a mensagem LaserScan
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    
    rospy.spin()    # Mantém o programa funcionando até que seja fechado

if __name__ == '__main__':    
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # Cria um publisher para o tópico '/cmd_vel' com a mensagem Twist    
    try:        
        laser_subscriber()  # Inicia o subscriber
    except rospy.ROSInterruptException:
        pass
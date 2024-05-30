#! /usr/bin/env python

import rospy
from services_quiz.srv import BB8CustomServiceMessage, BB8CustomServiceMessageResponse
from geometry_msgs.msg import Twist
import time
import signal
import sys

def my_callback(request):
    rospy.loginfo("The Service move_bb8_in_square_custom has been called")

    voltas = 0   
    i = 0
    while voltas <= request.repetitions: 
        while i < 4:
            lado = 0
            while lado <= request.side:
                # move reto
                move_square.linear.x = 0.4
                move_square.angular.z = 0.0
                my_pub.publish(move_square)
                rate.sleep()
                rate.sleep()
                lado = lado + 1

            # vira 90°
            move_square.linear.x = 0.0
            move_square.angular.z = 2.05
            my_pub.publish(move_square)
            rate.sleep()
            i = i+1

        voltas=voltas+1
        
    move_square.linear.x = 0
    move_square.angular.z = 0
    my_pub.publish(move_square)
    rospy.loginfo("Finished service move_bb8_in_circle_custom")
    
    #response = BB8CustomServiceMessageResponse()
    #response.success = True
    #return response # the service Response class, in this case True or False

    success = True
    return BB8CustomServiceMessageResponse(success)

def shutdown_hook():        # chamada durante o desligamento do nó
    rospy.loginfo("Shutting down BB8 Custom Service Server...")
    move_square.linear.x = 0
    move_square.angular.z = 0
    my_pub.publish(move_square)

'''
Captura o sinal de interrupção (SIGINT), que é enviado quando você pressiona Ctrl+C.
Chama rospy.signal_shutdown para iniciar o processo de encerramento do ROS.
'''
def signal_handler(sig, frame):     
    rospy.loginfo('Interrupt received, shutting down...')
    rospy.signal_shutdown('Interrupt received')

signal.signal(signal.SIGINT, signal_handler) #Registra o manipulador de sinal 
rospy.init_node('service_move_bb8_in_square_server_node') 
my_service = rospy.Service('/move_bb8_in_square_custom', BB8CustomServiceMessage , my_callback) # create the Service called move_bb8_in_square_custom with the defined callback
my_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
move_square = Twist()
rate = rospy.Rate(1)
rospy.loginfo("Service /move_bb8_in_square_custom Ready")
rospy.on_shutdown(shutdown_hook)    # garante que o shutdown_hook seja chamado quando o nó for encerrado.
rospy.spin() # maintain the service open.


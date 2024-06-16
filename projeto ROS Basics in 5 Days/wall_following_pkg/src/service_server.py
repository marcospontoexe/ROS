#! /usr/bin/env python

import rospy
from wall_following_pkg.srv import FindWall, FindWallResponse # you import the service message python classes generated from Empty.srv.
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

fimFlag = False
inicioFlag = True

def laser_callback(data):   # Callback para lidar com os dados recebidos do laser

    l = data.ranges
    distanciaTrig = data.ranges[270]
    distanciaFront = data.ranges[360]

    if inicioFlag:    
        min_distance = min(l)
        min_index = l.index(min_distance)
        print ("O menos distância é: ", min_distance)
        print(f"Feixe com maior distância: {min_index}")
        inicioFlag = False
    
    # Twist para enviar comandos de velocidade
    vel_msg = Twist()
   
    if distanciaFront > min_distance: # fica de frente para a parede mais próxima
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.1          
    elif distanciaFront >= 0.3:    # se aproxima até a parede mais próxima
        vel_msg.linear.x = 0.1       
        vel_msg.angular.z = 0.0
        min_distance = min(l)
    elif distanciaTrig > min_distance: # vira para à esquerda 
        vel_msg.linear.x = 0.0       
        vel_msg.angular.z =  0.5
    else :                       # anda em linha reta
        fimFlag = True
    
    
    vel_pub.publish(vel_msg)    # Publica a mensagem de velocidade
   

def my_callback(request):
    rospy.loginfo("The Service find_wall has been called")

    while fimFlag == False:
        # Cria um subscriber para o tópico '/scan' com a mensagem LaserScan
        rospy.Subscriber('/scan', LaserScan, laser_callback)    
        rospy.spin()    # Mantém o programa funcionando até que seja fechado
        
    rospy.loginfo("Finished service find_wall")
    return FindWallResponse(True) # the service Response class, in this case EmptyResponse

rospy.init_node('service_server_node') 
my_service = rospy.Service('/find_wall', FindWall , my_callback) # create the Service called find_wall with the defined callback
my_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
vel_pub = Twist()
rospy.loginfo("Service /move_bb8_in_circle Ready")
rospy.spin() # mantain the service open.
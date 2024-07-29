#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import rospkg
from wall_following_pkg.srv import FindWall, FindWallRequest # you import the service message python classes generated from FindWall.srv.
import actionlib
from wall_following_pkg.msg import OdomRecordAction, OdomRecordGoal

def client_record_odom():     
    print("chamou a ação")    
    # Inicializa o cliente de ação para o servidor de gravação de odometria
    odom_client = actionlib.SimpleActionClient('record_odom', OdomRecordAction)
    
    # Espera até que o servidor de ação esteja disponível
    odom_client.wait_for_server()

    
    
    # Inicia a gravação de odometria quando envia um goal
    goal = OdomRecordGoal()  # Cria um objetivo vazio e envia ao servidor de ação
    odom_client.send_goal(goal)
    rospy.loginfo("Odometry recording started")   

    laser_subscriber()
    
    # Aguarda o resultado
    odom_client.wait_for_result()

    # Obtém o resultado do servidor de ação
    result = odom_client.get_result()

    if result:
        rospy.loginfo("Odometry recording finished")
        rospy.loginfo("Odom List: {}".format(result.list_of_odoms))
        rospy.loginfo("Total Distance: {}".format(result.current_total))
    else:
        rospy.logerr("Action did not return a result")
    

def service_client():    
    rospy.wait_for_service('/find_wall') # Wait for the service client /find_wall to be running
    wall_following_client = rospy.ServiceProxy('/find_wall', FindWall) # Create the connection to the service
    wall_following_client_request_object = FindWallRequest() # Create an object of type EmptyRequest

    result = wall_following_client(wall_following_client_request_object) # Send through the connection the path to the trajectory file to be executed
    print(result) # Print the result given by the service called


def laser_callback(data):   # Callback para lidar com os dados recebidos do laser

    l = data.ranges
    distanciaFront = data.ranges[360]
    distanciaRight = data.ranges[180]
    

    '''
    min_distance = min(l)
    min_index = l.index(min_distance)
    print ("O menos distância é: ", min_distance)
    print(f"Feixe com maior distância: {min_index}")
    '''
    
    # Twist para enviar comandos de velocidade
    vel_msg = Twist()
   
    
    if distanciaFront < 0.5:    # obstáculo a frente (virar pa à esquerda)
        print(f"Virando à esquerda")
        vel_msg.linear.x = 0.05        
        vel_msg.angular.z = 1
    elif distanciaRight > 0.3: # vira para à direita (se aproximama da parede)
        print(f"Virando à direita")
        vel_msg.linear.x = 0.1        
        vel_msg.angular.z = - 0.5
    elif distanciaRight < 0.2: # vira para à esquerda (se afasta da parede)
        print(f"Virando à esquerda")
        vel_msg.linear.x = 0.1        
        vel_msg.angular.z =  0.5
    else :                       # anda em linha reta
        print(f"indo reto")
        vel_msg.linear.x = 0.1
        vel_msg.angular.z = 0.0
    
    
    vel_pub.publish(vel_msg)    # Publica a mensagem de velocidade
    

def laser_subscriber():      
    # Cria um subscriber para o tópico '/scan' com a mensagem LaserScan
    rospy.Subscriber('/scan', LaserScan, laser_callback)    
    rospy.spin()    # Mantém o programa funcionando até que seja fechado

if __name__ == '__main__':    
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # Cria um publisher para o tópico '/cmd_vel' com a mensagem Twist    
    try:       
        rospy.init_node('wall_following_node', anonymous=True) # Inicializa o nó ROS            
        service_client()
        client_record_odom()
          
    except rospy.ROSInterruptException:
        pass


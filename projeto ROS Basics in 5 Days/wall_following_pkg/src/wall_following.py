#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import rospkg
from wall_following_pkg.srv import FindWall, FindWallRequest # you import the service message python classes generated from Empty.srv.

import actionlib
from wall_following_pkg.msg import OdomRecordAction, OdomRecordGoal
'''
class WallFollowingNode:
    def __init__(self):
        # Inicializa o nó ROS
        rospy.init_node('wall_following_node')
        
        # Inicializa o cliente de ação para o servidor de gravação de odometria
        self.odom_client = actionlib.SimpleActionClient('record_odom', OdomRecordAction)
        
        # Espera até que o servidor de ação esteja disponível
        self.odom_client.wait_for_server()
        
        # Chama a função para iniciar a gravação de odometria
        self.start_odom_recording()

    def start_odom_recording(self):
        # Cria um objetivo vazio e envia ao servidor de ação
        goal = OdomRecordGoal()
        self.odom_client.send_goal(goal)
        rospy.loginfo("Odometry recording started")

    def wall_following_logic(self):
        # Aqui vai a lógica existente de seguir paredes
        
        # Condição para finalizar a volta (substitua pela condição real)
        if condition_to_finish_lap:  
            # Cancela o objetivo de gravação de odometria
            self.odom_client.cancel_goal()
            
            # Obtém o resultado do servidor de ação
            result = self.odom_client.get_result()
            
            rospy.loginfo("Odometry recording finished")
            rospy.loginfo("Odom List: {}".format(result.list_of_odoms))
            rospy.loginfo("Total Distance: {}".format(result.current_total))

'''

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
    rospy.init_node('wall_following_node', anonymous=True) # Inicializa o nó ROS    
    # Cria um subscriber para o tópico '/scan' com a mensagem LaserScan
    rospy.Subscriber('/scan', LaserScan, laser_callback)    
    rospy.spin()    # Mantém o programa funcionando até que seja fechado

if __name__ == '__main__':    
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # Cria um publisher para o tópico '/cmd_vel' com a mensagem Twist    
    try:   
        # Cria uma instância do nó de seguir paredes
        #node = WallFollowingNode()   
                
        service_client()
        laser_subscriber()  # Inicia o subscriber
    except rospy.ROSInterruptException:
        pass
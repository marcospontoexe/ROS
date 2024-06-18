#!/usr/bin/env python

import rospy
from wall_following_pkg.srv import FindWall, FindWallResponse # you import the service message python classes generated from Empty.srv.
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class FindWallServer:
    def __init__(self):
        rospy.init_node('find_wall_server')
        
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.service = rospy.Service('/find_wall', FindWall, self.handle_find_wall)
        
        self.laser_data = None
        self.distanciaTrig = 0
        self.distanciaFront = 0

    def laser_callback(self, msg):
        self.laser_data = msg.ranges
        self.distanciaTrig = msg.ranges[270]
        self.distanciaFront = msg.ranges[360]


    def handle_find_wall(self, req):
        if self.laser_data is None:
            rospy.loginfo("No laser data received yet")
            return FindWallResponse(wallfound=False)


        # Step 1: Identify the shortest laser ray
        min_distance = min(self.laser_data)
        min_index = self.laser_data.index(min_distance)
        distaAtual =  self.distanciaFront
        # Step 2: Rotate the robot until the front of it is facing the wall
        twist = Twist() 
        if min_index > 360:
            while min_index > 360:  # até que a menor distância seja o índice 360 do laser (frente do robô)
                twist.angular.z = 0.5 
                self.cmd_pub.publish(twist)
                rospy.sleep(0.1)                
                min_distance = min(self.laser_data)
                min_index = self.laser_data.index(min_distance)
                print("menor distÂcia: ", min_distance)
                print(f"índice da menor distância: {min_index}")
        else:
            while min_index < 360:  # até que a menor distância seja o índice 360 do laser (frente do robô)
                twist.angular.z = -0.5 
                self.cmd_pub.publish(twist)
                rospy.sleep(0.1)                
                min_distance = min(self.laser_data)
                min_index = self.laser_data.index(min_distance)
                print("menor distÂcia: ", min_distance)
                print(f"índice da menor distância: {min_index}")
        '''          
        while min_index != 360:    # até que a menor distância seja o índice 360 do laser (frente do robô)
            #print("distanciaFront: ", self.distanciaFront)
            #print(f"distanciaTrig: {self.distanciaTrig}")
            print("menor distÂcia: ", min_distance)
            print(f"índice da menor distância: {min_index}")
            if min_index > 360:
                twist.angular.z = 0.1                
            else:
                twist.angular.z = -0.1            
            self.cmd_pub.publish(twist)
            rospy.sleep(0.1)
            min_distance = min(self.laser_data)
            min_index = self.laser_data.index(min_distance)
            if self.distanciaFront > distaAtual:
                break
            distaAtual =  self.distanciaFront
        '''
        twist.angular.z = 0
        self.cmd_pub.publish(twist)

        # Step 3: Move forward until the front ray is smaller than 30 cm
        while self.distanciaFront > 0.3:
            print("distanciaFront: ", self.distanciaFront)
            twist.linear.x = 0.1
            self.cmd_pub.publish(twist)
            rospy.sleep(0.1)
        twist.linear.x = 0
        self.cmd_pub.publish(twist)

        distaAtual =  self.distanciaTrig
        # Step 4: Rotate until ray number 270 is pointing to the wall
        while min_index > 270:       
            twist.angular.z = 0.5                    
            self.cmd_pub.publish(twist)
            rospy.sleep(0.1)
            min_distance = min(self.laser_data)
            min_index = self.laser_data.index(min_distance)
            print("menor distÂcia: ", min_distance)
            print(f"índice da menor distância: {min_index}") 
        twist.angular.z = 0
        self.cmd_pub.publish(twist)

        # Step 5: Consider the robot ready to start following the wall
        return FindWallResponse(wallfound=True)

if __name__ == '__main__':
    try:
        server = FindWallServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#---------------------------------------------------
'''
#! /usr/bin/env python

import rospy
from wall_following_pkg.srv import FindWall, FindWallResponse # you import the service message python classes generated from Empty.srv.
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

fimFlag = False
inicioFlag = True
min_distance = 0

def laser_callback(data):   # Callback para lidar com os dados recebidos do laser

    l = data.ranges
    distanciaTrig = data.ranges[270]
    distanciaFront = data.ranges[360]
    global inicioFlag
    global fimFlag
    global min_distance

    if inicioFlag:    
        min_distance = min(l)
        min_index = l.index(min_distance)
        print ("a menor distância é: ", min_distance)
        print(f"Feixe com maior distância: {min_index}")
        inicioFlag = False
    
   
   
    print("distanciaFront: ", distanciaFront)
    print(f"distanciaTrig: {distanciaTrig}")
    
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


    while my_pub.get_num_connections() < 1:
        # wait for a connection to publisher
        # you can do whatever you like here or simply do nothing
        rospy.loginfo("Tentando publicar no tópico /cmd_vel")
    
    my_pub.publish(vel_msg)    # Publica a mensagem de velocidade
   

def my_callback(request):
    rospy.loginfo("The Service find_wall has been called")
    global fimFlag
    while fimFlag == False:
        # Cria um subscriber para o tópico '/scan' com a mensagem LaserScan
        rospy.Subscriber('/scan', LaserScan, laser_callback)    
        rospy.spin()    # Mantém o programa funcionando até que seja fechado
        
    rospy.loginfo("Finished service find_wall")
    return FindWallResponse(True) # the service Response class, in this case EmptyResponse

rospy.init_node('service_server_node') 
my_service = rospy.Service('/find_wall', FindWall , my_callback) # create the Service called find_wall with the defined callback
 # Twist para enviar comandos de velocidade
my_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
vel_msg = Twist()

rospy.loginfo("Service /find_wall Ready")
rospy.spin() # mantain the service open.

'''
#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from wall_following_pkg.msg import OdomRecordAction, OdomRecordFeedback, OdomRecordResult


class OdomRecordServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('record_odom', OdomRecordAction, self.execute, False)  # Inicializa o servidor de ação
        self.server.start()  # Inicia o servidor de ação (o servidor começa a aceitar metas (goal).)
        
        self.odom_list = [] # Lista para armazenar as odometrias
        self.total_distance = 0.0   # Variável para armazenar a distância total percorrida
        self.last_position = None   # Variável para armazenar a última posição registrada
        
        # Inscreve-se no tópico de odometria
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
            
    def odom_callback(self, msg): # Callback para lidar com mensagens de odometria
        # Callback para lidar com mensagens de odometria
        position = msg.pose.pose.position
        current_position = Point32(position.x, position.y, 0.0) # Usamos 0.0 porque estamos tratando de 2D
        
        # Se houver uma posição anterior, calcula a distância percorrida
        if self.last_position:
            distance = self.calculate_distance(self.last_position, current_position)
            self.total_distance += distance
        
        # Atualiza a última posição e adiciona a posição atual à lista
        self.last_position = current_position
        self.odom_list.append(current_position)

    def calculate_distance(self, p1, p2):
        # Calcula a distância euclidiana entre duas posições
        return ((p1.x - p2.x)**2 + (p1.y - p2.y)**2)**0.5

    def execute(self, goal):
        # Função principal do servidor de ação
        rate = rospy.Rate(1)  # Define a taxa de execução para 1 Hz (uma vez por segundo)
        
    

        # Loop para fornecer feedback e checar se o objetivo foi cancelado
        while not self.server.is_preempt_requested():
            feedback = OdomRecordFeedback()
            feedback.current_total = self.total_distance
            self.server.publish_feedback(feedback)            
            rate.sleep()
            if self.total_distance > 6:    # quando realiza uma volta completa
                break
        
        # Quando o objetivo é concluído, preenche o resultado com a lista de odometrias e define o estado como bem-sucedido
        result = OdomRecordResult()
        result.list_of_odoms = self.odom_list
        self.server.set_succeeded(result)

if __name__ == '__main__':
    # Inicializa o nó ROS
    rospy.init_node('record_odom_server')
    
    # Cria uma instância do servidor de ação
    server = OdomRecordServer()
    
    # Mantém o nó em execução
    rospy.spin()

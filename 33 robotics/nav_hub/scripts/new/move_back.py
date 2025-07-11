#!/usr/bin/env python3

import rospy
import time
import os
from nav_msgs.msg import Path  # Tipo de mensagem para o plano global
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped  # Tipos de mensagens para posição e pontos
from actionlib_msgs.msg import GoalID  # Tipos de mensagens para status e cancelamento do objetivo
from move_base_msgs.msg import MoveBaseActionGoal
from dynamic_reconfigure.client import Client
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool

class GlobalPlanHandler:
    def __init__(self):
        """
        Caso o robo fique parado por um determinado tempo, essa classe;
        - altera min_vel_x do DWAPlanner para -0.1 
        - cancela a rota global 
        - publica o destino usando o ponto de origem
        - aṕos alguns segundos cancela a rota global 
        - altera min_vel_x do DWAPlanner para 0.0
        - publica o destino usando o ponto de destino
        Quando uma rota global é publicada no tópico /move_base/GlobalPlanner/plan e o robo ainda não esteja navegando até um objetivo, o primeiro ponto dessa rota é armazenado como ponto de origem, e o ultimo ponto como ponto de destino .
        """
        rospy.init_node('move_back_node')  # Inicializa o nó ROS

        # Inicializa variáveis de controle
        self.iniciouRota = False  # Indica se a navegação com a rota inicial foi iniciada
        self.rotaInicial = None  # Armazena a rota inicial recebida
        self.flagNewRoute = False
        self.startTimer = False
        self.reached = False
        self.start_time = 0.0
        self.elapsed_time = 0.0
        self.goal = MoveBaseActionGoal()
        self.origin = MoveBaseActionGoal()
        self.goal.goal.target_pose.header.frame_id = "map"
        self.goal.goal.target_pose.header.seq = 0
        self.origin.goal.target_pose.header.seq = 0
        self.origin.goal.target_pose.header.frame_id = "map"

        self.vel = Twist()

        # Inicializa os publicadores
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1, latch=True)  # Para cancelar a rota atual
        # self.clicked_point_pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=1)  # Para publicar pontos relevantes
        self.cor_pub = rospy.Publisher("/cor", String, queue_size=1, latch=True)  # publica uma cor para o código fitaLed_v2.0 gerenciar
        self.goal_publisher = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=1, latch=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Inscreve-se nos tópicos 
        # rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.global_plan_callback, queue_size=1)  # Recebe planos globais
        rospy.Subscriber("/cmd_vel", Twist, self.velocidade_callback, queue_size=1)
        rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self.goal_callback, queue_size=1)
        rospy.Subscriber("/keyence_pose", PoseWithCovarianceStamped, self.poseCallBack, queue_size=1)  # Para verificar a posição do robô
        rospy.Subscriber("has_reached", Bool, self.reached_callback, queue_size=1)  # publicado pelo main

        self.velocidade = Twist()

    def reached_callback(self, data):
        self.reached = data.data
        if self.reached:            # Verifica se o objetivo foi alcançado, publicado pelo main
            self.flagNewRoute = False
            self.iniciouRota = False
            self.startTimer = False
            # rospy.loginfo("Objetivo alcançado. Variável iniciouRota reiniciada.")

    def poseCallBack(self, data):
        if not self.iniciouRota:
            try:
                # salva o ponto inicial da rota global como origem
                self.origin.goal.target_pose.header.stamp = rospy.Time.now()
                self.origin.goal.target_pose.pose.position.x = data.pose.pose.position.x
                self.origin.goal.target_pose.pose.position.y = data.pose.pose.position.y
                self.origin.goal.target_pose.pose.orientation.z = data.pose.pose.orientation.z
                self.origin.goal.target_pose.pose.orientation.w = data.pose.pose.orientation.w
            except:
                rospy.logwarn("Não foi possível obter a pose inicial do robo!")

    def goal_callback(self, msg):
        if not self.iniciouRota: # Caso a navegação ainda não tenha iniciado, armazena o ponto origem e de destino   
            
            # salva o ponto final da rota global como destino
            self.goal.goal.target_pose.header.stamp = rospy.Time.now()
            self.goal.goal.target_pose.pose.position.x = msg.goal.target_pose.pose.position.x
            self.goal.goal.target_pose.pose.position.y = msg.goal.target_pose.pose.position.y
            self.goal.goal.target_pose.pose.orientation.z = msg.goal.target_pose.pose.orientation.z
            self.goal.goal.target_pose.pose.orientation.w = msg.goal.target_pose.pose.orientation.w

            self.iniciouRota = True

    def velocidade_callback(self, data):
        self.velocidade = data
    
    def min_vel_x(self, value):
        client = Client("/move_base/DWAPlannerROS", timeout=5)
        params = {"min_vel_x": value}
        client.update_configuration(params)

    def footprint(self, value):
        client = Client("/move_base/local_costmap", timeout=5)
        client1 = Client("/move_base/global_costmap", timeout=5)
        params = {"footprint": value}
        params1 = {"footprint": value}
        client.update_configuration(params)
        client1.update_configuration(params1)

    def footprint_padding(self, value):
        client = Client("/move_base/local_costmap", timeout=5)
        client1 = Client("/move_base/global_costmap", timeout=5)
        params = {"footprint_padding": value}
        params1 = {"footprint_padding": value}

        client.update_configuration(params)
        client1.update_configuration(params1)
    
    def move_backwards(self):
        print("iniciou movimento de  ré")
        self.vel.linear.x = -0.1  # Move para trás a 20 cm/s
        self.vel.angular.z = 0.0
        duration = 1.0  # Move para trás por 2.5 segundos (50 cm)
        
        tempo_inicio = time.time()
        # tempo_decorrido = time.time() - tempo_inicio # Calcula o tempo decorrido
        while (time.time() - tempo_inicio) < duration:
            self.cmd_vel_pub.publish(self.vel)
        
        print("finalizou movimento de  ré")

    def processRoute(self):
        if self.flagNewRoute:   # qunado o robo fica parado até atingir o tempo limite do cronometro
            # self.cor_pub.publish("Roxo") 
            print("Iniciando rotina!")

            # Publica uma mensagem GoalID para cancelar o objetivo atual
            self.cancel_pub.publish(GoalID())  
            print("Rota global atual cancelada.")

            self.min_vel_x(-0.1) # altera a velocidade mínima para poder andar de ré
            # self.footprint([[0.1, -0.25], [0.1, 0.25], [-0.40, 0.25], [-0.40, -0.25]]) # caso o robo esteja encostado na inflação
            self.footprint_padding(-0.2)
            # os.system('rosparam set /move_base/DWAPlannerROS/min_vel_x -0.1')      

            # self.move_backwards()       # caso o robo esteja encostado na inflação

            # Publica o destino usando o ponto de origem
            self.goal_publisher.publish(self.origin)

            time.sleep(5)
            # self.footprint([[0.16, -0.25], [0.16, 0.25], [-0.40, 0.25], [-0.40, -0.25]])
            self.footprint_padding(0.0)

            time.sleep(2)

            # Publica uma mensagem GoalID para cancelar o objetivo atual
            self.cancel_pub.publish(GoalID())  
            print("Rota global atual cancelada.")

            os.system('rosservice call /move_base/clear_costmaps "{}"')     # limpa o mapa
            self.min_vel_x(0.0)   # altera a velocidade mínima para não andar de ré
            # os.system('rosparam set /move_base/DWAPlannerROS/min_vel_x 0.0')      # altera a velocidade mínima para não andar de ré

            # Publica o destino usando o ponto de destino
            self.goal_publisher.publish(self.goal)

            # #caso nãe seja possível andar de ré, e seja possível sair apenas se girar no proprio eixo
            # time.sleep(5)
            # # self.footprint([[0.16, -0.25], [0.16, 0.25], [-0.40, 0.25], [-0.40, -0.25]])
            # self.footprint_padding(0.0)

            self.flagNewRoute = False

        elif self.iniciouRota:  #caso o robo esteja se deslocando para o objetivo
            if abs(self.velocidade.linear.x) <= 0.0062 and abs(self.velocidade.angular.z) <= 0.037: #  robo parado
                if not self.startTimer:     # inicia um cronometro 
                    self.startTimer = True
                    self.start_time = time.time()    # Marca o tempo inicial
                    self.elapsed_time = 0.0
                    # print("Timer iniciado.")
                
                self.elapsed_time = time.time() - self.start_time # Calcula o tempo decorrido
                if self.elapsed_time >= 5:     # chama nova rota
                    self.flagNewRoute = True
                    self.startTimer = False
                    # print("Tempo estourado.")
            
            else:   #robo em movimento
                if self.startTimer: # para reiniciar o cronometro
                    self.startTimer = False
                    # print("Timer parado e reiniciado.")


if __name__ == '__main__':
    objt = GlobalPlanHandler()
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        objt.processRoute()
        rate.sleep()



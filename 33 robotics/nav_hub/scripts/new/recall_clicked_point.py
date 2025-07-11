#!/usr/bin/env python3

import rospy
import time
import os
import math
from nav_msgs.msg import Path  # Tipo de mensagem para o plano global
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped, Twist  # Tipos de mensagens para posição e pontos
from actionlib_msgs.msg import GoalID  # Tipos de mensagens para status e cancelamento do objetivo
from std_msgs.msg import String, Bool

class GlobalPlanHandler:
    def __init__(self):
        """
        Essa classe cancela a rota global e publica novamente caso o robo fique parado mais que self.wait_time, evitando que o move base aborte a navegação.
        Quando uma rota ponto a ponto é publicada no tópico /new_clicked_point, os pontos são armazenados em "self.pontos", e esses mesmos pontos são publicados no tópico /clicked_point.
        Quando o robo fica parado mais que self.wait_time, é verificado em que posição o robo se encontra na rota global (/move_base/WaypointGlobalPlanner/global_plan), 
        publicando somente os pontos contidos em "self.pontos" que estão a partir da posição do robo até o destino.
        Essa classe também publica somente os pontos contidos em "self.pontos" que estão a partir da posição do robo até o destino caso o planejador de rotas
        locais peça ao move base para republicar a rota global. 
        """
        rospy.init_node('recall_cliked_point_node')  # Inicializa o nó ROS

        # Inicializa variáveis de controle
        self.velocidade = Twist()
        self.pontos = []  # Lista de pontos publicados no tópico new_clicked_point
        self.LastIndexRout = 0
        self.flagNewRoute = False
        self.started = False
        self.iniciouRota = False  # Indica se a navegação com a rota inicial foi iniciada
        self.rotaInicial = None  # Armazena a rota inicial recebida
        self.startTimer = True
        self.start_time = 0.0
        self.elapsed_time = 0.0
        self.wait_time = 5      # tempo em segundos que o robo ficará parado sem republicar uma nova rota
        
        
        self.epsilon_path = '/move_base/WaypointGlobalPlanner/epsilon'                # Caminho completo para o parâmetro epsilon
        self.waypoints_per_meter_path = '/move_base/WaypointGlobalPlanner/waypoints_per_meter'    # Caminho completo para o parâmetro waypoints_per_meter

        # Inicializa os publicadores
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)  # Para cancelar a rota atual
        self.clicked_point_pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=50)  # Para publicar pontos relevantes
        self.cor_pub = rospy.Publisher("/cor", String, queue_size=1)  # publica uma cor para o código fitaLed_v2.0 gerenciar


        # Inscreve-se nos tópicos 
        rospy.Subscriber("has_reached", Bool, self.reached_callback, queue_size=1)  # publicado pelo main
        rospy.Subscriber('/move_base/WaypointGlobalPlanner/global_plan', Path, self.new_global_plan_callback, queue_size=1)  # Recebe planos globais
        rospy.Subscriber('/new_clicked_point', PointStamped, self.new_clicked_point_callback)  # Recebe pontos publicados pelo goal_manipulation_ponto_ponto
        rospy.Subscriber("/cmd_vel", Twist, self.velocidade_callback, queue_size=1)
        rospy.Subscriber("/keyence_pose", PoseWithCovarianceStamped, self.pose_callBack, queue_size=1) # Para verificar a posição do robô
    
    def reached_callback(self, data):
        self.reached = data.data
        if not self.started:
            # Obtém o valor do parâmetro waypoints_per_meter
            self.dist_epsilon = rospy.get_param(self.epsilon_path)
            self.pontos_por_metro = rospy.get_param(self.waypoints_per_meter_path)
            self.dist_entre_pontos = 1.0 / self.pontos_por_metro

            # print(f"self.dist_epsilon: {self.dist_epsilon}")
            # print(f"self.pontos_por_metro: {self.pontos_por_metro}")
            # print(f"dist_entre_pontos: {self.dist_entre_pontos}")
            # print(f"larguraDoRobo: {self.larguraDoRobo}")
            # print(f"dist_minima: {self.dist_minima}")
            # print(f"passo_globalPath: {self.passo_globalPath}")
            self.started = True 

        if self.reached:            # publicado pelo main
            self.flagNewRoute = False
            self.iniciouRota = False
            self.pontos = []  # Limpa a lista de pontos clicados
            self.startTimer = False
        

    def new_global_plan_callback(self, global_plan):
        """
        Callback para processar uma nova rota global publicada no tópico 'new_global_plan'.
        """
        # print("new_global_plan_callback")
        if not self.iniciouRota:
            # Caso a navegação ainda não tenha iniciado, armazena a rota inicial 
            self.rotaInicial = global_plan
            self.LastIndexRout = 0
            self.iniciouRota = True
            # rospy.loginfo("Rota inicial armazenada.")
        else: 
            # print("chamou /move_base/WaypointGlobalPlanner/global_plan") 
            self.flagNewRoute = True


    def new_clicked_point_callback(self, msg):
        """
        Callback para armazenar pontos clicados no Rviz quando a navegação não está em andamento.
        """
        self.pontos.append(msg.point)  # Adiciona os pontos publicados no tópico new_clicked_point
        # rospy.loginfo(f"Ponto armazenado: {msg.point}")
        # Publica os pontos atualizados

        # Publica os pontos clicados que fazem parte da rota inicial
        self.clicked_point_pub.publish(msg)
        time.sleep(0.1)
    
    def velocidade_callback(self, data):
        self.velocidade = data

    def pose_callBack(self, data):
        #Função para verificar a posição atual do robô .
        self.robot_x = data.pose.pose.position.x
        self.robot_y = data.pose.pose.position.y


    # def publish_points(self, robot_pose, pontos):
    def publish_points(self, pontos):
        """
        Publica a posição atual do robô e os pontos clicados que fazem parte da rota inicial.
        """
        # Publica a posição atual do robô no tópico '/clicked_point'
        # current_robot_point = PointStamped()
        # current_robot_point.header.frame_id = "map"
        # current_robot_point.point.x = robot_pose.pose.pose.position.x
        # current_robot_point.point.y = robot_pose.pose.pose.position.y
        # current_robot_point.point.z = 0.0
        # self.clicked_point_pub.publish(current_robot_point)

        os.system('rosservice call /move_base/clear_costmaps "{}"')
        # rospy.loginfo(f"{len(pontos)} pontos publicados.\n")

        # Publica os pontos clicados que fazem parte da rota inicial
        for ponto in pontos:
            point_msg = PointStamped()
            point_msg.header.frame_id = "map"
            point_msg.point = ponto
            # print(ponto)
            self.clicked_point_pub.publish(point_msg)
            time.sleep(0.1)
            # rospy.loginfo(f"Ponto relevante publicado: {ponto}")
            # print("ponto publicado")
    
    def distanciaEuclidiana(self, pontoA, pontoB):
        '''Calcula a distancia euclidiana em metros entre dois pontos do mapa global.

        :param pontoA: lista contendo os pontos x e y de uma posição no mapa global.
        :param pontoB: lista contendo os pontos x e y de uma posição no mapa global.
        :return distancia: Retorna a distancia euclidiana em metros entre os dois pontos.
        '''
        # Calcula a distância euclidiana entre os pontos
        distancia = math.sqrt((pontoA[0] - pontoB[0])**2 + (pontoA[1] - pontoB[1])**2)

        return distancia
    
    def positionAtRout(self):
        """
        Retorna o valor do índice onde ocorreu a menor distancia entre os pontos da rota global e o robo.
        Manerira de fazer um trimming na rota global.
        """
     
        # Determina o índice onde ocorreu a menor distancia entre os pontos da rota inicial e o robo
        indice = 0
        dist_minima_robot_rout = float('inf')  # Inicializa a menor distância como infinito

        # comprimentoRota = len(self.rotaInicial.poses)
        # print(f"Comprimento da rotaInicial: {comprimentoRota}")
        
        for i, pose in enumerate(self.rotaInicial.poses[self.LastIndexRout:]): #verifica qual é o índice onde ocoreu a menor distancia entre o robo e a rota inicial
            dist = self.distanciaEuclidiana([self.robot_x, self.robot_y], [pose.pose.position.x, pose.pose.position.y])    # distancia entre o robo e o ponto na rota global
            if dist < dist_minima_robot_rout:  # ao percorrer a rota global, cada ponto da rota esta ficando mais próximo do robo
                dist_minima_robot_rout = dist
                indice = i + self.LastIndexRout # Atualiza a menor distância e o índice correspondente
                
        # print(f"dist_minima: {dist_minima_robot_rout}")
        # print(f"indice: {indice}")
        return indice
    
    def processRoute(self):
        if self.flagNewRoute:   # qunado o robo fica para até atingir o tempo limite self.wait_time, ou quando a rota global do waypoints é chamada novamente. Cancela a rota, e traça uma nova rota
            # self.cor_pub.publish("Roxo") 
            
            self.LastIndexRout = self.positionAtRout()  # recebe qual é o índice da rota global em que o robo está mais pŕoximo

            comprimentoRota = len(self.rotaInicial.poses)

            # Verifica quais pontos publicados no tópico new_clicked_point estão presentes na rota inicial 
            pontos_na_rota = []
            # insere na lista pontos_na_rota, todos os pontos da lista "pontos" (rota clicked_point) que estejam a rota inicial trimada (lista rotaInicial)
            for ponto in self.pontos:       # itera sobre todos os pontos publicados no tópico new_clicked_point
                if (self.LastIndexRout + self.pontos_por_metro) < comprimentoRota - self.pontos_por_metro:  # o primeiro ponto será criado a um metro do robo, para que o robo não gire no prórpio eixo caso não esteja exatamente em cima da rota global
                    for pose in self.rotaInicial.poses[self.LastIndexRout + self.pontos_por_metro:]:
                        # verifica a menor distancia, meneira de "trimar" a rota clickd_point
                        dist = self.distanciaEuclidiana([ponto.x, ponto.y], [pose.pose.position.x, pose.pose.position.y])  # distancia entre o clicked _point e o ponto da rota inicial
                        if dist < self.dist_entre_pontos:  # verifica se o clickd_point está contido na rota inicial, "trimando" a rota clicked_point
                            pontos_na_rota.append(ponto)
                            break
                else:       # O robo está a menos de um metro do objetivo, o primeiro ponto deve ser o self.LastIndexRout da rota inicial
                    for pose in self.rotaInicial.poses[self.LastIndexRout:]:    
                        dist = self.distanciaEuclidiana([ponto.x, ponto.y], [pose.pose.position.x, pose.pose.position.y])  # distancia entre o clicked _point e o ponto da rota inicial
                        if dist < self.dist_entre_pontos:  # verifica se o clickd_point está contido na rota inicial, "trimando" a rota clicked_point
                            pontos_na_rota.append(ponto)
                            break
                    

            novo_ponto = PointStamped()
            novo_ponto.header.frame_id = "map"
            novo_ponto.point.z = 0.0

            # print(f"Quantidade de pontos_na_rota: {len(pontos_na_rota)}")
            # A rota clicked_point deve conter pelo menos os dois ultimos pontos
            if len(pontos_na_rota) == 0:   
                penultimo_ponto_rota_inicial = self.rotaInicial.poses[-2].pose.position # Obtém o penúltimo ponto da rota inicial
                ultimo_ponto_rota_inicial = self.rotaInicial.poses[-1].pose.position # Obtém o último ponto da rota inicial
                # Cria dois pontos a serem inseridos
                novo_ponto.point.x = penultimo_ponto_rota_inicial.x
                novo_ponto.point.y = penultimo_ponto_rota_inicial.y
                
                # Insere o penúltimo índice de pontos_na_rota
                pontos_na_rota.append(novo_ponto.point)
                # Insere o último índice de pontos_na_rota
                novo_ponto.point.x = ultimo_ponto_rota_inicial.x
                novo_ponto.point.y = ultimo_ponto_rota_inicial.y
                pontos_na_rota.append(novo_ponto.point)

            elif len(pontos_na_rota) == 1:  # quando apenas o ultimo ponto da rota cliked_point foi inserido na nota rota cliked_point
                penultimo_ponto_rota_inicial = self.rotaInicial.poses[-2].pose.position # Obtém o penúltimo ponto da rota inicial
                # Cria um novo ponto a ser inserido
                novo_ponto.point.x = penultimo_ponto_rota_inicial.x
                novo_ponto.point.y = penultimo_ponto_rota_inicial.y
                

                # Insere o novo ponto no inicio de pontos_na_rota
                pontos_na_rota.insert(0, novo_ponto.point)

            else:    # Verifica a distância entre os dois últimos pontos de pontos_na_rota
                ultimo_ponto = pontos_na_rota[-1]
                penultimo_ponto = pontos_na_rota[-2]
                # print(f"ultimo_ponto: {ultimo_ponto}")
                # print(f"penultimo_ponto: {penultimo_ponto}")

                # Calcula a distância entre os dois últimos pontos
                dist_entre_pontos = self.distanciaEuclidiana([ultimo_ponto.x, ultimo_ponto.y ], [penultimo_ponto.x, penultimo_ponto.y])
                # print(f"Distancia entre os dois ultimos pontos: {dist_entre_pontos}")
                if dist_entre_pontos >= self.dist_epsilon:   #se a distancia entre os dois ultimos pontos for maior que o parametro 'epsilon' do waypoints_global_planner
                    penultimo_ponto_rota_inicial = self.rotaInicial.poses[-2].pose.position # Obtém o penúltimo ponto da rota inicial

                    # Cria um novo ponto a ser inserido
                    novo_ponto.point.x = penultimo_ponto_rota_inicial.x
                    novo_ponto.point.y = penultimo_ponto_rota_inicial.y
                
                    # print(f"pontos_na_rota antes da inserção: {pontos_na_rota}")
                    # Insere o novo ponto no inicio de pontos_na_rota
                    pontos_na_rota.insert(0, novo_ponto.point)
                    # print(f"pontos_na_rota depois da inserção: {pontos_na_rota}")

                    # rospy.loginfo(f"Novo ponto inserido na penultima posição!")
                
                inicioRota = PointStamped()
                inicioRota.header.frame_id = "map"
                inicioRota.point.z = 0.0
                
                # O primeiro ponto a ser criado precisa pertencer a rota inicial, para mantar a originalidade da rota
                # cria o primeiro ponto na nova rota a um metro do robo, para que o robo não gire no prórpio eixo caso não esteja exatamente em cima da rota global
                if (self.LastIndexRout + self.pontos_por_metro) < comprimentoRota - self.pontos_por_metro:
                    # o primeiro ponto será criado a um metro do robo, para que o robo não gire no prórpio eixo caso não esteja exatamente em cima da rota global
                    novo_ponto.point.x = self.rotaInicial.poses[self.LastIndexRout + self.pontos_por_metro].pose.position.x
                    novo_ponto.point.y = self.rotaInicial.poses[self.LastIndexRout + self.pontos_por_metro].pose.position.y
                    #verifica a distancia dos pontos onde ocorreu a menor distancia entre robo e rota global, e o primeiro ponto a ser publicado
                    distanciaEtrePontosIniciais = self.distanciaEuclidiana([novo_ponto.point.x, novo_ponto.point.y ], [pontos_na_rota[0].x, pontos_na_rota[0].y])
                    if distanciaEtrePontosIniciais > self.dist_epsilon:   # se 'distanciaEtrePontosIniciais' for maior que o parametro 'epsilon' adiciona o ponto em 'pontos_na_rota
                        # print("ponto insirido no inicio da rota")
                        pontos_na_rota.insert(0, novo_ponto.point)

            # Publica uma mensagem GoalID para cancelar o objetivo atual
            self.cancel_pub.publish(GoalID())  
            # rospy.loginfo("Rota global atual cancelada.")

            # Publica os pontos atualizados
            self.publish_points(pontos_na_rota)
            time.sleep(3)

            # Publica a posição atual e os pontos clicados relevantes
            # self.publish_points(robot_pose, pontos_na_rota)
            
            self.flagNewRoute = False

        elif self.iniciouRota:  #caso o robo esteja se deslocando para o objetivo
            if abs(self.velocidade.linear.x) <= 0.0062 and abs(self.velocidade.angular.z) <= 0.037: #  robo parado
                if not self.startTimer:     # inicia um cronometro 
                    self.startTimer = True
                    self.start_time = time.time()    # Marca o tempo inicial
                    self.elapsed_time = 0.0
                    # print("Timer iniciado.")
                
                self.elapsed_time = time.time() - self.start_time # Calcula o tempo decorrido
                if self.elapsed_time >= self.wait_time:     # chama nova rota quando o cronometro estoura o tempo de self.wait_time:
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



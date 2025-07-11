#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, Twist, Vector3, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path  # Tipo de mensagem para o plano global
import time
import math
import numpy as np
from std_msgs.msg import Bool

class Stop_Obstacle:
    """
    Essa classe mantem o robo parado caso as coordenadas x e y de obstáculos detectados pelo scan coincidem com as coordenadas x e y da rota global
    Os feixes do scan que estejam dentro da máscara retangular (safety zone), serão convertidos para as
    coordnadas X e Y do mapa global. Caso a distancia entre esses pontos e os ponto da rotoa global seja nenor do que self.dist_minima, 
    significa que o obstáculo está muito próximo da rota global e causará uma colisão com o robo.
    Quando uma colisão for detectaca, será enviado velocidade zero para o tópcio /cmd_vel, até que o caminho esteja limpo.
    """
    def __init__(self):
        self.velocity = Twist()
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0

        self.lastVel = Twist()
        self.lastVel.linear.x = 0
        self.lastVel.angular.z = 0
        
        # TF para transformar coordenadas locais em globais
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.started = False
        self.rotaInicial = None  # Armazena a rota inicial recebida
        self.stopDistance = 1.0 # Feixes do scan que sejam maior do que essa distancia seram descartados.
        self.iniciouRota = False
        self.flagIniciouScan = False
        self.LastIndexRout = 0
        self.radianoIncremento = 0.0174533  # 1 grau em radiano
        self.larguraDoRobo = 0.6        #em metros
        self.dist_minima = (self.larguraDoRobo / 2.0) + 0.05   # distância mínima entre osbstáculos e a rota global, para fazer o robo ficar parado
        self.passo_globalPath = self.dist_minima / 2.0 # passo de 15 centimetros na rota global
        
        # campo de visão
        self.anguloDireita = 0   #45                
        self.anguloEsquerda = 180  #135

        # Caminho completo para o parâmetro waypoints_per_meter
        self.param_path = '/move_base/WaypointGlobalPlanner/waypoints_per_meter'

        # publicadores
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        #subscritores
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber("/cmd_vel_stop_obstacle", Twist, self.vel_callback, queue_size=1) # recebe a velocidade do move base
        rospy.Subscriber('/move_base/WaypointGlobalPlanner/global_plan', Path, self.new_global_plan_callback, queue_size=1)  # Recebe planos globais
        # rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.new_global_plan_callback)  # Recebe planos globais
        rospy.Subscriber("/keyence_pose", PoseWithCovarianceStamped, self.pose_callBack, queue_size=1) # Para verificar a posição do robô
        rospy.Subscriber("has_reached", Bool, self.reached_callback, queue_size=1)  # publicado pelo main

    def scan_callback(self, msg):
        self.laser_ranges = np.array(msg.ranges)
        # self.laser_ranges = msg.ranges
        
        if not self.flagIniciouScan:
            self.flagIniciouScan = True
            self.angle_min = msg.angle_min
            self.angle_increment = msg.angle_increment
            self.laser_frame_id = msg.header.frame_id

    def pose_callBack(self, data):
        #Função para verificar a posição atual do robô .
        self.robot_x = data.pose.pose.position.x
        self.robot_y = data.pose.pose.position.y

    def vel_callback(self, data):
        self.velocity = data

    def reached_callback(self, data):
        self.reached = data.data
        if not self.started:
            # Obtém o valor do parâmetro waypoints_per_meter
            self.dist_entre_pontos = 1.0 / rospy.get_param(self.param_path)
            # print(f"dist_entre_pontos: {self.dist_entre_pontos}")
            # print(f"larguraDoRobo: {self.larguraDoRobo}")
            # print(f"dist_minima: {self.dist_minima}")
            # print(f"passo_globalPath: {self.passo_globalPath}")
            self.started = True 

        if self.reached:            # publicado pelo main
            self.iniciouRota = False
            # rospy.loginfo("Objetivo alcançado. Variável iniciouRota reiniciada.")
        
    def new_global_plan_callback(self, global_plan):
        """
        Callback para processar uma nova rota global publicada no tópico 'new_global_plan'.
        """
        if not self.iniciouRota:
            # Caso a navegação ainda não tenha iniciado, armazena a rota inicial 
            self.rotaInicial = global_plan
            self.LastIndexRout = 0
            # print(f"self.LastIndexRout: {self.LastIndexRout}")
            self.iniciouRota = True
        
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
    
    def get_front_obstacles(self):
        """Filtra os scan que esteja entre os angulos definidos, criando uma máscara retangular (safety zone) e que tenha distancia menor do que self.stopDistance.
        
        return: retorna as coordnadas X e Y dos feixes do scan filtrados em relação ao frame do scan
        """
        if self.laser_ranges is None:
            return []
        
        indices = np.arange(len(self.laser_ranges))     #cria um arraylist de 0 a 750
        angles = self.angle_min + indices * self.angle_increment    # cria um arraylist contendo qual é o grau em rad de cada feixe do scan
        # print(f"indices: {indices}")
        # print(f"angles: {angles}")

        #Safety zone cônico
        # cria uma máscara para filtrar os feixes de scan que estejam entre os angulos definidos 
        front_mask = (angles > self.radianoIncremento*self.anguloDireita) & (angles < self.radianoIncremento*self.anguloEsquerda)  
        front_ranges = self.laser_ranges[front_mask]
        front_angles = angles[front_mask]
        # print(f"front_mask: {front_mask}")
        # print(f"front_ranges: {front_ranges}")
        # print(f"front_angles: {front_angles}")        

        # Remove leituras inválidas (infinito ou NaN)
        valid_mask = np.isfinite(front_ranges)
        front_ranges = front_ranges[valid_mask]
        front_angles = front_angles[valid_mask]
        # print(f"valid_mask: {valid_mask}")
        # print(f"front_ranges: {front_ranges}")
        # print(f"front_angles: {front_angles}")

        # cria uma máscara para filtrar os feixes de scan são menor do que self.stopDistance
        distance_mask = front_ranges < self.stopDistance
        front_ranges = front_ranges[distance_mask]
        front_angles = front_angles[distance_mask]
        # print(f"distance_mask: {distance_mask}")
        # print(f"front_ranges: {front_ranges}")
        # print(f"front_angles: {front_angles}")


        
        #transformam medições do LIDAR (distâncias e ângulos) em coordenadas cartesianas (x,y) em relação ao lidar. (0° é o eixo +X, 135° é a frente do lidar e é o eixo +Y, 180° é o eixo -X, traseira do lidar é o eixo -Y)
        x_coords = front_ranges * np.cos(front_angles)
        y_coords = front_ranges * np.sin(front_angles)

        # print(f"x_coords: {x_coords}")
        # print(f"y_coords: {y_coords}")

         # Aplica uma máscara retangular, de 60X200 cm. Safety zone retangular
        rect_mask = (x_coords >= ((self.dist_minima)*(-1)) ) & (x_coords <= ((self.dist_minima)*(1)) ) & (y_coords >= 0.0) & (y_coords <= self.stopDistance)

        x_coords = x_coords[rect_mask]
        y_coords = y_coords[rect_mask]

        # print(f"x_coords: {x_coords}")
        # print(f"y_coords: {y_coords}")

        return list(zip(x_coords, y_coords))
    
    def lidar_to_global(self, lidar_x, lidar_y):
        """Transforma coordenadas x e y do lidar para o sistema global

        :param local_x: Coordenada x do lidar.
        :param local_y: Coordenada y do lidar.
        :return world_x: Coordenada x no mapa global
        :return world_y: Coordenada y no mapa global
        """
        try:
            '''
            "trans" conterá uma estrutura com posição e orientação para converter coordenadas locais em globais
            '''
            trans = self.tf_buffer.lookup_transform("map", self.laser_frame_id, rospy.Time(0), rospy.Duration(1.0))
            '''
            "map" → Frame de destino (global).
            self.laser_frame_id → Frame de origem (do lidar).
            rospy.Time(0) → Pede a transformação mais recente disponível.
            rospy.Duration(1.0) → espera até 1 segundo para a transformação estar disponível.
            '''
            point_local = PointStamped()
            point_local.header.frame_id = self.laser_frame_id
            point_local.header.stamp = rospy.Time(0)
            point_local.point.x = lidar_x
            point_local.point.y = lidar_y
            point_local.point.z = 0.0

            point_global = tf2_geometry_msgs.do_transform_point(point_local, trans)
            return point_global.point.x, point_global.point.y
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Falha ao transformar coordenadas do scan para o mapa global.")
            return None, None

    def distanciaEuclidiana(self, pontoA, pontoB):
        '''Calcula a distancia euclidiana em metros entre dois pontos do mapa global.

        :param pontoA: lista contendo os pontos x e y de uma posição no mapa global.
        :param pontoB: lista contendo os pontos x e y de uma posição no mapa global.
        :return distancia: Retorna a distancia euclidiana em metros entre os dois pontos.
        '''
        # Calcula a distância euclidiana entre os pontos
        distancia = math.sqrt((pontoA[0] - pontoB[0])**2 + (pontoA[1] - pontoB[1])**2)

        return distancia

    def comparePosition(self):
        """Obtém as coordenadas dos obstáculos em relação ao frame do scan, converte para coordenadas do mapa global, e compara com as coordenadas da rota global
        
        return: retorna True caso as coordenadas da rota global e do scan coincidam
        """
        obstacles = self.get_front_obstacles()  # recebe as coordnadas X e Y de cada obstáculo do scan filtrado, em relação ao frame do scan
        # print(f"    tamanho: {len(obstacles)}")
        coordenadasGlobais = []
        #converte os obstáculos encontrado para coordnadas do mapa global
        for pos, obs in enumerate(obstacles):
            x, y = self.lidar_to_global(obs[0],obs[1])
            if x is not None and y is not None:
                coordenadasGlobais.append((x, y)) 

        self.LastIndexRout = self.positionAtRout()  # recebe qual é o índice da rota global em que o robo está mais pŕoximo
        # contTest = 0
        for cont, coord in enumerate(coordenadasGlobais): # itera sobre todas as coordnadas X e Y dos obstáculos filtrados
            # print(f"cont: {cont}")
            # contTest += 1
            for i, pose in enumerate(self.rotaInicial.poses[self.LastIndexRout:]): #verifica se a distancia da coordenada da rota global passa perto da coordenada do mapa local
                # print(f"iteração: {i % int(self.passo_globalPath/self.dist_entre_pontos)}")
                if (i % round(self.passo_globalPath/self.dist_entre_pontos)) == 0:     # a cada 3 iteraçoes (15 centimetros)
                    # print(f"I           : {i}")
                    dist = self.distanciaEuclidiana(coord, [pose.pose.position.x, pose.pose.position.y])    # distancia entre o obstáculo e o ponto na rota global
                    if dist < self.dist_minima: 
                        print(f"Obstáculo detectado na rota: {coord[0]} | {coord[1]}")
                        return True

                if i > int(self.stopDistance/self.dist_entre_pontos):      # não faz comparação com os pontos da rota global que estão acima do da distancia self.stopDistance
                # if i > 100:
                    # print("\n")
                    break
        # print(f"contTest: {contTest}")
        # print("\n")
        return False
    
    def smooth_transition(self, vel_inicial, vel_final, duration):
        """
        Faz uma transição linear de velocidade inicial para velocidade final em 'duration' segundos,
        publicando em 'pub' a cada 1/rate_hz segundos.

        :param vel_inicial: geometry_msgs.msg.Twist inicial
        :param vel_final: geometry_msgs.msg.Twist final
        :param duration: tempo total da transição (segundos)
        """

        freq_pub = 600   # frequência de publicação

        # Número de passos
        steps = round(duration * freq_pub)
        if steps < 1:
            # Se duração for muito curta, publica diretamente vel_final
            pub.publish(vel_final)
            return

        # Diferenças totais
        delta_lin_x = vel_final.linear.x    - vel_inicial.linear.x
        # delta_lin_y = vel_final.linear.y    - vel_inicial.linear.y
        # delta_lin_z = vel_final.linear.z    - vel_inicial.linear.z
        # delta_ang_x = vel_final.angular.x   - vel_inicial.angular.x
        # delta_ang_y = vel_final.angular.y   - vel_inicial.angular.y
        delta_ang_z = vel_final.angular.z   - vel_inicial.angular.z

        # print(f"vel_inicial.linear.x; {vel_inicial.linear.x}")
        # print(f"vel_inicial.angular.z; {vel_inicial.angular.z}")
        # print(f"vel_final.linear.x; {vel_final.linear.x}")
        # print(f"vel_final.angular.z; {vel_final.angular.z}")
        # print(f"delta_lin_x; {delta_lin_x}")
        # print(f"delta_ang_z; {delta_ang_z}")

        twist = Twist()

        for i in range(1, steps + 1):
            alpha = float(i) / steps  # fração de progresso de 0 (início) a 1 (fim)

            #interpolação quadrática desacelerada (rápida no início, lenta no fim) 
            interpolation = (1 - (1 - alpha)**2) 

            # print(f"smoother_alpha; {smoother_alpha}")
            
            # Interpolação linear em cada componente
            twist.linear.x    = vel_inicial.linear.x  + delta_lin_x * interpolation
            # twist.linear.y    = vel_inicial.linear.y  + delta_lin_y * interpolation
            # twist.linear.z    = vel_inicial.linear.z  + delta_lin_z * interpolation
            # twist.angular.x   = vel_inicial.angular.x + delta_ang_x * interpolation
            # twist.angular.y   = vel_inicial.angular.y + delta_ang_y * interpolation
            twist.angular.z   = vel_inicial.angular.z + delta_ang_z * interpolation

            # print(f"twist.linear.x: {twist.linear.x}")
            # print(f"twist.angular.z: {twist.angular.z}")
            # print("\n")
            # No último passo, forçar exatamente o valor final
            if twist.linear.x < 0.004 and twist.angular.z < 0.01:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                return twist

            self.cmd_vel_pub.publish(twist)
            time.sleep(1/freq_pub)

        return twist
    
    def run(self): 
        if self.iniciouRota:
            # obstaculo = self.comparePosition()
            # print(f"Obstáculo: {obstaculo}")
            
                
            while self.comparePosition() and not rospy.is_shutdown():
                # obstaculo = self.comparePosition()
                # print(f"while")
              
                if self.lastVel.linear.x > 0.0:
                    # para controlar transissões brutas
                    self.lastVel = self.smooth_transition(self.lastVel, Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)), 1)

                # self.cmd_vel_pub.publish(Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)))

               
                # time.sleep(0.1)
            self.lastVel = self.velocity
            self.cmd_vel_pub.publish(self.velocity)
            # print(f"publicou velocidade")
            

if __name__ == '__main__':
    rospy.init_node('stop_obstacle_WayPoint_node', anonymous=True)
    processor = Stop_Obstacle()
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        processor.run()
        rate.sleep()

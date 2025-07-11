#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, Twist, Vector3, PoseWithCovarianceStamped
from nav_msgs.msg import Path  # Tipo de mensagem para o plano global
from move_base_msgs.msg import MoveBaseActionResult # para monitorar o tópico /movebase/result
import time


class LocalCostmapProcessor:
    """
    Essa classe mantem o robo parado caso as coordenadas x e y do mapa local coincidem com as coordenadas xe y da rota global

    """
    def __init__(self):
        # publicadores
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        #subscritores
        rospy.Subscriber("/cmd_vel_stop_obstacle", Twist, self.vel_callback, queue_size=1) # recebe a velocidade do move base
        rospy.Subscriber('/move_base/WaypointGlobalPlanner/global_plan', Path, self.new_global_plan_callback, queue_size=1)  # Recebe planos globais
        # rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.new_global_plan_callback)  # Recebe planos globais
        rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.costmap_callback, queue_size=1)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback, queue_size=1)  # Monitora o status da navegação
        rospy.Subscriber("/keyence_pose", PoseWithCovarianceStamped, self.pose_callBack, queue_size=1) # Para verificar a posição do robô

        self.velocity = Twist()
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0
        
        # TF para transformar coordenadas locais em globais
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.costmap = None
        self.rotaInicial = None  # Armazena a rota inicial recebida
        self.dist_minima = 0.2
        self.iniciouRota = False
        self.cont = 0
        self.LastIndexRout = 0

        
    
    def costmap_callback(self, msg):
        """Recebe o costmap local e armazena os parâmetros"""
        self.costmap = msg.data
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.width = msg.info.width
        self.height = msg.info.height
        self.frame_id = msg.header.frame_id
    

    def pose_callBack(self, data):
        #Função para verificar a posição atual do robô .
        self.robot_x = data.pose.pose.position.x
        self.robot_y = data.pose.pose.position.y
    
    def result_callback(self, msg):
        """
        Callback para processar o status do move_base. Reseta as variáveis de controle ao atingir o objetivo.
        """        
        # print(f"result_callback do clicked: {msg.status.status}")
        if msg.status.status == 3:  # Verifica se o objetivo foi alcançado
            self.iniciouRota = False
            # rospy.loginfo("Objetivo alcançado. Variável iniciouRota reiniciada.")

    def vel_callback(self, data):
        self.velocity = data
        
    def new_global_plan_callback(self, global_plan):
        """
        Callback para processar uma nova rota global publicada no tópico 'new_global_plan'.
        """
        if not self.iniciouRota:
            # Caso a navegação ainda não tenha iniciado, armazena a rota inicial 
            self.rotaInicial = global_plan
            self.iniciouRota = True
            # rospy.loginfo("Rota inicial armazenada.")
        
            
    def costmap_to_local(self, costmap_x, costmap_y):
        """Converte coordenadas do costmap para coordenadas do mapa local
        
        :param costmap_x: Representa o índice das colunas.
        :param costmap_y: Representa o índice das linhas.
        :return world_x: Coordenada x no mapa local
        :return world_y: Coordenada y no mapa local
        """
        world_x = self.origin_x + (costmap_x * self.resolution)
        world_y = self.origin_y + (costmap_y * self.resolution)
        # world_y = self.origin_y + ((self.height - costmap_y - 1) * self.resolution)

        return world_x, world_y
    
    def transform_to_global(self, local_x, local_y):
        """Transforma coordenadas do costmap local para o sistema global

        :param local_x: Coordenada x no mapa local.
        :param local_y: Coordenada y no mapa local.
        :return world_x: Coordenada x no mapa global
        :return world_y: Coordenada y no mapa global
        """
        try:
            '''
            "trans" conterá uma estrutura com posição e orientação para converter coordenadas locais em globais
            '''
            trans = self.tf_buffer.lookup_transform("map", self.frame_id, rospy.Time(0), rospy.Duration(1.0))
            '''
            "map" → Frame de destino (global).
            self.frame_id → Frame de origem (do costmap local).
            rospy.Time(0) → Pede a transformação mais recente disponível.
            rospy.Duration(1.0) → Aguarda até 1 segundo para encontrar a 
            '''
            point_local = PointStamped()
            point_local.header.frame_id = self.frame_id
            point_local.header.stamp = rospy.Time(0)
            point_local.point.x = local_x
            point_local.point.y = local_y
            point_local.point.z = 0.0
            
            point_global = tf2_geometry_msgs.do_transform_point(point_local, trans)
            return point_global.point.x, point_global.point.y
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Falha ao transformar coordenadas do mapa local para o mapa global.")
            return None, None
        
    def positionAtRout(self):
        """
        Retorna o valor do índice onde ocorreu a menor distancia entre os pontos da rota global e o robo.
        Manerira de fazer um trimming na rota global.
        """
     
        # Determina o índice onde ocorreu a menor distancia entre os pontos da rota inicial e o robo
        indice = None
        dist_minima_robot_rout = float('inf')  # Inicializa a menor distância como infinito

        comprimentoRota = len(self.rotaInicial.poses)
        # print(f"Comprimento da rotaInicial: {comprimentoRota}")
        for i, pose in enumerate(self.rotaInicial.poses[self.LastIndexRout:]): #verifica qual é o índice onde ocoreu a menor distancia entre o robo e a rota inicial
            dist = ((pose.pose.position.x - self.robot_x)**2 + (pose.pose.position.y - self.robot_y)**2)**0.5
            if dist < dist_minima_robot_rout:  # ao percorrer a rota global, cada ponto da rota esta ficando mais próximo do robo
                dist_minima_robot_rout = dist
                indice = i + self.LastIndexRout # Atualiza a menor distância e o índice correspondente
                # print(f"dist_minima: {dist_minima_robot_rout}")
                # print(f"indice: {indice}")

        return indice
    
    def comparePosition(self):
        """Obtém as coordenadas dos obstáculos no costmap local, converte para coordenadas do mapa global, e compara com as coordenadas da rota global
        
        return: retorna True caso as coordenadas da rota global e mapa local coincidam
        """
        if self.costmap is None:
            rospy.logwarn("Costmap local ainda não foi recebido corretamente. Aguardando...")
            return []
        for y in range(0, self.height, 5):    # linhas, conta de 5 em 5 células 
            # print(f"y: {y}")
            for x in range(0, self.width, 5): # colunas, conta de 5 em 5 células 
                # print(f"x: {x}") 
                index = y * self.width + x  # itera sobre todos os índices de cada linha
                cell_value = self.costmap[index]
                
                if cell_value > 95:  # Considera células ocupadas
                    local_x, local_y = self.costmap_to_local(x, y)
                    global_x, global_y = self.transform_to_global(local_x, local_y)
                    # print(f"X: {global_x}, Y: {global_y}")
                    self.LastIndexRout = self.positionAtRout()
                    # print(f"O robo está no índice {self.LastIndexRout} da rota!")
                    for i, pose in enumerate(self.rotaInicial.poses[self.LastIndexRout:]): #verifica se a distancia da coordenada da rota global passa perto da coordenada do mapa local
                        if (i%15) == 0:     # a cada 15 índices
                            # print(f"I           : {i}")
                            dist = ((pose.pose.position.x - global_x)**2 + (pose.pose.position.y - global_y)**2)**0.5
                            if dist < self.dist_minima: 
                                print(f"Obstáculo detectado na rota: {global_x} | {global_y}")
                                # print(f"y: {y}, x: {x}")
                                # print(f"X: {global_x}, Y: {global_y}")
                                return True

                        if i > 100:      # para mapa local de 2x2 metros
                            # print("\n")
                            break
        return False
    
    
    def run(self): 
        # self.cont += 1
        # print(f"self.cont: {self.cont}")
        if self.rotaInicial:
            obstaculo = self.comparePosition()
            # print(f"Obstáculo: {obstaculo}")
            if obstaculo:
                # print(f"velocity.linear.x: {velocity.linear.x}")
                # print(f"velocity.angular.z: {velocity.angular.z}")
                self.cmd_vel_pub.publish(Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)))
            # else:
            #     # self.cmd_vel_pub.publish(Twist(linear=Vector3(self.velocity.linear.x, 0, 0), angular=Vector3(0, 0, self.velocity.angular.z)))
            #     self.cmd_vel_pub.publish(self.velocity)
                
            
            # while self.comparePosition() and not rospy.is_shutdown():
            #     # velocity = Twist()
            #     # velocity.linear.x = 0
            #     # velocity.angular.z = 0
            #     # print(f"velocity.linear.x: {velocity.linear.x}")
            #     # print(f"velocity.angular.z: {velocity.angular.z}")
            #     # self.cmd_vel_pub.publish(velocity)
            #     self.cmd_vel_pub.publish(Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)))

            #     # start_time = rospy.Time.now().to_sec()
            #     # while rospy.Time.now().to_sec() - start_time < 5.0:
            #     #     self.cmd_vel_pub.publish(Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)))
            #         # print("velocidade zero!!!")
            #     # time.sleep(3)
            
            # self.cmd_vel_pub.publish(self.velocity)
            


if __name__ == '__main__':
    rospy.init_node('CostMapObstacle_node', anonymous=True)
    processor = LocalCostmapProcessor()
    rate = rospy.Rate(5)
    
    while not rospy.is_shutdown():
        processor.run()
        rate.sleep()

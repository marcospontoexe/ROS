#!/usr/bin/env python3

import time
import rospy
import json
from log_manager import LogManager
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import String, Bool, Int32
from actionlib_msgs.msg import GoalID  
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult # para monitorar o tópico /movebase/result
from dynamic_reconfigure.client import Client
from sensor_msgs.msg import BatteryState
import os
import math
import tf

class NavStack(object):

    def __init__(self):

        # Inicializa o LogManager
        self.log_manager = LogManager()

        # self.caminho_rota = "/home/ubuntu/catkin_ws/src/nav_hub/scripts/config/backup_do_route.json"
        # self.caminho_rota = "/home/ubuntu/catkin_ws/src/nav_hub/scripts/config/backup_do_route_botoeira_teste_renault.json"
        # self.caminho_rota = "/home/ubuntu/catkin_ws/src/nav_hub/scripts/config/pontos_proximos.json"
        # self.caminho_rota = "/home/ubuntu/catkin_ws/src/nav_hub/scripts/config/gemu_pequeno.json"
        # self.caminho_rota = "/home/ubuntu/catkin_ws/src/nav_hub/scripts/config/gemu_plano.json"
        # self.caminho_rota = "/home/ubuntu/catkin_ws/src/nav_hub/scripts/config/gemu_V2.0.json"
        # self.caminho_rota = "/home/ubuntu/catkin_ws/src/nav_hub/scripts/config/volks_global_planner.json"
        # self.caminho_rota = "/home/ubuntu/catkin_ws/src/nav_hub/scripts/config/aceleradora.json"
        self.caminho_rota = "/home/ubuntu/catkin_ws/src/nav_hub/scripts/config/demo_aceleradora.json"


        with open(self.caminho_rota, "r+") as fi:  
            self.datajson = json.load(fi)
            fi.close() # Fecha o arquivo 

        
        self.goal_x = 0.0
        self.goal_y = 0.0

        #the "beginning" flag determines if the code is at its beginning or not
        self.beginning = True

        #clear the map
        self.command = 'rosservice call /move_base/clear_costmaps "{}"'

        self.poseAtual = [0.0, 0.0]
        # self.rampa = [10.282468836770, 16.03000360]          # coordenadas para reduzir a velocidade
        # self.flagDiminuirVelocidade = False
        self.current_yaw = None
        self.flagReached = False
        self.emMovimento = False
        self.navegando = False
        self.flagChegando = False
        self.voltageBattery = None

        self.velocidade = Twist()
        self.vel = Twist()

        #variable to make the robot STOP
        self.goal_empty = GoalID()

        self.started = False
        self.iniciou = False

        # self.fitaled = fitaledClass.RobotController()

        #subscribers
        rospy.Subscriber("/button", Int32, self.sub_button, queue_size=1) # publicada pela botoeira 
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.movebase_result_callback, queue_size=1)  # Monitora o status da navegação
        rospy.Subscriber("/cmd_vel", Twist, self.velocidade_callback, queue_size=1)
        rospy.Subscriber("/keyence_pose", PoseWithCovarianceStamped, self.poseCallBack, queue_size=1)  # Para verificar a posição do robô
        rospy.Subscriber('/battery_state', BatteryState, self.callbackBattery, queue_size=1)

        #publisher
        #possíveis sons: Pronto - Andando - Parado - Botão
        self.som_pub = rospy.Publisher("/som", String, queue_size=1, latch=True)  # publica uma som para o código controlaSom.py gerenciar
        self.cor_pub = rospy.Publisher("/cor", String, queue_size=1, latch=True)  # publica uma cor para o código fitaLed_v2.0 gerenciar
        self.position_publisher = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
        self.reached_pub = rospy.Publisher("/has_reached", Bool, queue_size=1, latch=True) # usado pelo botoeira e move_back.py
        self.nomeRota_pub = rospy.Publisher("/nomeRota", String, queue_size=1, latch=True)
        self.goal_publisher = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=1, latch=True)
        self.quantidade_rotas_publisher = rospy.Publisher("/quantidade_rotas", Int32, queue_size=1, latch=True) # publicada a quantidade de rotas para o código da botoeira
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.navegando_pub = rospy.Publisher("/navegando", Bool, queue_size=1, latch=True)
    
    def velocidade_callback(self, data):
        self.velocidade = data
    
    def poseCallBack(self, data):
        try:
            self.poseAtual[0] = data.pose.pose.position.x
            self.poseAtual[1] = data.pose.pose.position.y

        except:
            rospy.logwarn("Não foi possível obter a pose do robo!")

    def movebase_result_callback(self, msg):
        """
        Callback para processar o status do move_base. 
        """        
        # print(f"result_callback do main: {msg.status.status}")
        if msg.status.status == 3:  # Verifica se o objetivo foi alcançado
            self.flagReached = True
            self.emMovimento = False
        elif msg.status.status == 4:  # Status 4 significa falha no planejamento (ABORTED)
            rospy.loginfo("O move_base abortou a rota!")
            self.log_manager.gera_log("O move_base abortou a rota!", LogManager.Warn)
            self.cor_pub.publish("Apagado")
            time.sleep(3)
            # os.system('pkill -f essential_global_planner.launch')
            # os.system('pkill -f global_planner.launch')
            self.resetLaunch.killLaunch()
        
    def callbackBattery(self, msg):
        """
        Callback que monitora o estado da bateria e muda o LED para vermelho se a carga estiver baixa.
        """
        self.voltageBattery = msg.voltage

    def contar_rotas(self):
        dados = self.datajson
        contador = 0
        for chave in dados.keys():
            # print(f"chave do json: {chave}")
            if 'step' in chave:
                # print("entrou no if")
                contador += 1
                
        return contador

    def sub_button(self, msg):
        # ponto = Int32()
        ponto = msg.data
        # print(f"entrou no sub_button(): {type(ponto)}")
        os.system(self.command)
        value = 'nothing'
        for things in self.datajson:    # obtem a posição e orientação do arquivo json do ponto selecionado.
            try:
                if (self.datajson[things]['sequence']) == ponto:
                    value = things
            except:
                pass
        
        if self.beginning:  
            # Define a posição inicial do robô
            self.beginning = False
            
            position = PoseWithCovarianceStamped()

            # time.sleep(1)
            position.header.seq = 0
            position.header.stamp = rospy.Time.now()
            position.header.frame_id = "map"
            
            position.pose.pose.position.x = self.datajson[value]['x']
            position.pose.pose.position.y = self.datajson[value]['y']
            position.pose.pose.orientation.z = self.datajson[value]['z']
            position.pose.pose.orientation.w = self.datajson[value]['w']
            position.pose.covariance = self.datajson["initial"]["covariance"]
            self.position_publisher.publish(position)

            print(f"Definindo posição inicial do robô em: {self.datajson[value]['name']}")
            self.log_manager.gera_log("Definindo posição inicial do robô em " + self.datajson[value]['name'], LogManager.Info)
            
            os.system(self.command)
        else:     # envia um destino objetivo para o move base       
            #print("GOAL enviado pelo sub_button() quando beginning == False")               
            goal = MoveBaseActionGoal()

            goal.goal.target_pose.header.seq = 0
            goal.goal.target_pose.header.stamp = rospy.Time.now()
            goal.goal.target_pose.header.frame_id = "map"
            
            goal.goal.target_pose.pose.position.x = self.datajson[value]['x']
            goal.goal.target_pose.pose.position.y = self.datajson[value]['y']
            goal.goal.target_pose.pose.orientation.z = self.datajson[value]['z']
            goal.goal.target_pose.pose.orientation.w = self.datajson[value]['w']
            self.goal_publisher.publish(goal)
            self.nomeRota_pub.publish(self.datajson[value]['name'])
            self.goal_x = self.datajson[value]['x']
            self.goal_y = self.datajson[value]['y']
            # print(f"x: {goal_x}")
            # print(f"y: {goal_y}")
   
            self.navegando = True
            self.navegando_pub.publish(True)
            self.flagChegando = False
            self.reached_pub.publish(False)
            self.set_min_vel_x(0.0)
            self.max_vel_theta(0.5)
            self.cor_pub.publish("Ciano")
            self.som_pub.publish("Andando")
            print(f"Enviado para: {self.datajson[value]['name']}")
            self.log_manager.gera_log("Enviado para: " + self.datajson[value]['name'], LogManager.Info)
            self.log_manager.gera_log("Robô em movimento.", LogManager.Info)
                
    def set_vel_x(self, value):
        client = Client("/move_base/DWAPlannerROS", timeout=5)
        params = {"max_vel_x": value}
        params1 = {"max_vel_trans": value}
        client.update_configuration(params)
        client.update_configuration(params1)

    def set_min_vel_x(self, value):
        client = Client("/move_base/DWAPlannerROS", timeout=5)
        params = {"min_vel_x": value}
        client.update_configuration(params)

    def max_vel_theta(self, value):
        client = Client("/move_base/DWAPlannerROS", timeout=5)
        params = {"max_vel_theta": value}
        client.update_configuration(params)
        
    def navigate(self):
        if not self.iniciou:    #executa apenas no início da execução do código
            self.iniciou = True
            qntdRotas = self.contar_rotas()
            self.quantidade_rotas_publisher.publish(qntdRotas)
            rospy.wait_for_service('/move_base/clear_costmaps')  # Espera até que o serviço esteja disponível
            os.system(self.command)
            # self.cor_pub.publish("Verde")
            self.reached_pub.publish(True)
            self.navegando_pub.publish(False)
            self.som_pub.publish("Pronto")
            print("main_global_planner_linear_dinamica.py pronto.")
            nav_stack_object.log_manager.gera_log("main_global_planner_linear_dinamica.py iniciado com sucesso.", LogManager.Info)
        
        #Determines if the robot has arrived or not  
        elif self.flagReached and self.flagChegando:    # "self.flagChegando" precisa ser usada por causa do "move_back.py"
            self.flagReached = False                                                                
            self.navegando = False  
            self.navegando_pub.publish(False)
            rospy.loginfo("main_global_planner_linear_dinamica detectou objetivo alcançado.")
            self.log_manager.gera_log("Tensão na bateria: " + str(self.voltageBattery), LogManager.Info)
            self.reached_pub.publish(True)
            #clear the map
            os.system(self.command)
            time.sleep(1)   
            
        elif self.navegando: #quando uma rota é enviada para o move base
            if abs(self.velocidade.linear.x) <= 0.0062 and abs(self.velocidade.angular.z) <= 0.037: # caso o robo esteja se deslocando para o objetivo e fique parado
                # print("entrou no if parado")
                if not self.emMovimento:
                    self.emMovimento = True
                    self.som_pub.publish("Parado")
                    self.cor_pub.publish("Amarelo")
                    self.log_manager.gera_log("Robô parou devido a obstáculo.", LogManager.Info)
            
            else:   #em movimento
                # print("entrou no else andando")
                if self.emMovimento:
                    self.emMovimento = False
                    self.som_pub.publish("Andando")
                    self.cor_pub.publish("Ciano")
                    self.log_manager.gera_log("Robô em movimento.", LogManager.Info)    

            if not self.flagChegando:
                # print(f"distacia: {math.sqrt((self.poseAtual[0] - self.goal_x)**2 + (self.poseAtual[1] - self.goal_y)**2)}")
                if math.sqrt((self.poseAtual[0] - self.goal_x)**2 + (self.poseAtual[1] - self.goal_y)**2) <= 0.5:   #verifica se a distancia euclidiana entre o robo e o destino é menor do que 50 cm
                    self.flagChegando = True
                    self.set_min_vel_x(-0.1)    # permite o robo mover-se de ré quando está próximo do destino
                    self.max_vel_theta(0.25)      # para girar lentamente, não perdendo a orientação
                
                # # para fazer o robo diminir a velocidade quando se aproxima de um obstáculo
                # dist = math.sqrt((self.poseAtual[0] - self.rampa[0])**2 + (self.poseAtual[1] - self.rampa[1])**2)
                # # print(f"Distancia: {dist}")
                # if (dist <= 4) and not self.flagDiminuirVelocidade:   #se o robo estiver próximo da rampa diminui a velocidade
                #     if not self.flagDiminuirVelocidade:
                #         self.flagDiminuirVelocidade = True
                #         self.set_vel_x(0.3)     #diminui velocidade
                #         print("Diminui velocidade")
                # elif (dist > 4) and self.flagDiminuirVelocidade:   #se o robo estiver longe da rampa aumenta a velocidade
                #     if self.flagDiminuirVelocidade:
                #         self.flagDiminuirVelocidade = False
                #         self.set_vel_x(0.5)     # almenta velocidade
                #         print("Aumentou velocidade")
           

if __name__ == '__main__':
    rospy.init_node("main_global_planner_linear_dinamica_node", anonymous=True)
    rate = rospy.Rate(1)
    nav_stack_object = NavStack()
    print("Iniciando main_global_planner_linear_dinamica.py...")
    nav_stack_object.log_manager.gera_log("Iniciando  main_global_planner_linear_dinamica.py.", LogManager.Info)
    while not rospy.is_shutdown():
        nav_stack_object.navigate()
        rate.sleep()
    nav_stack_object.cor_pub.publish("Apagar") #apaga a fita led

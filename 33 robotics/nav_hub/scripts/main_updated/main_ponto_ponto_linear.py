#!/usr/bin/env python
import time
import rospy
import json
from move_base_msgs.msg import MoveBaseActionResult # para monitorar o tópico /movebase/result
from log_manager import LogManager
from collections import OrderedDict
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped, Twist, Pose, PoseArray
from std_msgs.msg import String, Bool, Int32
from actionlib_msgs.msg import GoalID
from sensor_msgs.msg import BatteryState
import os
import random

class NavStack(object):

    def __init__(self):
        # self.caminho_rota = "/home/ubuntu/catkin_ws/src/nav_hub/scripts/config/ponto-ponto/"    
        # self.caminho_rota = "/home/ubuntu/catkin_ws/src/nav_hub/scripts/config/ponto-ponto-hercules/" 
        self.caminho_rota = "/home/ubuntu/catkin_ws/src/nav_hub/scripts/config/ponto-ponto-demo/"    
   

        self.total_rotas = 0 
        #variable that stores the next goal

        #the "beginning" flag determines if the code is at its beginning or not
        self.beginning = True
    
        #clear the map
        self.command = 'rosservice call /move_base/clear_costmaps "{}"'

        self.flagReached = False
        self.iniciou = False
        self.navegando = False
        self.emMovimento = False
        self.poseAtual = [0.0, 0.0]
        self.voltageBattery = None

        self.log_manager = LogManager()
        self.velocidade = Twist()

        #subscribers
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.movebase_result_callback, queue_size=1)  # Monitora o status da navegação
        rospy.Subscriber("/destination", Int32, self.destination_callback, queue_size=1)
        rospy.Subscriber("/cmd_vel", Twist, self.velocidade_callback, queue_size=1)
        rospy.Subscriber('/battery_state', BatteryState, self.callbackBattery, queue_size=1)


        #publishers
        self.reached_pub = rospy.Publisher("/has_reached", Bool, queue_size=1, latch=True)
        self.totRotas_pub = rospy.Publisher("/total_rotas", Int32, queue_size=1, latch=True)
        self.cor_pub = rospy.Publisher("/cor", String, queue_size=1, latch=True)  # publica uma cor para o código fitaLed_v2.0 gerenciar
        self.som_pub = rospy.Publisher("/som", String, queue_size=1, latch=True)  # publica uma som para o código controlaSom.py gerenciar
        #possíveis sons: Pronto - Andando - Parado - Botão
        self.particle_pub = rospy.Publisher('/particlecloud', PoseArray, queue_size=1, latch=True)  #para espalhar a nuvem de particulas
        self.position_publisher = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True) 
        self.clicked_pub = rospy.Publisher("/new_clicked_point", PointStamped, queue_size=50, latch=True)
        # self.clicked_pub = rospy.Publisher("/clicked_point", PointStamped, queue_size=50, latch=True)
        self.navegando_pub = rospy.Publisher("/navegando", Bool, queue_size=1, latch=True)

    
    def movebase_result_callback(self, msg):
        """
        Callback para processar o status do move_base. 
        """        
        # print(f"result_callback do main: {msg.status.status}")
        if msg.status.status == 3:  # Verifica se o objetivo foi alcançado
            self.flagReached = True
            self.emMovimento = False
        elif msg.status.status == 4:  # Status 4 significa falha no planejamento (ABORTED)
            rospy.loginfo("O move_base abortou a rota!.")
            self.log_manager.gera_log("O move_base abortou a rota!", LogManager.Warn)
            self.cor_pub.publish("Apagado")
            time.sleep(3)
            # os.system('pkill -f essential_ponto_ponto.launch')
            # os.system('pkill -f ponto_ponto.launch')
            self.resetLaunch.killLaunch()
            

    def velocidade_callback(self, data):
            self.velocidade = data
            
    def callbackBattery(self, msg):
        """
        Callback que monitora o estado da bateria e muda o LED para vermelho se a carga estiver baixa.
        """
        self.voltageBattery = msg.voltage

    def destination_callback(self, data):
        destino = str(data.data)
        print(f"rota: {destino}")
        with open(self.caminho_rota+destino+".json", "r+") as fi:
            route_data = json.load(fi, object_pairs_hook=OrderedDict)
            fi.close()

        if self.beginning:  # apenas no inicio
            self.beginning = False
            
            #Define a posição e orientação inicial do robô.
            position = PoseWithCovarianceStamped()
            position.header.seq = 0
            position.header.stamp = rospy.Time.now()
            position.header.frame_id = "map"
            # Pose: A posição e orientação estimada do robô, especificada como um geometry_msgs/Pose.
            position.pose.pose.position.x = route_data['initial']['x']
            position.pose.pose.position.y = route_data['initial']['y']
            position.pose.pose.orientation.z = route_data['initial']['z']
            position.pose.pose.orientation.w = route_data['initial']['w']
            # Covariance: Uma matriz que representa a incerteza associada à pose estimada, o que permite que o sistema ajuste a confiança nessa estimativa.
            position.pose.covariance = route_data['initial']['covariance']
            
            # Publica a posição e orientação inicial no tópico /initialpose.
            self.position_publisher.publish(position)
            print(f"Posição inicial configurada no ponto {destino}")

        os.system(self.command)
        value = 'nothing'
        sequence = 2
        contador = 0
        things = ''
        goal = PointStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        for things in route_data:
            if things != "initial":
                try:                    
                    if (route_data[things]['sequence']) == sequence:
                        goal.point.x = route_data[things]['x']
                        goal.point.y = route_data[things]['y']
                        contador += 1
                        goal.header.seq = contador
                        self.clicked_pub.publish(goal)
                        sequence += 1
                except:
                    pass
            if things == "ending":
                break

        self.navegando = True
        self.navegando_pub.publish(True)
        self.reached_pub.publish(False)
        self.cor_pub.publish("Ciano")
        self.som_pub.publish("Andando")
        self.log_manager.gera_log("Robô em movimento.", LogManager.Info)


    def contar_arquivos_json(self, diretorio):
        '''Conta quantos arquivos terminados em ".json" contem no diretório "diretorio"
        '''
        try:
            arquivos = os.listdir(diretorio)
            json_files = [arquivo for arquivo in arquivos if arquivo.endswith(".json")]
            return len(json_files)
        except FileNotFoundError:
            print(f"O diretório {diretorio} não foi encontrado.")
            return 0
        except PermissionError:
            print(f"Sem permissão para acessar {diretorio}.")
            return 0    
        
    def navigate(self):
        if not self.iniciou:    #executa apenas no início da execução do código
            self.iniciou = True
            self.total_rotas = self.contar_arquivos_json(self.caminho_rota)
            # self.total_rotas = int((self.total_rotas + 1) / 2)        # feito para demonstração na metal kraft. A quantidade de rotas realizadas pela botoeira lora não entra na contagem
            rospy.wait_for_service('/move_base/clear_costmaps')  # Espera até que o serviço esteja disponível
            os.system(self.command)
            self.reached_pub.publish(True)
            self.navegando_pub.publish(False)
            self.totRotas_pub.publish(self.total_rotas)
            self.cor_pub.publish("Verde")
            self.som_pub.publish("Pronto")
            print("Navegação ponto a ponto linear pronta.")
            self.log_manager.gera_log("Navegação ponto a ponto linear iniciada com sucesso." , LogManager.Info)

        #Determines if the robot has arrived or not  
        elif self.flagReached:
            self.flagReached = False                                                                
            self.reached_pub.publish(True)
            self.navegando_pub.publish(False)
            self.navegando = False
            #clear the map
            os.system(self.command)
            rospy.loginfo("main_ponto_ponto_linear detectou objetivo alcançado.")
            self.log_manager.gera_log("Tensão na bateria: " + str(self.voltageBattery), LogManager.Info)

        
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


if __name__ == '__main__':
    rospy.init_node("main_ponto_ponto_linear_node")
    rate = rospy.Rate(2)
    nav_stack_object = NavStack()
    print("Iniciando main_ponto_ponto_linear.py...")
    nav_stack_object.log_manager.gera_log("Iniciando main_ponto_ponto_linear.py", LogManager.Info)
    while not rospy.is_shutdown():
        nav_stack_object.navigate()
        rate.sleep()
    nav_stack_object.cor_pub.publish("Apagar") #apaga a fita led

#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PointStamped
from std_msgs.msg import String, Bool
import os
import subprocess
import threading
import time
import json
import tf

class JoyTeleop:
    def __init__(self):
        self.caminho_rota = "/home/ubuntu/catkin_ws/src/nav_hub/scripts/config/ponto-ponto-demo/"  

        # Diretório e nome para salvar o mapa
        self.caminho_mapa = "/home/ubuntu/catkin_ws/src/nav_hub/maps/mapa_demo"
        
        self.serializarMapa = f'rosservice call /slam_toolbox/serialize_map "filename: \'{self.caminho_mapa}\'"'     # Comando para serializar o mapa
        self.salvarMapa = f'rosrun map_server map_saver -f \'{self.caminho_mapa}\''     # Comando para salvar o mapa com o map server
        

        # Fator de escala para as velocidades linear e angular
        self.linear_scale = 0.7
        self.angular_scale = 0.5
        self.smothLinear = 0.08
        self.smothAngular = 1

        self.lastVel = Twist()
        self.lastVel.linear.x = 0
        self.lastVel.angular.z = 0

        # Armazena a mensagem Twist
        self.twist = Twist()
        
        # Variável para armazenar o último valor recebido do /joy
        self.last_joy_msg = None
        self.velLinear = 0.0
        self.velAngular = 0.0

        self.flagToggleLaunch = False
        self.flagSalvarMapa = False
        self.flagTerminarCapuraPontos = False
        self.flagCapturarPontos = False
        self.flagControle = False
        self.flagApagarRotas = False
        self.flagMapping = False
        self.flagReiniciar = False
        self.flagDesligar = False
        self.flagParar = False
        self.flagTeleopON = True
        self.flagToggleTeleop = False
        self.flagToggleVelo = False
        self.flagVelMax = True
        self.flagAlavanca = True
        self.flagIniciou = False

        self.flagMappingOnce = True
        self.navegando = False

        self.reached = False

        self.lastStatColor = "Branco"

        self.lock = threading.Lock()

        self.PontosRotas = []

        self.tempo_inicial = 0.0
        self.tempo_atual = 0.0

        # Subscritor para o tópico /joy
        rospy.Subscriber('/joy', Joy, self.joy_callback, queue_size=1)
        rospy.Subscriber('has_reached', Bool, self.reached_callback, queue_size=1)
        rospy.Subscriber('navegando', Bool, self.navegando_callback, queue_size=1)


        # Publicador para o tópico /lds15_cmdvel
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1, latch=True)
        self.cor_pub = rospy.Publisher("/cor", String, queue_size=1, latch=True)  # publica uma cor para o código fitaLed_v2.0.py gerenciar


        self.cor_pub.publish(self.lastStatColor)

    # def powerOff(self):
    #     '''Função para desligar o systema operacional
    #     '''

    def joy_callback(self, joy_msg):

        if not self.flagIniciou:
            self.flagIniciou = True
        
        if self.flagAlavanca:
            self.twist.linear.x = joy_msg.axes[1] * self.linear_scale
            self.twist.angular.z = joy_msg.axes[0] * self.angular_scale
        else:
            self.twist.linear.x = joy_msg.axes[7] * self.linear_scale
            self.twist.angular.z = joy_msg.axes[6] * self.angular_scale

        self.tempo_inicial = time.time()
        self.tempo_atual = 0.0
        

        if joy_msg.buttons[7]:#botão usado para ativar o uso dos demais botões. qualquer outro botão só será lido se este botão estiver apertado

            #botão para capturar um ponto de uma rota ponto-ponto
            if joy_msg.buttons[0]:      #A
                self.flagCapturarPontos = True

            #botão para alterar o controle direcional entre alavanca ou cruz
            if joy_msg.buttons[1]:      #B
                self.flagControle = True
            
            #botão toggle velociade
            elif joy_msg.buttons[3]:      #X
                self.flagToggleVelo = True
                # self.cmd_vel_pub.publish(Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)))

            #botão toggle teleop
            elif joy_msg.buttons[4]:      #Y
                self.flagToggleTeleop = True
                # self.cmd_vel_pub.publish(Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)))
            
            #botão para iniciar o mapeamento ou ponto-ponto
            elif joy_msg.buttons[8]:      #L2
                self.flagToggleLaunch = True

            #botão para salvar o mapa
            elif joy_msg.buttons[9]:      #R2 
                self.flagSalvarMapa = True
            
            #botão para apagar todas as rotas criadas
            elif joy_msg.buttons[10]:      #Select
                self.flagApagarRotas = True

            #botão para encerrar a captura dos pontos de uma rota ponto-ponto e criar um arquivo .json contendo a rota
            elif joy_msg.buttons[11]:      #Start
                self.flagTerminarCapuraPontos = True

            #botão para reiniciar o system operacional
            elif joy_msg.buttons[13]:      #analogico esquerdo
                self.flagReiniciar = True

            #botão para desligar o system operacional
            elif joy_msg.buttons[14]:      #analogico direito
                self.flagDesligar = True
            
        
            
    def reached_callback(self, data):
        self.reached = data.data

    def navegando_callback(self, data):
        self.navegando = data.data
        print(f"navegando: {self.navegando}")

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

    def criarJson(self, nomeRota, pontos):
        """
        Cria um arquivo .json no diretório "self.caminho_rota"

        Parâmetros:
            nomeRota (str): nome do arquivo a ser criado (sem ou com extensão .json)
            pontos (list): lista de listas/tuplas, onde cada elemento é [x, y, w, z]

        Retorna:
            bool: True se criar o arquivo com sucesso, False caso contrário.
        """

        # print(f"nomeRota: {nomeRota}")

        # Se o nome não terminar com ".json", adiciona
        if not nomeRota.lower().endswith(".json"):
            nome_arquivo = nomeRota + ".json"
        else:
            nome_arquivo = nomeRota

        # print(f"nome do arquivo: {nome_arquivo}")

        # Caminho completo do arquivo
        caminho_completo = os.path.join(self.caminho_rota, nome_arquivo)

        # Dados fixos de covariância para a chave "initial"
        covariancia_fixa = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942
        ]

        try:
            # Garante que o diretório existe; se não existir, cria as pastas necessárias
            os.makedirs(self.caminho_rota, exist_ok=True)

            # Montar estrutura do JSON
            dados_json = {}

            # Montar a chave "initial" a partir do ponto de índice 0
            x0, y0, w0, z0 = pontos[0]
            # print(f"x0: {x0}")
            # print(f"y0: {y0}")
            # print(f"w0: {w0}")
            # print(f"z0: {z0}")

            dados_json["initial"] = {
                "sequence": 1,
                "covariance": covariancia_fixa,
                "w": w0,
                "y": y0,
                "x": x0,
                "z": z0
            }

            # Montar a chave "step0" a partir do mesmo ponto inicial (índice 0)
            dados_json["step0"] = {
                "sequence": 2,
                "w": w0,
                "y": y0,
                "x": x0,
                "z": z0
            }

            # Caso haja pontos intermediários (entre o índice 1 e o penúltimo),
            # monta as chaves "step1", "step2", etc. Até o índice N-2
            # O valor de "sequence" vai aumentando sequencialmente
            # Já reservamos sequence=1 para “initial” e sequence=2 para “step0”
            seq_atual = 3  # próxima sequência: 3
            # percorrer índices de 1 até len(pontos)-2 (inclusive) para criar step1, step2, ...
            for idx in range(1, len(pontos) - 1):
                xi, yi, wi, zi = pontos[idx]
                # print(f"idx: {idx}")
                # print(f"xi: {xi}")
                # print(f"yi: {yi}")
                # print(f"wi: {wi}")
                # print(f"zi: {zi}")
                chave_step = f"step{idx}"
                dados_json[chave_step] = {
                    "sequence": seq_atual,
                    "w": wi,
                    "y": yi,
                    "x": xi,
                    "z": zi
                }
                seq_atual += 1

            # Montar a chave "ending" com base no último ponto (índice len(pontos)-1)
            x_last, y_last, w_last, z_last = pontos[-1]
            dados_json["ending"] = {
                "sequence": len(pontos) + 1,
                "w": w_last,
                "y": y_last,
                "x": x_last,
                "z": z_last
            }

            # Gravar em disco com identação de 4 espaços para facilitar leitura
            with open(caminho_completo, "w") as f:
                json.dump(dados_json, f, indent=4)
                f.close()

            return True

        except Exception as e:
            # Caso haja qualquer erro (permissão, diretório inexistente, etc.), retorna False
            # Opcionalmente, pode-se fazer um log: print(f"Erro ao criar JSON: {e}")
            return False
            

    def apagar_jsons(self):
        """
        Acessa o diretório "self.caminho_rota" e apaga todos os arquivos com extensão ".json".

        Retorna:
            True  - se todos os arquivos JSON forem excluídos (ou se não houver .json)
            False - se ocorrer qualquer erro (diretório inacessível, permissão negada, etc.)
        """
        
        try:
            # Verifica se o diretório existe antes de tentar listar
            if not os.path.isdir(self.caminho_rota):
                # Retorna False caso o diretório não exista ou não seja uma pasta
                return False

            # Itera sobre todas as entradas (arquivos e pastas) no diretório
            for nome_arquivo in os.listdir(self.caminho_rota):
                # Verifica se o nome termina com ".json"
                if nome_arquivo.lower().endswith(".json"):
                    # Monta o caminho completo para o arquivo
                    caminho_completo = os.path.join(self.caminho_rota, nome_arquivo)
                    # Remove o arquivo JSON encontrado
                    os.remove(caminho_completo)

            return True

        except Exception:
            # Qualquer erro durante listagem ou remoção resulta em False
            return False

    
    def tf_entre_map_e_odom(self):
        listener = tf.TransformListener()
        try:
            listener.waitForTransform("/map", "/odom", rospy.Time(0), rospy.Duration(5.0))
            return True
        except tf.Exception:
            return False
        

    def run(self):
        if self.flagIniciou:
            self.tempo_atual = time.time() - self.tempo_inicial # Calcula o tempo decorrido
            if self.tempo_atual >= 1:       # caso perca a conexão com o joystick, e o tempo seja maior que 3 segundos
                self.twist.linear.x = 0
                self.twist.angular.z = 0

            if not self.navegando:

                # para controlar transissões brutas
                self.lastVel.linear.x = self.lastVel.linear.x + ((self.twist.linear.x - self.lastVel.linear.x) * self.smothLinear)
                self.lastVel.angular.z = self.lastVel.angular.z + ((self.twist.angular.z - self.lastVel.angular.z) * self.smothAngular)

                # self.twist.linear.x = self.lastLinear 
                # self.twist.angular.z = self.lastAngular

                # print(f"linear.x: {self.twist.linear.x}")
                # print(f"angular.z: {self.twist.angular.z}")

                # print(f"lastLinear.x: {self.lastVel.linear.x}")
                # print(f"lastLinear.z: {self.lastVel.angular.z}")

                # Publica a mensagem Twist no tópico /cmd_vel
                self.cmd_vel_pub.publish(self.lastVel)
                

            if self.flagToggleLaunch:      # altera entre a lauch de mapeamento e de navegação
                if not self.flagMapping:    # modo mapeamento
                    self.flagMapping = True
                    self.reached = False
                    self.navegando = False
                    self.flagVelMax = False
                    self.linear_scale = 0.2
                    self.angular_scale = 0.2
                    self.smothLinear = 0.4
                    self.smothAngular = 0.4
                    self.lastStatColor = "Azul"
                    self.cor_pub.publish("Apagado")
                    time.sleep(1)
                    # encerra navegação
                    subprocess.run(['pkill', '-f', 'essential_ponto_ponto.launch']) 
                    subprocess.run(['pkill', '-f', 'ponto_ponto.launch']) 
                    # subprocess.run(['pkill', '-f', 'essential_global_planner.launch']) 
                    # subprocess.run(['pkill', '-f', 'global_planner.launch']) 
                    subprocess.run(['rosnode', 'kill', '/fitaLed_node'])   
                    with self.lock:     # executa a launch mapeamento (Protege chamadas a subprocess.Popen/terminate em threads)
                        try:
                            self.procMapping = subprocess.Popen(['roslaunch', 'slam_toolbox', 'mapeamento.launch'])        # disparado em background
                            # print(f"procMapping: {self.procMapping}")
                        except Exception as e:
                                rospy.logerr(f"[mapeamento.launch] Erro ao executar roslaunch: {e}")
                    while not self.tf_entre_map_e_odom():     # enquanto não publica tf entre map e odom
                        time.sleep(0.1)
                    self.cor_pub.publish(self.lastStatColor) 

                    
                else:       # modo navegação
                    # encerra a launch mapeamento
                    self.flagMapping = False
                    self.navegando = False
                    self.flagVelMax = True
                    self.cor_pub.publish("Apagado")
                    time.sleep(1)
                    self.lastStatColor = "Verde"
                    self.linear_scale = 0.7
                    self.angular_scale = 0.4
                    self.smothLinear = 0.08
                    self.smothAngular = 1
                    subprocess.run(['pkill', '-f', 'mapeamento.launch']) 
                    # self.procMapping.terminate()   

                    # inicia launch de navegação
                    with self.lock:     # executa a launch navegação (Protege chamadas a subprocess.Popen/terminate em threads)
                        try:
                            self.procMapping = subprocess.Popen(['roslaunch', 'nav_hub', 'essential_ponto_ponto.launch'])        # disparado em background
                            self.procMapping = subprocess.Popen(['roslaunch', 'nav_hub', 'ponto_ponto.launch'])        # disparado em background
                            # self.procMapping = subprocess.Popen(['roslaunch', 'nav_hub', 'essential_global_planner.launch'])        # disparado em background
                            # self.procMapping = subprocess.Popen(['roslaunch', 'nav_hub', 'global_planner.launch'])        # disparado em background
                        except Exception as e:
                                rospy.logerr(f"[mapeamento.launch] Erro ao executar roslaunch: {e}")
                    
                    # while not self.reached:     # enquanto o main não publicar true no tópico /has_reached
                    #     time.sleep(1)
                    # self.reached = False

                time.sleep(1)   # delay usado como proteao de debounce caso nenhum if seja atendido
                self.flagToggleLaunch = False

                   
            elif self.flagSalvarMapa and self.flagMapping:   # encerra a launch mapeamento somente se a launch ja estiver rodando            
                self.cor_pub.publish("Laranja")
                os.system(self.salvarMapa)
                os.system(self.serializarMapa)
                time.sleep(1)   # delay usado como proteao de debounce 
                self.flagSalvarMapa = False
                self.cor_pub.publish(self.lastStatColor)
                
            
            elif self.flagCapturarPontos:
                self.cor_pub.publish("Laranja")
                # Obtém a posição atual do robô a partir do tópico '/keyence_pose'
                robot_pose = rospy.wait_for_message('/keyence_pose', PoseWithCovarianceStamped)
                robot_x = robot_pose.pose.pose.position.x
                robot_y = robot_pose.pose.pose.position.y
                robot_w = robot_pose.pose.pose.orientation.w
                robot_z = robot_pose.pose.pose.orientation.z

                self.PontosRotas.append((robot_x, robot_y, robot_w, robot_z))
                # print(f"self.PontosRotas: {self.PontosRotas}")
                time.sleep(1)
                self.flagCapturarPontos = False
                self.cor_pub.publish(self.lastStatColor)

            elif self.flagTerminarCapuraPontos:
                if len(self.PontosRotas) > 1:       # craia um arquivo json apenas se existir dois ou mais pontos capturados
                    nomeNovoJson = str(self.contar_arquivos_json(self.caminho_rota) + 1)      # retorna o total de arquivos .json existentes + 1
                    resultCriar = self.criarJson(nomeNovoJson, self.PontosRotas)             # cria um arquivo com o nome do parametros "nomeNovoJson", com o conteudo de "self.PontosRotas"
                    if resultCriar:
                        self.cor_pub.publish("Laranja")
                        self.PontosRotas = []
                        time.sleep(1)
                        self.cor_pub.publish(self.lastStatColor)
                    else:
                        self.cor_pub.publish("Vermelho")

                time.sleep(1)
                self.flagTerminarCapuraPontos = False
                

            elif self.flagApagarRotas: 
                resultApagar = self.apagar_jsons()
                self.PontosRotas = []
                if resultApagar:
                    self.cor_pub.publish("Laranja")
                    time.sleep(1)
                    self.cor_pub.publish(self.lastStatColor)
                else:
                    self.cor_pub.publish("Vermelho")

                time.sleep(1)
                self.flagApagarRotas = False

            elif self.flagReiniciar:
                os.system('sudo reboot now')
                time.sleep(1)
                self.flagReiniciar = False
            
            elif self.flagDesligar:
                os.system('sudo shutdown now')
                time.sleep(1)
                self.flagDesligar = False

            elif self.flagToggleTeleop:
                self.cor_pub.publish("Laranja")
                if self.flagTeleopON:
                    self.flagTeleopON = False
                    self.navegando = True
                else: 
                    self.flagTeleopON = True
                    self.navegando = False
                time.sleep(1)
                self.flagToggleTeleop = False
                self.cor_pub.publish(self.lastStatColor)
            
            elif self.flagToggleVelo:
                self.cor_pub.publish("Laranja")
                if self.flagVelMax:     # troca para velocidade baixa
                    self.flagVelMax = False
                    self.linear_scale = 0.2
                    self.angular_scale = 0.2
                    self.smothLinear = 0.4
                    self.smothAngular = 0.4
                    
                else:       # troca para velocidade alta
                    self.flagVelMax = True
                    self.linear_scale = 0.7
                    self.angular_scale = 0.4
                    self.smothLinear = 0.08
                    self.smothAngular = 1
                time.sleep(1)
                self.flagToggleVelo = False
                self.cor_pub.publish(self.lastStatColor)
            
            elif self.flagControle:
                self.cor_pub.publish("Laranja")
                if self.flagAlavanca:     # velocidade alta
                    self.flagAlavanca = False
                else:       # velocidade baixa
                    self.flagAlavanca = True
                    
                time.sleep(1)
                self.flagControle = False
                self.cor_pub.publish(self.lastStatColor)

    
                
                

if __name__ == '__main__':
    # Inicializa o nó ROS
    rospy.init_node('teleop_joy_node')
    rate = rospy.Rate(20)
    teleop = JoyTeleop()
    while not rospy.is_shutdown():
        teleop.run()
        rate.sleep()
    # nav_stack_object.cor_pub.publish("Apagar") #apaga a fita led

#!/usr/bin/env python3
import time
import rospy
from log_manager import LogManager
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String, Int32, Bool
import RPi.GPIO as GPIO
import displayOLED
from restart_launch import LaunchManager
from sensor_msgs.msg import BatteryState



class botoeira():
    """
    Essa classe lê os botões pressionados, enviando um destino de a cordo com o botão pressionado.
    Desenvolvido para rotas lineares, do ponto A para o B, do B para o C,... do N para o A.
    
    
    """
    def __init__(self):
        self.SEQ_BUTTON_1 = 17  # VERDE
        self.SEQ_BUTTON_2 = 27  # AZUL
        # self.SEQ_BUTTON_3 = 22  # AMARELO

        GPIO.setwarnings(False)  # Ignore warning for now
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.SEQ_BUTTON_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.SEQ_BUTTON_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        # GPIO.setup(self.SEQ_BUTTON_3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        time.sleep(0.5)

         # Inicialize o display OLED
        # self.displayOled = displayOLED.SSD1306()

        self.has_started = False
        self.reached = True
        self.started_timer = False
        self.once = True
        self.disponivel = False
        self.flagIniciouNavegacao = False
       
        self.tempo_inicial = 0.0
        self.tempo_atual = 0.0
        self.flagPararTempo = False

        self.flagTravado = False

        self.destination = 0
        self.destinoEnviado = False  
        self.last_pressed = 0 
        self.rotaJson = 0

        self.log_manager = LogManager()
        self.resetLaunch = LaunchManager()        
        
        # self.displayOled.clear()  # Limpa o display oled
        # self.displayOled.display_text("ORIGEM:", str(self.botao))  # Exibe texto no display oled
        # self.quantidadeRotas = 0
        self.iniciou = False

        self.voltageBattery = 0.0
        self.flagCarregando = False
        self.flagInitVoltage = False
        self.voltageTotal = 0.0
        self.voltageMeida = 0.0
        self.amostrasMedia = 10
        self.cont = 0  
        self.lastVoltage = 0.0
        self.bateriaFraca = 23.0  
        

        #subscribers
        rospy.Subscriber('has_reached', Bool, self.reached_callback, queue_size=1)
        rospy.Subscriber("lora_message", String, self.lora_callback, queue_size=1)  # publicado pelas placas esp32
        # rospy.Subscriber("/total_rotas", Int32, self.total_rotas_callback, queue_size=1)
        rospy.Subscriber('/battery_state', BatteryState, self.callbackBattery, queue_size=1)

        #publishers
        self.destination_pub = rospy.Publisher('destination', Int32, queue_size=1, latch=True)
        self.cor_pub = rospy.Publisher("/cor", String, queue_size=1, latch=True)  # publica uma cor para o código fitaLed_v2.0.py gerenciar
        self.som_pub = rospy.Publisher("/som", String, queue_size=1, latch=True)  # publica uma som para o código controlaSom.py gerenciar
        #possíveis sons: Pronto - Andando - Parado - Botão
    
    # def total_rotas_callback(self, data):
    #     self.quantidadeRotas = data.data
    #     print(f"total de rotas: {self.quantidadeRotas}")
    
    def reached_callback(self, data):
        self.reached = data.data
      
    def lora_callback(self, data):
        try:
            # b1 #b2 #b3
            msg = data.data
            print(msg)

            if msg[0] == "b":
                msg.split("b")

                ponto = msg[1]

                if self.reached and self.last_pressed != 0: # so recebe via lora após de iniciar o robo e a posição do robo
                    self.button_pressed(int(ponto)-1)
                    # self.reached = False
                    # self.destination = ponto
                    # if not self.on_point:
                    #     self.destination = ponto
                    #     self.reached = False
                # else:
                #     self.sequence_pub.publish(int(self.destination))  # Caso o robo esteja no meio da rota, envia o ultimo botão apertado (envia a mesma rota), usado para a versão de botoeira em que o led fica aceso
            else:
                print(f"Mensagem recebida via protocolo Lora não esperada: {data.data}")
                self.log_manager.gera_log("Mensagem recebida via protocolo Lora não esperada: " + data.data, LogManager.Error)
                return
        except:
            return
        
    def callbackBattery(self, msg):
        """
        Callback que monitora o estado da bateria e muda o LED para vermelho se a carga estiver baixa.
        """
        self.voltageBattery = float(msg.voltage)
        if not self.flagInitVoltage:
            self.flagInitVoltage = True
            self.lastVoltage = self.voltageBattery
            self.voltageMeida = self.voltageBattery
        else:
            if (self.voltageBattery >= self.lastVoltage-0.5) and (self.voltageBattery <= self.lastVoltage+0.5):  #evita leituras outlayer
                self.voltageTotal += self.voltageBattery
                self.cont += 1
                # print(f"cont: {self.cont}")
                # print(f"voltageBattery: {self.voltageBattery}")
                # print(f"voltageTotal: {self.voltageTotal}")
                if self.cont >= self.amostrasMedia:
                    self.voltageMeida = self.voltageTotal / self.amostrasMedia  
                    # print(f"voltageBattery: {self.voltageBattery}")
                    # print(f"voltageMeida: {self.voltageMeida}")              
                    self.voltageTotal = 0.0
                    self.cont = 0
            self.lastVoltage = self.voltageBattery
        
    def dockStation(self):
        self.log_manager.gera_log("Bateria fraca, enviado ao dockstation!", LogManager.Info)
        self.disponivel = False
        self.rotaJson = int(str(self.last_pressed) + str(1))
        rospy.loginfo("destino publicado no tópico /destination: " + str(self.rotaJson))      
        self.destination_pub.publish(self.rotaJson)     
        self.last_pressed = 1 
        self.reached = False
        self.flagIniciouNavegacao = True


    def button_pressed(self, destino):
        """
        Envia a posição inicial do robo.
        Publica no tópico /sequence_botao o ponto de destino, quando o robo está destravado.
        Destrava o robo quando o 'destino' recebido é igual ao local atual do robo.

        Parâmetros:
        -----------
        destino : int
            ponto de destino, que será procurado dentro de uma pasta que contem vários arquivos json. Cada arquivo é uma rota
        """
        # print(f"entrou no button_pressed() {destino}")
        self.som_pub.publish("Botão")
        if self.last_pressed == 0:      # seleciona a posição inicial
            self.once = True
            self.disponivel = True
            self.last_pressed = destino     # seleciona o ponto atual
            self.som_pub.publish("Pronto")
            print(f"Ponto inicial de serviço: {destino}")
            self.log_manager.gera_log("Ponto de inicial de serviço: " + str(destino), LogManager.Info)
            # if (self.voltageMeida < self.bateriaFraca):   # quando está no ponto de dockstation e está com bateria fraca, desliga o robo
            #     self.shutdown_pub.publish("sudo shutdown now")
            
        elif self.last_pressed == destino and self.reached:   # para destravar o robo, quando o robo chega ao destino
            print("robot already on point")
            self.once = True
            self.disponivel = True
            self.som_pub.publish("Pronto")
            if (self.voltageMeida < self.bateriaFraca) and (self.last_pressed != 1):
                self.dockStation()

        elif self.disponivel and (self.last_pressed != destino) and not self.flagCarregando and self.voltageMeida >= self.bateriaFraca:      # envia para o destino
            # print("botão diferente")
            self.disponivel = False
            self.rotaJson = int(str(self.last_pressed) + str(destino))
            rospy.loginfo("destino publicado no tópico /destination: " + str(self.rotaJson))

            self.destination_pub.publish(self.rotaJson)

            self.last_pressed = destino
            # self.on_point = False
            self.reached = False
            self.flagIniciouNavegacao = True
        

    def await_button(self):
        if not self.iniciou:    #executa apenas no início da execução do código
            self.iniciou = True 
            self.tempo_inicial = time.time()
            self.tempo_atual = 0.0   
            
        if not self.flagPararTempo:
            self.tempo_atual = time.time() - self.tempo_inicial # Calcula o tempo decorrido
            self.log_manager.gera_log("Iniciando botoeira_ponto_ponto_mesh_node: " + str(self.tempo_atual) + " segundos.", LogManager.Info)
            if self.tempo_atual >= 30:          # reinicia as launchs caso o move base ainda não tenha iniciado
                self.log_manager.gera_log("Navegação reiniciado por estouro de tempo: " + str(self.tempo_atual) + " segundos.", LogManager.Info)
                # self.resetLaunch.killLaunch()
                time.sleep(5)
        
        if self.reached:   # quando o main_ponto_ponto inicia
            # print(f"total de rotas: {self.quantidadeRotas}")
            if not self.flagPararTempo:
                self.flagPararTempo = True       # Quando recebe um True pelo tópico /has_reached, impede de  chamar "self.resetLaunch.killLaunch()"
                self.cor_pub.publish("Roxo")
                self.som_pub.publish("Pronto")


            if self.flagIniciouNavegacao:  #quando o robo chega ao destino
                self.flagIniciouNavegacao = False
                self.once = True
                self.log_manager.gera_log("Chegou ao destino: " + str(self.last_pressed), LogManager.Info)
                print(f"Chegou ao destino: {self.last_pressed}")
                self.som_pub.publish("Pronto")
                self.cor_pub.publish("Verde")
                time.sleep(1)

            elif not self.disponivel and self.once:  # o robo ja chegou ao destino mas está bloqueado
                self.once = False
                # self.ledstrip_pub.publish("Roxo")
                self.cor_pub.publish("Roxo")

            elif self.disponivel and self.once: # quando é apertado o botão de onde o robo ja está, liberando o robo para outras rotas
                self.once = False
                # self.ledstrip_pub.publish("Verde")
                self.cor_pub.publish("Verde")
                
        if GPIO.input(self.SEQ_BUTTON_1) == GPIO.HIGH:
            # proteção de debound
            time.sleep(0.5)
            print("BOTAO VERDE")
            self.start_time = time.time()    # Marca o tempo inicial
            self.elapsed_time = 0.0
            while GPIO.input(self.SEQ_BUTTON_1) == GPIO.HIGH:
                # time.sleep(0.01)
                self.elapsed_time = time.time() - self.start_time # Calcula o tempo decorrido
                if self.elapsed_time >= 2:
                    self.log_manager.gera_log("Navegação reiniciada atraves do botão verde", LogManager.Info)
                    self.resetLaunch.killLaunch()
                    time.sleep(5)
    
            if self.reached:
                self.button_pressed(1)

        elif GPIO.input(self.SEQ_BUTTON_2) == GPIO.HIGH:
            # proteção de debound
            time.sleep(0.5)
            print("BOTAO AZUL")
            self.start_time = time.time()    # Marca o tempo inicial
            self.elapsed_time = 0.0
            while GPIO.input(self.SEQ_BUTTON_2) == GPIO.HIGH:
                # time.sleep(0.01)
                self.elapsed_time = time.time() - self.start_time # Calcula o tempo decorrido
                if self.elapsed_time >= 2:
                    self.log_manager.gera_log("Navegação reiniciada atraves do botão azul", LogManager.Info)
                    self.resetLaunch.killLaunch()
                    time.sleep(5)
            
            if self.reached:
                self.button_pressed(2)

        # elif GPIO.input(self.SEQ_BUTTON_3) == GPIO.HIGH:
        #     # proteção de debound
        #     time.sleep(0.5)
        #     print("BOTAO AMARELO")
        #     self.start_time = time.time()    # Marca o tempo inicial
        #     self.elapsed_time = 0.0
        #     while GPIO.input(self.SEQ_BUTTON_3) == GPIO.HIGH:
        #         # time.sleep(0.01)
        #         self.elapsed_time = time.time() - self.start_time # Calcula o tempo decorrido
        #         if self.elapsed_time >= 2:
        #             self.log_manager.gera_log("Navegação reiniciada atraves do botão amarelo", LogManager.Info)
        #             self.resetLaunch.killLaunch()
        #             time.sleep(5)
            
        #     if self.reached:
        #         self.button_pressed(3)


if __name__ == '__main__':
    rospy.init_node("botoeira_ponto_ponto_mesh_node")
    objt = botoeira()
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        objt.await_button()
        rate.sleep()
    # objt.displayOled.clear()  # Limpa o display oled
    objt.cor_pub.publish("Apagar") #apaga a fita led


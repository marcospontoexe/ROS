#!/usr/bin/env python3
import time
import rospy
import RPi.GPIO as GPIO
from log_manager import LogManager
from restart_launch import LaunchManager
from std_msgs.msg import Bool, Int32, String
from sensor_msgs.msg import BatteryState


class botoeira:
    def __init__(self):
        self.SEQ_BUTTON_1 = 17  # VERDE
        self.SEQ_BUTTON_2 = 27  # AZUL
        self.SEQ_BUTTON_3 = 22  # AMARELO

        GPIO.setwarnings(False)  # Ignore warning for now
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.SEQ_BUTTON_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.SEQ_BUTTON_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.SEQ_BUTTON_3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        # GPIO.setup(self.SEQ_BUTTON_4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        # GPIO.setup(self.SEQ_BUTTON_5, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


        self.has_started = False
        self.msg = "nopress"

        self.reached = False
        self.disponivel = False
        self.iniciou = False

        # self.started_timer = False
        self.tempo_inicial = 0.0
        self.tempo_atual = 0.0

        self.last_pressed = 0

        self.on_point = True
        self.destination = ""
        self.flagIniciouNavegacao = False
        self.rota = ""
        self.once = True
        self.flagPararTempo = False
        self.log_manager = LogManager()
        self.resetLaunch = LaunchManager()

        self.voltageBattery = 0.0
        self.flagCarregando = False
        self.flagInitVoltage = False
        self.voltageTotal = 0.0
        self.voltageMeida = 0.0
        self.amostrasMedia = 10
        self.cont = 0  
        self.lastVoltage = 0.0
        self.bateriaFraca = 23.0        #23.0

        #publishers
        self.button = rospy.Publisher("button", Int32, queue_size=1, latch=True)  # Tópico usado pelo Main
        # self.sequence_pub = rospy.Publisher("sequence_botao", Int32, queue_size=1)  # publica o destino como um int, que é a KEY 'sequence' do json. Usado pelo goal_manipulation_botoes. Usado pela esp32
        self.cor_pub = rospy.Publisher("/cor", String, queue_size=1, latch=True)  # publica uma cor para o código fitaLed_v2.0.py gerenciar
        # self.ledstrip_pub = rospy.Publisher("LEDs", String, queue_size=10)
        self.som_pub = rospy.Publisher("/som", String, queue_size=1, latch=True)  # publica uma som para o código controlaSom.py gerenciar
        #possíveis sons: Pronto - Andando - Parado - Botão
        self.shutdown_pub = rospy.Publisher('/power_off', String, queue_size=1, latch=True)  # envia comando de shutdown para o nó powerOff_node


        #subscribers
        rospy.Subscriber("has_reached", Bool, self.reached_callback, queue_size=1)
        rospy.Subscriber("lora_message", String, self.lora_callback, queue_size=1)  # publicado pelas placas esp32
        rospy.Subscriber("nomeRota", String, self.nomeRota_callback, queue_size=1)  # publicado pelo main. publica o valor contido na key "name" do json
        rospy.Subscriber('/battery_state', BatteryState, self.callbackBattery, queue_size=1)



    def nomeRota_callback(self, data):
        self.rota = data.data
        print(f"rota recebida pelo tópico: {self.rota}")

    def lora_callback(self, data):
        try:
            # b1 #b2 #b3
            msg = data.data
            print(msg)

            if msg[0] == "b":
                msg.split("b")

                ponto = msg[1]

                if self.reached and self.last_pressed != 0: # so recebe via lora após de iniciar o robo e a posição do robo
                    self.button_pressed(int(ponto))
                    # self.reached = False
                    self.destination = ponto
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

    def reached_callback(self, data):
        self.reached = data.data

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
        rospy.loginfo("destino publicado no tópico /sequence_botao: " + str(2))
        self.button.publish(2)
        self.last_pressed = 2
        self.reached = False
        self.flagIniciouNavegacao = True
    

    def button_pressed(self, ponto_botao):
        """
        Envia a posição inicial do robo.
        Publica no tópico /sequence_botao o ponto de destino, quando o robo está destravado.
        Destrava o robo quando o 'ponto_botao' recebido é igual ao local atual do robo.

        Parâmetros:
        -----------
        ponto_botao : int
            ponto de destino que será procurado na KEY 'sequence' do json
        """
        # print(f"entrou no button_pressed() {ponto_botao}")
        self.som_pub.publish("Botão")
        if self.last_pressed == 0:      # seleciona a posição inicial
            self.once = True
            self.disponivel = True
            self.last_pressed = ponto_botao
            # self.sequence_pub.publish(ponto_botao)
            # time.sleep(0.5) # para dar tempo de atualizar o tópico sequence_botao antes de enviar o 'start' no tópico button
            self.button.publish(ponto_botao)
            self.som_pub.publish("Pronto")
            # if (self.voltageMeida < self.bateriaFraca):   # quando está no ponto de dockstation e está com bateria fraca, desliga o robo
            #     self.shutdown_pub.publish("sudo shutdown now")
            

        elif self.last_pressed == ponto_botao and self.reached:   # para destravar o robo, quando o robo chega ao destino
            print("robot already on point")
            # self.ledstrip_pub.publish("Verde")
            # self.cor_pub.publish("Verde")
            # self.on_point = True
            self.once = True
            self.disponivel = True
            self.som_pub.publish("Pronto")
            if (self.voltageMeida < self.bateriaFraca) and (self.last_pressed != 2):
                self.dockStation()

        elif self.disponivel and (self.last_pressed != ponto_botao) and not self.flagCarregando and self.voltageMeida >= self.bateriaFraca:      # envia para o destino
            # print("botão diferente")
            self.disponivel = False
            # self.sequence_pub.publish(ponto_botao)
            rospy.loginfo("destino publicado no tópico /sequence_botao: " + str(ponto_botao))
            # time.sleep(0.5) # para dar tempo de atualizar o tópico sequence_botao antes de enviar o 'start' no tópico button
            # self.destination = ponto_botao

            self.button.publish(ponto_botao)

            self.last_pressed = ponto_botao
            # self.on_point = False
            self.reached = False
            self.flagIniciouNavegacao = True
            # self.log_manager.gera_log("Chamando o robô pela botoeira: " + str(ponto_botao), LogManager.Info)

    def await_button(self):
        if not self.iniciou:  # executa apenas no início da execução do código
            self.iniciou = True
            self.tempo_inicial = time.time()
            self.tempo_atual = 0.0
            # time.sleep(1)  # delay para dar tempo do move_base subir, para usaro o /move_base/clear_costmaps
            # self.ledstrip_pub.publish("Verde")
            # self.cor_pub.publish("Roxo")
            # self.on_point = True
            # self.disponivel = True
            # self.once = True
        if not self.flagPararTempo:
            self.tempo_atual = time.time() - self.tempo_inicial # Calcula o tempo decorrido
            # print(f"tempo: {self.tempo_atual}")
            self.log_manager.gera_log("Iniciando botoeira_global_planner_mesh_dinamica_node: " + str(self.tempo_atual) + " segundos.", LogManager.Info)
            if self.tempo_atual >= 30:
                self.log_manager.gera_log("Navegação reiniciado por estouro de tempo: " + str(self.tempo_atual) + " segundos.", LogManager.Info)
                # self.resetLaunch.killLaunch()
                time.sleep(5)
            
        if not self.flagCarregando and self.voltageMeida > 27.3:
            self.flagCarregando = True
            print("Carregador de bateria conectado.")
            self.log_manager.gera_log("Carregador de bateria conectado.", LogManager.Info)
        elif self.flagCarregando and self.voltageMeida <= 27.3:
            self.flagCarregando = False
            print("Carregador de bateria desonectado.")
            self.log_manager.gera_log("Carregador de bateria desconectado.", LogManager.Info)


        if self.reached:
            if not self.flagPararTempo:
                self.flagPararTempo = True      # Quando recebe um True pelo tópico /has_reached, impede de  chamar "self.resetLaunch.killLaunch()"
        
            if self.flagIniciouNavegacao:  # chegou ao destino
                self.flagIniciouNavegacao = False
                self.once = True
                self.log_manager.gera_log("Chegou ao destino: " + self.rota, LogManager.Info)
                print(f"Chegou ao destino: {self.rota}")
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
                ponto_botao = 2
                self.button_pressed(ponto_botao)

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
                ponto_botao = 3
                self.button_pressed(ponto_botao)

        elif GPIO.input(self.SEQ_BUTTON_3) == GPIO.HIGH:
            # proteção de debound
            time.sleep(0.5)
            print("BOTAO AMARELO")
            self.start_time = time.time()    # Marca o tempo inicial
            self.elapsed_time = 0.0
            while GPIO.input(self.SEQ_BUTTON_3) == GPIO.HIGH:
                # time.sleep(0.01)
                self.elapsed_time = time.time() - self.start_time # Calcula o tempo decorrido
                if self.elapsed_time >= 2:
                    self.log_manager.gera_log("Navegação reiniciada atraves do botão amarelo", LogManager.Info)
                    self.resetLaunch.killLaunch()
                    time.sleep(5)
            
            if self.reached:
                ponto_botao = 4
                self.button_pressed(ponto_botao)

        # elif GPIO.input(self.SEQ_BUTTON_4) == GPIO.HIGH:
        #     # proteção de debound
        #     time.sleep(0.5)
        #     print("BOTAO BRANCO")
        #     self.start_time = time.time()    # Marca o tempo inicial
        #     self.elapsed_time = 0.0
        #     while GPIO.input(self.SEQ_BUTTON_4) == GPIO.HIGH:
        #         # time.sleep(0.01)
        #         self.elapsed_time = time.time() - self.start_time # Calcula o tempo decorrido
        #         if self.elapsed_time >= 2:
        #             self.log_manager.gera_log("Navegação reiniciada atraves do botão branco", LogManager.Info)
        #             self.resetLaunch.killLaunch()
        #             time.sleep(5)

        #     if self.reached:
        #         ponto_botao = 5
        #         self.button_pressed(ponto_botao)

        # elif GPIO.input(self.SEQ_BUTTON_5) == GPIO.HIGH:
        #     # proteção de debound
        #     time.sleep(0.5)
        #     print("BOTAO PRETO")
        #     self.start_time = time.time()    # Marca o tempo inicial
        #     self.elapsed_time = 0.0
        #     while GPIO.input(self.SEQ_BUTTON_5) == GPIO.HIGH:
        #         # time.sleep(0.01)
        #         self.elapsed_time = time.time() - self.start_time # Calcula o tempo decorrido
        #         if self.elapsed_time >= 2:
        #             self.log_manager.gera_log("Navegação reiniciada atraves do botão preto", LogManager.Info)
        #             self.resetLaunch.killLaunch()
        #             time.sleep(5)

        #     if self.reached:
        #         ponto_botao = 6
        #         self.button_pressed(ponto_botao)



if __name__ == "__main__":
    rospy.init_node("botoeira_global_planner_mesh_dinamica_node", anonymous=True)
    objt = botoeira()
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        objt.await_button()
        rate.sleep()
    objt.cor_pub.publish("Apagar") #apaga a fita led
    # print("APAGAR LED")
    GPIO.cleanup()

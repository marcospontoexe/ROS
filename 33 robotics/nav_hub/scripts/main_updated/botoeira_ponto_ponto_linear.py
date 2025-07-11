#!/usr/bin/env python3
import time
import rospy
import PontoDestino
from log_manager import LogManager
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String, Int32, Bool
import RPi.GPIO as GPIO
import displayOLED
from restart_launch import LaunchManager


class botoeira():
    """
    Essa classe lê os botões pressionados, enviando um destino de a cordo com o botão pressionado.
    Desenvolvido para rotas lineares, do ponto A para o B, do B para o C,... do N para o A.
    
    
    """
    def __init__(self):
        self.DOWN_BUTTON = 17  # VERDE   
        self.SEQ_BUTTON = 27  # AZUL
        self.UP_BUTTON = 22  # AMARELO


        GPIO.setwarnings(False) # Ignore warning for now
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.SEQ_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.UP_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.DOWN_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        time.sleep(0.5)

         # Inicialize o display OLED
        # self.displayOled = displayOLED.SSD1306()

        self.has_started = False
        self.reached = True
        self.started_timer = False
        self.main_started = False
        self.tempo_inicial = 0.0
        self.tempo_atual = 0.0
        self.flagPararTempo = False

        self.flagTravado = False

        self.pontoSalvo = PontoDestino.PontoDestino()

        self.destination = 0
        self.destinoEnviado = False   

        self.log_manager = LogManager()
        self.resetLaunch = LaunchManager()

        self.botao = self.pontoSalvo.abreArquivo("pontosDestino")
        
        # self.displayOled.clear()  # Limpa o display oled
        # self.displayOled.display_text("ORIGEM:", str(self.botao))  # Exibe texto no display oled
        self.quantidadeRotas = 0
        self.iniciou = False
        

        #subscribers
        rospy.Subscriber('has_reached', Bool, self.reached_callback, queue_size=1)
        rospy.Subscriber("lora_message", String, self.lora_callback, queue_size=1)  # publicado pelas placas esp32
        rospy.Subscriber("/total_rotas", Int32, self.total_rotas_callback, queue_size=1)
        

        #publishers
        self.destination_pub = rospy.Publisher('destination', Int32, queue_size=1, latch=True)
        self.cor_pub = rospy.Publisher("/cor", String, queue_size=1, latch=True)  # publica uma cor para o código fitaLed_v2.0.py gerenciar
        self.som_pub = rospy.Publisher("/som", String, queue_size=1, latch=True)  # publica uma som para o código controlaSom.py gerenciar
        #possíveis sons: Pronto - Andando - Parado - Botão
    
    def total_rotas_callback(self, data):
        self.quantidadeRotas = data.data
        print(f"total de rotas: {self.quantidadeRotas}")
    
    def reached_callback(self, data):
        self.reached = data.data
        # print(f"recebido no tópico /has_reached: {self.reached}")
        if not self.main_started:
            self.main_started = True

    def lora_callback(self, data):
        try:
            # b1 #b2 #b3
            msg = data.data
            print(msg)

            if msg[0] == "b":
                msg.split("b")
                ponto = msg[1]
                # if ponto == '2': # envia para o ponto inicial (demonstração metal kraft)                           
                #     if self.botao != 1:    # se o robo não estiver no ponto inicial
                #        if self.reached and not self.flagTravado:
                #             self.cor_pub.publish("Roxo") 
                #             self.som_pub.publish("Botão") 
                #             self.flagTravado = True
                #             while(not self.pontoSalvo.salvaArquivo("pontosDestino", 1)):
                #                 time.sleep(0.1)
                #             time.sleep(0.5) #para dar tempo de manipular o arquivo txt
                           
                #             print(f"Enviado ao destino: {1}")
                #             self.log_manager.gera_log("Enviado ao destino: " + str(1), LogManager.Info)
                #             self.destination = 1
                #             self.destinoEnviado = True 
                #             self.reached = False
                #             self.has_started = True 
                #             print(f"botao: {self.botao * 100}")
                #             self.destination_pub.publish(self.botao * 100)
                #             self.botao = 1
                            
                # if ponto == '2': #decrementar o ponto de ínicio                            
                #     if not self.has_started:    #enquanto o botão de start não for apertado
                #         self.botao -= 1
                #         if self.botao < 1:
                #             self.botao = self.quantidadeRotas
                #         print(f"ponto recebido: {self.botao}")
                #         self.log_manager.gera_log("Ponto de inicial de serviço: " + str(self.botao), LogManager.Info)
                #         # self.displayOled.clear()  # Limpa o display oled
                #         # self.displayOled.display_text("ORIGEM:", str(self.botao))  # Exibe texto no display oled
                # elif ponto == '3':
                #     if self.reached:
                #         destino = self.botao + 1
                #         if destino > self.quantidadeRotas:
                #             destino = 1
                #         while(not self.pontoSalvo.salvaArquivo("pontosDestino", destino)):
                #             time.sleep(0.1)
                #         time.sleep(0.5) #para dar tempo de manipular o arquivo txt
                #         # self.displayOled.clear()  # Limpa o display oled
                #         # self.displayOled.display_text("DESTINO:", str(destino))  # Exibe texto no display oled
                #         self.button_pressed(self.botao)
                        
                #         self.botao = self.botao+1 if self.botao < self.quantidadeRotas else 1
                #         # print(f"botão: {self.botao}") 
                if ponto == '2':
                    # if self.has_started:    #somente depois que o  botão de start for apertado
                    self.button_pressed(self.botao)
                    self.botao = self.botao+1 if self.botao < self.quantidadeRotas else 1
                    # self.displayOled.clear()  # Limpa o display oled
                    # self.displayOled.display_text("ORIGEM:", str(self.botao))  # Exibe texto no display oled
                    
            else:
                print(f"Mensagem recebida via protocolo Lora não esperada: {data.data}")
                self.log_manager.gera_log("Mensagem recebida via protocolo Lora não esperada: " + data.data, LogManager.Error)
                return
        except:
            return

    def button_pressed(self, destino):
        self.cor_pub.publish("Laranja") 
        self.som_pub.publish("Botão") 

        self.flagTravado = True
        
        self.destination = destino+1
        if self.destination > self.quantidadeRotas:
            self.destination = 1
        print(f"Enviado ao destino: {self.destination}")
        self.log_manager.gera_log("Enviado ao destino: " + str(self.destination), LogManager.Info)
        self.destinoEnviado = True 
        self.reached = False
        self.destination_pub.publish(destino)
        
        # self.log_manager.gera_log("Robô esta indo ao ponto: " + msg, LogManager.Info)
        self.has_started = True

        # while(not self.pontoSalvo.salvaArquivo("pontosDestino", self.destination)):
        #     time.sleep(0.1)
        # time.sleep(0.5) #para dar tempo de manipular o arquivo txt
        # self.displayOled.clear()  # Limpa o display oled
        # self.displayOled.display_text("DESTINO:", str(self.destination))  # Exibe texto no display oled
        

    def await_button(self):
        if not self.iniciou:    #executa apenas no início da execução do código
            self.iniciou = True 
            self.tempo_inicial = time.time()
            self.tempo_atual = 0.0   
            
        if not self.flagPararTempo:
            self.tempo_atual = time.time() - self.tempo_inicial # Calcula o tempo decorrido
            self.log_manager.gera_log("Iniciando botoeira_ponto_ponto_linear_node: " + str(self.tempo_atual) + " segundos.", LogManager.Info)
            if self.tempo_atual >= 30:          # reinicia as launchs caso o move base ainda não tenha iniciado
                self.log_manager.gera_log("Navegação reiniciado por estouro de tempo: " + str(self.tempo_atual) + " segundos.", LogManager.Info)
                # self.resetLaunch.killLaunch()
                time.sleep(5)
        
        if self.main_started:   # quando o main_ponto_ponto inicia
            # print(f"total de rotas: {self.quantidadeRotas}")
            if not self.flagPararTempo:
                self.flagPararTempo = True       # Quando recebe um True pelo tópico /has_reached, impede de  chamar "self.resetLaunch.killLaunch()"
                print(f"ponto recebido: {self.botao}")

            if self.destinoEnviado and self.reached:   #quando o robo chega ao destino
                self.flagTravado = False    # apagar essa linha. Elimina o efeito de travar o robo ao chegar no destino
                self.destinoEnviado = False   
                self.log_manager.gera_log("Chegou ao destino: " + str(self.destination), LogManager.Info)
                print(f"Chegou ao destino: {self.destination}")
                self.som_pub.publish("Pronto")
                self.cor_pub.publish("Roxo")
                time.sleep(1)
                
            #botão para incrementar o ponto de ínicio
            if GPIO.input(self.UP_BUTTON) == GPIO.HIGH:
                # print("botaõ de incremento apertado!")
                #proteção de debound
                time.sleep(0.5) 
                self.start_time = time.time()    # Marca o tempo inicial
                self.elapsed_time = 0.0 
                while GPIO.input(self.UP_BUTTON) == GPIO.HIGH:
                    # time.sleep(0.1)
                    self.elapsed_time = time.time() - self.start_time # Calcula o tempo decorrido
                    if self.elapsed_time >= 2:
                        self.resetLaunch.killLaunch()
                        time.sleep(5)
                    
                if not self.has_started:    #enquanto o botão de start não for apertado
                    self.botao += 1
                    if self.botao > self.quantidadeRotas:
                        self.botao = 1
                    print(f"ponto recebido: {self.botao}")
                    self.log_manager.gera_log("Ponto de inicial de serviço: " + str(self.botao), LogManager.Info)
                    # self.displayOled.clear()  # Limpa o display oled
                    # self.displayOled.display_text("ORIGEM:", str(self.botao))  # Exibe texto no display oled

            #botão para decrementar o ponto de ínicio
            if GPIO.input(self.DOWN_BUTTON) == GPIO.HIGH:
                # print("botaõ de decremento apertado!")
                #proteção de debound
                time.sleep(0.5)  
                self.start_time = time.time()    # Marca o tempo inicial
                self.elapsed_time = 0.0
                while GPIO.input(self.DOWN_BUTTON) == GPIO.HIGH:
                    # time.sleep(0.1)
                    self.elapsed_time = time.time() - self.start_time # Calcula o tempo decorrido
                    if self.elapsed_time >= 2:
                        self.resetLaunch.killLaunch()
                        time.sleep(5)
                    
                if not self.has_started:    #enquanto o botão de start não for apertado
                    self.botao -= 1
                    if self.botao < 1:
                        self.botao = self.quantidadeRotas
                    print(f"ponto recebido: {self.botao}")
                    self.log_manager.gera_log("Ponto de inicial de serviço: " + str(self.botao), LogManager.Info)
                    # self.displayOled.clear()  # Limpa o display oled
                    # self.displayOled.display_text("ORIGEM:", str(self.botao))  # Exibe texto no display oled

            # Botão para enviar ao próximo destino
            if GPIO.input(self.SEQ_BUTTON) == GPIO.HIGH:
                # print("botaõ apertado")
                #proteção de debound
                time.sleep(0.5)  
                self.start_time = time.time()    # Marca o tempo inicial
                self.elapsed_time = 0.0
                while GPIO.input(self.SEQ_BUTTON) == GPIO.HIGH:
                    # time.sleep(0.01)
                    self.elapsed_time = time.time() - self.start_time # Calcula o tempo decorrido
                    if self.elapsed_time >= 2:
                        self.resetLaunch.killLaunch()
                        time.sleep(5)

                # print(f"self.botao: {self.botao}")
                
                if self.reached:
                    #lógicac para travar o robo ao chegar no destino
                    # if  self.flagTravado:
                    #     self.flagTravado = False
                    #     self.som_pub.publish("Pronto")
                    #     self.cor_pub.publish("Verde")
                    # else:
                    #     self.button_pressed(self.botao)
                        
                    #     self.botao = self.botao+1 if self.botao < self.quantidadeRotas else 1
                    #     # print(f"botão: {self.botao}") 
                    
                    #lógica sem travar o robo ao chegar no destino
                    self.button_pressed(self.botao)
                    self.botao = self.botao+1 if self.botao < self.quantidadeRotas else 1
                    # print(f"botão: {self.botao}")

        

if __name__ == '__main__':
    rospy.init_node("botoeira_ponto_ponto_node")
    objt = botoeira()
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        objt.await_button()
        rate.sleep()
    # objt.displayOled.clear()  # Limpa o display oled
    objt.cor_pub.publish("Apagar") #apaga a fita led


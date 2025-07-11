#!/usr/bin/env python3
import time
import rospy
import RPi.GPIO as GPIO
from log_manager import LogManager
from restart_launch import LaunchManager
from std_msgs.msg import Bool, Int32, String



class botoeira:
    def __init__(self):
        self.DOWN_BUTTON = 17  # VERDE
        self.SEQ_BUTTON = 27  # AZUL
        self.UP_BUTTON = 22  # AMARELO

        GPIO.setwarnings(False)  # Ignore warning for now
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.DOWN_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.SEQ_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.UP_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        self.log_manager = LogManager()
        self.resetLaunch = LaunchManager()

        self.has_started = False
        self.msg = "nopress"

        self.reached = False
        self.disponivel = False
        self.iniciou = False

        # self.started_timer = False
        self.tempo_inicial = 0.0
        self.tempo_atual = 0.0

        self.botao = 1
        self.destino = 1
        self.quantidadeRotas = 1
        
        self.destination = ""
        self.flagIniciouNavegacao = False
        self.rota = ""
        self.flagSeguindo = False
        self.once = True
        self.flagPararTempo = False
        self.flagFirstRout = True
        

        #publishers
        self.button = rospy.Publisher("button", Int32, queue_size=1, latch=True)  # Tópico usado pelo Main
        # self.sequence_pub = rospy.Publisher("sequence_botao", Int32, queue_size=1)  # publica o destino como um int, que é a KEY 'sequence' do json. Usado pelo goal_manipulation_botoes. Usado pela esp32
        self.cor_pub = rospy.Publisher("/cor", String, queue_size=1, latch=True)  # publica uma cor para o código fitaLed_v2.0.py gerenciar
        # self.ledstrip_pub = rospy.Publisher("LEDs", String, queue_size=10)
        self.som_pub = rospy.Publisher("/som", String, queue_size=1, latch=True)  # publica uma som para o código controlaSom.py gerenciar
        #possíveis sons: Pronto - Andando - Parado - Botão

        #subscribers
        rospy.Subscriber("has_reached", Bool, self.reached_callback, queue_size=1)
        rospy.Subscriber("lora_message", String, self.lora_callback, queue_size=1)  # publicado pelas placas esp32
        rospy.Subscriber("nomeRota", String, self.nomeRota_callback, queue_size=1)  # publicado pelo main. publica o valor contido na key "name" do json
        rospy.Subscriber("follower", Bool, self.callbackFollower, queue_size=1)   # quado o robo está seguindo alguem, não aceita comando das botoeiras
        rospy.Subscriber("/quantidade_rotas", Int32, self.QuantidadeRotas, queue_size=1) # recebe a quantidade de rotas pelo main

    def nomeRota_callback(self, data):
        self.rota = data.data
        print(f"rota recebida pelo tópico: {self.rota}")
    
    def QuantidadeRotas(self, data):
        self.quantidadeRotas = data.data
        print(f"quantidade rotas: {self.quantidadeRotas}")

    def lora_callback(self, data):
        try:
            # b1 #b2 #b3
            msg = data.data
            print(msg)

            # if msg[0] == "b":
            #     msg.split("b")

            #     ponto = msg[1]

            #     if self.reached and self.last_pressed != 0: # so recebe via lora após de iniciar o robo e a posição do robo
            #         self.button_pressed(int(ponto))
            #         # self.reached = False
            #         self.destination = ponto
            #         # if not self.on_point:
            #         #     self.destination = ponto
            #         #     self.reached = False
            #     # else:
            #     #     self.sequence_pub.publish(int(self.destination))  # Caso o robo esteja no meio da rota, envia o ultimo botão apertado (envia a mesma rota), usado para a versão de botoeira em que o led fica aceso
            # else:
            #     print(f"Mensagem recebida via protocolo Lora não esperada: {data.data}")
            #     self.log_manager.gera_log("Mensagem recebida via protocolo Lora não esperada: " + data.data, LogManager.Error)
            #     return
        except:
            return

    def reached_callback(self, data):
        self.reached = data.data
        

    def callbackFollower(self, data):
        '''True quando está seguindo algum
        False quando não está senguindo
        '''
        
        self.flagSeguindo = data.data
        # print(f"Callback self.flagSeguindo: {self.flagSeguindo}")
        # if not self.flagSeguindo:
        #     self.disponivel = True
        # else: 
        #     self.disponivel = True
    


    def button_pressed(self, ponto_botao):
        """
        Verifica se o botão apertado corresponde a um ponto válido, e então publica o ponto de destino no tópico /sequence_botao

        Parâmetros:
        -----------
        ponto_botao : int
            ponto de destino que será procurado na KEY 'sequence' do json
        """
        # print(f"entrou no button_pressed() {ponto_botao}")
        self.cor_pub.publish("Laranja") 
        self.som_pub.publish("Botão") 
        
        
        self.reached = False
        self.button.publish(ponto_botao + 1)     # somar 1 porque o "sequence" da primeira rota é 2
        
        # self.log_manager.gera_log("Robô esta indo ao ponto: " + msg, LogManager.Info)
        self.has_started = True
        self.flagIniciouNavegacao = True


    def await_button(self):
        if not self.iniciou:  # executa apenas no início da execução do código
            self.iniciou = True
            self.tempo_inicial = time.time()
            self.tempo_atual = 0.0
            

        if not self.flagPararTempo:
            self.tempo_atual = time.time() - self.tempo_inicial # Calcula o tempo decorrido
            # print(f"tempo: {self.tempo_atual}")
            self.log_manager.gera_log("Iniciando botoeira_global_planner_linear_dinamica_node: " + str(self.tempo_atual) + " segundos.", LogManager.Info)
            if self.tempo_atual >= 30:
                self.log_manager.gera_log("Navegação reiniciado por estouro de tempo: " + str(self.tempo_atual) + " segundos.", LogManager.Info)
                # self.resetLaunch.killLaunch()
                time.sleep(5)

        if self.reached:
            if not self.flagPararTempo:
                self.flagPararTempo = True   # Quando recebe um True pelo tópico /has_reached, impede de  chamar "self.resetLaunch.killLaunch()"
                self.cor_pub.publish("Verde")  
                
            if self.flagIniciouNavegacao:  # chegou ao destino
                self.flagIniciouNavegacao = False
                self.log_manager.gera_log("Chegou ao destino: " + self.rota, LogManager.Info)
                print(f"Chegou ao destino: {self.rota}")
                self.som_pub.publish("Pronto")
                self.cor_pub.publish("Verde")
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
                    self.cor_pub.publish("Laranja") 
                    self.som_pub.publish("Botão") 
                    time.sleep(1) 
                    self.cor_pub.publish("Verde") 
                    self.botao += 1
                    if self.botao > self.quantidadeRotas:
                        self.botao = 1
                    print(f"ponto recebido: {self.botao + 1}")      # somar 1 porque o "sequence" da primeira rota é 2
                    self.log_manager.gera_log("Ponto de inicial de serviço: " + str(self.botao+1), LogManager.Info)
                    
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
                    self.cor_pub.publish("Laranja") 
                    self.som_pub.publish("Botão") 
                    time.sleep(1) 
                    self.cor_pub.publish("Verde") 
                    self.botao -= 1
                    if self.botao < 1:
                        self.botao = self.quantidadeRotas
                    print(f"ponto recebido: {self.botao + 1}")  # somar 1 porque o "sequence" da primeira rota é 2
                    self.log_manager.gera_log("Ponto de inicial de serviço: " + str(self.botao+1), LogManager.Info)
                    

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
                    if self.flagFirstRout:
                        self.flagFirstRout = False
                        self.button_pressed(self.botao) # envia o ponto para configurar a posição e orientação inicial
                        time.sleep(3)   # aguarda para coonfigurar a posição inicial
                        self.botao = self.botao+1 if self.botao < self.quantidadeRotas else 1   # incrementa a rota

                    self.button_pressed(self.botao) # envia o ponto como sendo o destino
                    self.botao = self.botao+1 if self.botao < self.quantidadeRotas else 1   # incrementa a rota

        
if __name__ == "__main__":
    rospy.init_node("botoeira_global_planner_linear_dinamica_node", anonymous=True)
    objt = botoeira()
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        objt.await_button()
        rate.sleep()
    objt.cor_pub.publish("Apagar") #apaga a fita led
    # print("APAGAR LED")
    GPIO.cleanup()

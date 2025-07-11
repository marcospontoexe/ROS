#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from log_manager import LogManager
import ledstrip


class LedController:
    '''
    Essa classe controla a fita led não endereçavel, usada na primeira versão do mobby (pharmy),
    juntamente com o "ledstrip driver" conectado à raspberry.
    
    '''
    def __init__(self):
        rospy.init_node("fitaLed_node", anonymous=True)

        # Configuração dos pinos GPIO e inicialização da fita LED
        CLK = 23  # Pino do clock da fita LED
        DAT = 24  # Pino de dados da fita LED 
        self.fitaLed = ledstrip.LEDStrip(CLK, DAT)

        #  para gerenciar a bateria
        self.battery_low = False
        self.voltageBattery = None
        self.voltageTotal = 0.0
        self.amostrasMedia = 10
        self.cont = 0

        self.corRecebida = False
        self.cor = "Apagado"
        self.flagInit = False
        self.lastVoltage = 0.0

        self.log_manager = LogManager()
        self.vel = Twist()
        self.vel.angular.z = 0
        self.vel.linear.x = 0
      
        self.ledstrip_pub = rospy.Publisher('LEDs', String, queue_size=10)  # para publica uma cor para a fita led
        '''Cores para o tópico LEDs
        Branco - Laranja -  Amarelo - Azul - Verde - Roxo - Ciano - Vermelho - Apagado
        '''

        rospy.Subscriber('/battery_state', BatteryState, self.callbackBattery, queue_size=1)
        rospy.Subscriber('cor', String, self.callback_cor, queue_size=1)  # recebe uma string contendo uma cor
        
        

    def callbackBattery(self, msg):
        """
        Callback que monitora o estado da bateria e muda o LED para vermelho se a carga estiver baixa.
        """
        self.voltageBattery = msg.voltage
        if not self.flagInit:
            self.flagInit = True
            self.lastVoltage = self.voltageBattery
    
    def callback_cor(self, msg):
        self.cor = msg.data
        # print(f"mensagem recebida no tópico cor: {self.cor}")
        self.corRecebida = True
        

    def main(self):
        if self.flagInit:         # começa a calcular apenas depois de receber algo no tópico /battery_state 
            if (self.voltageBattery >= self.lastVoltage-0.5) and (self.voltageBattery <= self.lastVoltage+0.5):  #evita leituras outlayer
                self.voltageTotal += self.voltageBattery
                self.cont += 1
                # print(f"cont: {self.cont}")
                # print(f"voltageBattery: {self.voltageBattery}")
                # print(f"voltageTotal: {self.voltageTotal}")
                if self.cont >= self.amostrasMedia:
                    self.voltageTotal = self.voltageTotal / self.amostrasMedia  
                    # print(f"voltageBattery: {self.voltageBattery}")
                    # print(f"voltageTotal: {self.voltageTotal}")              
                    if self.voltageBattery <= 24.0:
                        self.battery_low = True
                        # self.strip.setcolourred()
                        self.log_manager.gera_log("Bateria fraca, favor carregar: " + str(self.voltageTotal) + " Volts", LogManager.Warn)
                    else:
                        self.battery_low = False

                    self.voltageTotal = 0.0
                    self.cont = 0
                self.lastVoltage = self.voltageBattery
        
        # if self.corRecebida:    # para publicar no tópico /LEDs apenas quando receber um cor diferente
        #     self.corRecebida = False
        #     if self.battery_low:
        #         self.ledstrip_pub.publish("Vermelho")
        #     else:
        #         self.ledstrip_pub.publish(self.cor)
        if self.battery_low:
            if self.cor == "Apagado":
                self.fitaLed.setcolouroff()
            else:
                self.fitaLed.setcolourred()
            # print("bateria fraca")
        else:
            # print("bateria boa")
            if self.cor == "Laranja":
                self.fitaLed.setcolourrgb(255, 60, 0)
            elif self.cor == "Amarelo":
                self.fitaLed.setcolourrgb(255, 120, 10)
            elif self.cor == "Azul":
                self.fitaLed.setcolourblue()
            elif self.cor == "Verde":
                self.fitaLed.setcolourgreen()
            elif self.cor == "Roxo":
                self.fitaLed.setcolourrgb(255, 0, 80)
            elif self.cor == "Ciano":
                self.fitaLed.setcolourhex("00FFFF")
            elif self.cor == "Vermelho":
                self.fitaLed.setcolourred()
            elif self.cor == "Branco":
                self.fitaLed.setcolourwhite()
            else:   # fita apagada
                self.fitaLed.setcolouroff()


if __name__ == '__main__':
    try:
        objt = LedController()
        rate = rospy.Rate(2)
    except rospy.ROSInterruptException:
        rospy.loginfo("Erro ao iniciar o objeto LedController")
        pass
    
    while not rospy.is_shutdown():
        objt.main()
        rate.sleep()
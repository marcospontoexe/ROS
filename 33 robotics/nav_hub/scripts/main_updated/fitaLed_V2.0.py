#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from log_manager import LogManager

class LedController:
    '''
    Essa classe controla a fita led endereçavel, usada na versão atual mobby, conectada ao arduino.
    '''
    def __init__(self):
        rospy.init_node("fitaLed_node", anonymous=True)

        #  para gerenciar a bateria
        self.battery_low = False
        self.voltageBattery = None
        self.voltageTotal = 0.0
        self.amostrasMedia = 10
        self.cont = 0
        self.flagInit = False
        self.lastVoltage = 0.0

        self.corRecebida = False
        self.cor = "Apagado"

        self.log_manager = LogManager()
      
        self.ledstrip_pub = rospy.Publisher('LEDs', String, queue_size=2)  # para publica uma cor para a fita led
        '''Cores para o tópico LEDs
        Branco - Laranja -  Amarelo - Azul - Verde - Roxo - Ciano - Vermelho - Apagado
        '''
        
        rospy.Subscriber('/battery_state', BatteryState, self.callbackBattery, queue_size=1)
        rospy.Subscriber('cor', String, self.callback_cor, queue_size=1)  # recebe uma string contendo uma cor, e publica essa cor no tópico LEDs caso a bateria não esteja baixa

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
                    if self.voltageTotal <= 24:
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
                self.ledstrip_pub.publish(self.cor)
            else:
                self.ledstrip_pub.publish("Vermelho")
            # print("bateria fraca")
        else:
            # print("bateria boa")
            self.ledstrip_pub.publish(self.cor)


if __name__ == '__main__':
    try:
        objt = LedController()
        rate = rospy.Rate(2)
    except rospy.ROSInterruptException:
        rospy.loginfo("Erro ao iniciar o objeto LedController")
        pass
    
    while not rospy.is_shutdown():
        objt.main()     # a cor precisa ser publicada periodicamente, pois as vezes o arduino não consegue escutar o tópico via rosserial
        rate.sleep()
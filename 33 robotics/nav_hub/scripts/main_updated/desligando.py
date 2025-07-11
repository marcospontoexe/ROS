#!/usr/bin/env python3


import time
import rospy
from std_msgs.msg import String
import os
from log_manager import LogManager

class Desligando(object):

    def __init__(self):
        rospy.init_node("powerOff_node", anonymous=True)
        self.log_manager = LogManager()
        # Subscribers
        self.command_sub = rospy.Subscriber('/power_off', String, self.powerOff_callback)  # Recebe uma mensagem "off" para desligar o sistema
        
    def powerOff_callback(self, msg):
        command = msg.data
        rospy.loginfo("Executando o comando: {}".format(command))  # Compatível com Python 2 e 3
        self.log_manager.gera_log("Executando o comando: " + command, LogManager.Info)
        os.system('rostopic pub -1 /cor std_msgs/String "Apagar"')  #publica uma unica vez
        os.system(command)

    def run(self):
        # Mantém o nó ativo para receber mensagens e publicar periodicamente
        rospy.spin()

if __name__ == '__main__':
    try:
        power_object = Desligando()
        rospy.loginfo("Inicializando PowerOff_node...")
        power_object.run()
    except rospy.ROSInterruptException:
        pass

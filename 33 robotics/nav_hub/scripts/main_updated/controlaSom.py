#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Twist
import pygame
# from pygame import mixer
from log_manager import LogManager



class ControlaSom():

    def __init__(self):
        rospy.init_node("controlaSom_node", anonymous=True)

        #subscribers
        rospy.Subscriber('/som', String, self.callback_som, queue_size=1)

        self.tocarSom = True
        self.som = None

        # Inicializar o mixer do pygame
        pygame.mixer.init()

        pygame.mixer.music.set_volume(0.9)

        self.log_manager = LogManager()
        self.cor_atual = "Verde"
        self.path = '/home/ubuntu/catkin_ws/src/nav_hub/scripts/som/'
        
        # Carregar o arquivo de som 
        # self.chegou = pygame.mixer.music.load(self.path+'chegou.wav')
        # self.andando = pygame.mixer.music.load(self.path+'andando_editado2.wav')
        # self.parado = pygame.mixer.music.load(self.path+'parado.wav')
        # self.espera_botao = pygame.mixer.music.load(self.path+'espera_botao.wav') 

    def callback_som(self, data):
        self.som = data.data 


    def main(self):
        if pygame.mixer.get_busy() == False:
            if self.som == "Pronto":
                pygame.mixer.music.load(self.path+'chegou.wav')
                pygame.mixer.music.set_volume(0.9)
                pygame.mixer.music.play()
                # self.chegou.play()
                self.som = "Mute"
            elif self.som == "Andando":
                pygame.mixer.music.load(self.path+'andando_editado2.wav')
                pygame.mixer.music.set_volume(0.9)
                pygame.mixer.music.play()
                # self.chegou.play()

            elif self.som == "Parado":
                pygame.mixer.music.load(self.path+'parado.wav')
                pygame.mixer.music.set_volume(0.9)
                pygame.mixer.music.play()
                # self.chegou.play()

            elif self.som == "Botão":
                pygame.mixer.music.load(self.path+'espera_botao.wav')
                pygame.mixer.music.set_volume(0.9)
                pygame.mixer.music.play()
                # self.chegou.play()
                self.som = "Mute"
            

if __name__ == '__main__':
    try:
        objt = ControlaSom()
        rate = rospy.Rate(0.5)
    except rospy.ROSInterruptException:
        rospy.loginfo("Erro ao iniciar o objeto ControlaSom")
        pass
    
    while not rospy.is_shutdown():
        objt.main()
        rate.sleep()
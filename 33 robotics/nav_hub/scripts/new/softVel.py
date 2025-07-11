#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time
import math

class Vel:
    def __init__(self):
        """
        Essa classe recebe uma velocidade do tópico /cmd_vel_controller
        e publica novamnete no tópico /cmd_vel de forma a suavizar os trancos
        """
        

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
 
        rospy.Subscriber("/soft_cmd_vel", Twist, self.vel_callback, queue_size=1)       # para realizar testes
        # rospy.Subscriber("/cmd_vel", Twist, self.vel_callback, queue_size=1)

        self.velocity = Twist()
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0

        self.lastVel = Twist()
        self.lastVel.linear.x = 0
        self.lastVel.angular.z = 0
        
        
    def vel_callback(self, data):
        self.velocity = data

    def smooth_transition(self, vel_inicial, vel_final, duration):
        """
        Faz uma transição linear de velocidade inicial para velocidade final em 'duration' segundos,
        publicando em 'pub' a cada 1/rate_hz segundos.

        :param vel_inicial: geometry_msgs.msg.Twist inicial
        :param vel_final: geometry_msgs.msg.Twist final
        :param duration: tempo total da transição (segundos)
        """

        freq_pub = 100   # frequência de publicação

        # Número de passos
        steps = round(duration * freq_pub)
        if steps < 1:
            # Se duração for muito curta, publica diretamente vel_final
            pub.publish(vel_final)
            return

        # Diferenças totais
        delta_lin_x = vel_final.linear.x    - vel_inicial.linear.x
        delta_lin_y = vel_final.linear.y    - vel_inicial.linear.y
        delta_lin_z = vel_final.linear.z    - vel_inicial.linear.z
        delta_ang_x = vel_final.angular.x   - vel_inicial.angular.x
        delta_ang_y = vel_final.angular.y   - vel_inicial.angular.y
        delta_ang_z = vel_final.angular.z   - vel_inicial.angular.z

        print(f"vel_inicial.linear.x; {vel_inicial.linear.x}")
        print(f"vel_inicial.angular.z; {vel_inicial.angular.z}")
        print(f"vel_final.linear.x; {vel_final.linear.x}")
        print(f"vel_final.angular.z; {vel_final.angular.z}")
        print(f"delta_lin_x; {delta_lin_x}")
        print(f"delta_ang_z; {delta_ang_z}")
        print("\n")
        twist = Twist()

        
        for i in range(1, steps + 1):
            alpha = float(i) / steps  # fração de progresso de 0 (início) a 1 (fim)

            # aplica uma função de interpolação à quinta potencia suave (Smootherstep) tipo ease-in-out à aceleração (6α^5 − 15α^4 + 10α^3)
            interpolation = 6*alpha**5 - 15*alpha**4 + 10*alpha**3  # Smootherstep

            # aplica uma função de interpolação cúbica suave (Smoothstep) tipo ease-in-out à aceleração (3α² - 2α³)
            # interpolation = 3 * alpha**2 - 2 * alpha**3  # Smoothstep

            # interpolation = 2 * (1 / (1 + math.exp(-1 * (alpha)))) - 1 # sigmoid (1 / (1 + math.exp(-k * (x))) ) -> k controla a inclinação da reta, e t controla o deslocamento no eixo X
            # interpolation = (1 / (1 + math.exp(-1 * (alpha))))  # sigmoid (1 / (1 + math.exp(-k * (x))) ) -> k controla a inclinação da reta, e t controla o deslocamento no eixo X
            # interpolation = ((1 - math.exp(-2 * (alpha))) / (1 + math.exp(-2 * (alpha))))  # sigmoid (1 / (1 + math.exp(-k * (x))) ) -> k controla a inclinação da reta, e t controla o deslocamento no eixo X

            # interpolation = alpha**4 

            #interpolação quadrática desacelerada (rápida no início, lenta no fim) 
            # interpolation = (1 - (1 - alpha)**6) 

            # print(f"smoother_alpha; {smoother_alpha}")
            
            # Interpolação linear em cada componente
            twist.linear.x    = vel_inicial.linear.x  + delta_lin_x * interpolation
            twist.linear.y    = vel_inicial.linear.y  + delta_lin_y * interpolation
            twist.linear.z    = vel_inicial.linear.z  + delta_lin_z * interpolation
            twist.angular.x   = vel_inicial.angular.x + delta_ang_x * interpolation
            twist.angular.y   = vel_inicial.angular.y + delta_ang_y * interpolation
            twist.angular.z   = vel_inicial.angular.z + delta_ang_z * interpolation

            # print(f"twist.linear.x: {twist.linear.x}")
            # print(f"twist.angular.z: {twist.angular.z}")
            # print("\n")
            self.cmd_vel_pub.publish(twist)
            time.sleep(1/freq_pub)

        return twist

    def main(self):
        
        self.lastVel = self.smooth_transition(self.lastVel, self.velocity, 2)

        # time.sleep(0.1)
        
if __name__ == '__main__':
    rospy.init_node("softVel_node", anonymous=True)
    objt = Vel()

# if __name__ == '__main__':
#     try:
#         objt = Vel()
#         rate = rospy.Rate(1)
#     except rospy.ROSInterruptException:
#         rospy.loginfo("Erro ao iniciar o objeto Vel()")
#         pass
    
#     while not rospy.is_shutdown():
#         objt.main()
#         rate.sleep()
#!/usr/bin/env python3
import rospy
import math
import time
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import os


class AdjustVelocity:
    def __init__(self):
        rospy.init_node("VelocityController_node", anonymous=True)

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1, tcp_nodelay=True)
 
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber("/cmd_vel_controller", Twist, self.vel_callback, queue_size=1)
        # rospy.Subscriber("/cmd_vel", Twist, self.vel_callback)


        self.velocity = Twist()
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0
        self.vel = Twist()
        self.angMinRight = 0    # angulo inicial do intervalo angular do lado direito
        self.angMaxRight = 45    # angulo final do intervalo angular do lado direito
        self.angMinFront = 46    # angulo inicial do intervalo angular frontal
        self.angMaxFront = 134    # angulo final do intervalo angular frontal
        self.angMinLeft = 135    # angulo inicial do intervalo angular do lado esquerdo
        self.angMaxLeft = 180    # angulo final do intervalo angular do lado direiesquerdoto

        self.laser = None
        self.iniciouLaser = False
        self.initialRight = 0
        self.finalRight = 0  
        self.initialFront = 0
        self.finalFront = 0
        self.initialLeft = 0
        self.finalLeft = 0 
        self.LateralDistMin = 0.8  # 1.2  distancia lateral mínima de segurança, para se afastar da parede
        self.frontDistMin = 0.5 # 0.8 distancia frontal mínima de segurança para o robo parar antes de colidir

        self.stop_requested = False

    def scan_callback(self, data):
        self.laser = data
        
        # print(f"self.iniciouLaser: {self.iniciouLaser}")
        if not self.iniciouLaser:
            # print(f"self.laser\n: {self.laser}")
            
            self.iniciouLaser = True
            incrementoGrau = (self.laser.angle_increment) * (180 / math.pi)
            anguloInicial = self.laser.angle_min
            self.initialRight = int((45 / incrementoGrau) + (self.angMinRight / incrementoGrau)) # índice inicial do intervalo angular do lado direito
            self.finalRight = int((45 / incrementoGrau) + (self.angMaxRight / incrementoGrau))  # índice final do intervalo angular do lado direito
            self.initialFront = int((45 / incrementoGrau) + (self.angMinFront / incrementoGrau)) # índice inicial do intervalo angular frontal
            self.finalFront = int((45 / incrementoGrau) + (self.angMaxFront / incrementoGrau)) # índice final do intervalo angular frontal
            self.initialLeft = int((45 / incrementoGrau) + (self.angMinLeft / incrementoGrau))   # índice inicial do intervalo angular do lado esquerdo
            self.finalLeft = int((45 / incrementoGrau) + (self.angMaxLeft / incrementoGrau)) # índice final do intervalo angular do lado esquerdo
            
            # print(f"incrementoGrau: {incrementoGrau}")
            # print(f"anguloInicial: {anguloInicial}")
            # print(f"initialRight: {self.initialRight}")
            # print(f"self.finalRight: {self.finalRight}")
            # print(f"self.initialFront: {self.initialFront}")
            # print(f"self.finalFront: {self.finalFront}")
            # print(f"self.initialLeft: {self.initialLeft}")
            # print(f"self.finalLeft: {self.finalLeft}")
        
        
        if min(self.laser.ranges[self.initialFront:self.finalFront]) < self.frontDistMin:
            self.stop_requested = True
        else:
            self.stop_requested = False

        
    def vel_callback(self, data):
        self.velocity = data
        # print(f"self.velocity: {self.velocity}")


    def main(self):
        if self.iniciouLaser:  
            # print(f"initialRight: {self.initialRight}")
            # print(f"self.finalRight: {self.finalRight}")
            # print(f"self.initialFront: {self.initialFront}")
            # print(f"self.finalFront: {self.finalFront}")
            # print(f"self.initialLeft: {self.initialLeft}")
            # print(f"self.finalLeft: {self.finalLeft}")

            if self.stop_requested:
                # publica zero **sem** dormir a thread
                # print(f"minFrontSide: {min(self.laser.ranges[self.initialFront:self.finalFront])}")
                self.cmd_vel_pub.publish(Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)))
            else:
                 # rightList = self.laser.ranges[self.initialRight:self.finalRight]   # obtem as distancias do lado direito 
                MinRightSide = min(self.laser.ranges[self.initialRight:self.finalRight])    # menor distancia do lado direito 
                indexRightSide = self.laser.ranges[self.initialRight:self.finalRight].index(MinRightSide)       # índice onde ocorreu a menor distancia
                
                
                # leftList = self.laser.ranges[self.finalLeft-1:self.initialLeft-1:-1]      # obtem as distancias do lado esquerdo 
                minLeftSide = min(self.laser.ranges[self.finalLeft-1:self.initialLeft-1:-1])    # obtem as distancias do lado esquerdo 
                indexLeftSide = self.laser.ranges[self.finalLeft-1:self.initialLeft-1:-1].index(minLeftSide)       # índice onde ocorreu a menor distancia

               
                print(f"MinRightSide: {MinRightSide}")
                print(f"indexRightSide: {indexRightSide}")
                print("\n")
            
               
                print(f"minLeftSide: {minLeftSide}")
                print(f"indexLeftSide: {indexLeftSide}")
                print("\n")
                
                if (MinRightSide <= self.LateralDistMin) and (MinRightSide < minLeftSide):   #parede detectada do lado direito, deve se afastar para o lado esqeurdo
                    print("direita")
                    print(f"MinRightSide: {MinRightSide}")
                    print(f"indexRightSide: {indexRightSide}")
                    print("\n")
                    self.vel.linear.x = self.velocity.linear.x * 0.7
                    self.vel.angular.z = (indexRightSide/len(self.laser.ranges[self.initialRight:self.finalRight])) * 0.3
                elif (minLeftSide <= self.LateralDistMin) and (minLeftSide < MinRightSide):   #parede detectada do lado esquerdo, deve se afastar para o lado direito
                    print("esquerda")
                    print(f"minLeftSide: {minLeftSide}")
                    print(f"indexLeftSide: {indexLeftSide}")
                    print("\n")
                    self.vel.linear.x = self.velocity.linear.x * 0.7
                    self.vel.angular.z = -1 * (indexLeftSide/len(self.laser.ranges[self.finalLeft-1:self.initialLeft-1:-1]))  * 0.3
                else:
                    # print("else")
                    self.vel.linear.x = self.velocity.linear.x
                    self.vel.angular.z = self.velocity.angular.z
                

                print(f"vel.linear.x: {self.vel.linear.x}")
                print(f"vel.angular.z: {self.vel.angular.z}")
                print("\n")
              
                self.cmd_vel_pub.publish(self.vel)
                # time.sleep(0.1)  # Pequeno delay para garantir que o comando foi processado
           
            # laser = self.laser.ranges
            # # vel = Twist()

            # # print(f"initialRight: {self.initialRight}")
            # # print(f"self.finalRight: {self.finalRight}")
            # # print(f"self.initialFront: {self.initialFront}")
            # # print(f"self.finalFront: {self.finalFront}")
            # # print(f"self.initialLeft: {self.initialLeft}")
            # # print(f"self.finalLeft: {self.finalLeft}")

            # # print(f"laser: {laser}")
            # # print("\n")

            # # dist = min(self.laser.ranges[self.initialFront:self.finalFront])


            # # frontList = self.laser.ranges[self.initialFront:self.finalFront]     # obtem as distancias frontais  
            # minFrontSide = min(self.laser.ranges[self.initialFront:self.finalFront])    # menor distancia do lado direito 
            # indexFrontSide = self.laser.ranges[self.initialFront:self.finalFront].index(minFrontSide)       # índice onde ocorreu a menor distancia

            # if min(self.laser.ranges[self.initialFront:self.finalFront]) < self.frontDistMin:
            # # if minFrontSide < self.frontDistMin:
            #     # print("frente")
            #     # print(f"frontList: {frontList}")
            #     # print(f"frontList len: {len(frontList)}")
            #     # print(f"minFrontSide: {min(self.laser.ranges[self.initialFront:self.finalFront])}")
            #     # print(f"indexFrontSide: {indexFrontSide}")
            #     # print("\n")
            #     # vel.linear.x = 0
            #     # vel.angular.z = 0
            #     print(f"minFrontSide: {min(self.laser.ranges[self.initialFront:self.finalFront])}")
            #     # self.cmd_vel_pub.publish(vel)
            #     self.cmd_vel_pub.publish(Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)))
            #     # os.system("rostopic pub 1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'")
            #     print(f"minFrontSide: {min(self.laser.ranges[self.initialFront:self.finalFront])}")
            #     # time.sleep(2.0)  # Pequeno delay para garantir que o comando foi processado
            #     # self.cmd_vel_pub.publish(vel)
            # else:         
            #     # rightList = self.laser.ranges[self.initialRight:self.finalRight]   # obtem as distancias do lado direito 
            #     MinRightSide = min(self.laser.ranges[self.initialRight:self.finalRight])    # menor distancia do lado direito 
            #     indexRightSide = self.laser.ranges[self.initialRight:self.finalRight].index(MinRightSide)       # índice onde ocorreu a menor distancia
                
                
            #     # leftList = self.laser.ranges[self.finalLeft-1:self.initialLeft-1:-1]      # obtem as distancias do lado esquerdo 
            #     minLeftSide = min(self.laser.ranges[self.finalLeft-1:self.initialLeft-1:-1])    # obtem as distancias do lado esquerdo 
            #     indexLeftSide = self.laser.ranges[self.finalLeft-1:self.initialLeft-1:-1].index(minLeftSide)       # índice onde ocorreu a menor distancia

            #     # print(f"rightList: {rightList}")
            #     # print(f"rightList len: {len(rightList)}")
            #     # print(f"MinRightSide: {MinRightSide}")
            #     # print(f"indexRightSide: {indexRightSide}")
            #     # print("\n")
            #     # print(f"frontList: {frontList}")
            #     # print(f"frontList len: {len(frontList)}")
            #     # print(f"minFrontSide: {minFrontSide}")
            #     # print(f"indexFrontSide: {indexFrontSide}")
            #     # print("\n")
            #     # print(f"leftList: {leftList}")
            #     # print(f"leftList len: {len(leftList)}")
            #     # print(f"minLeftSide: {minLeftSide}")
            #     # print(f"indexLeftSide: {indexLeftSide}")
            #     # print("\n")
                
            #     if (MinRightSide <= self.LateralDistMin) and (MinRightSide < minLeftSide):   #parede detectada do lado direito, deve se afastar para o lado esqeurdo
            #         # print("direita")
            #         # print(f"rightList: {rightList}")
            #         # print(f"rightList len: {len(rightList)}")
            #         # print(f"MinRightSide: {MinRightSide}")
            #         # print(f"indexRightSide: {indexRightSide}")
            #         # print("\n")
            #         self.vel.linear.x = self.velocity.linear.x * 0.7
            #         self.vel.angular.z = (indexRightSide/len(self.laser.ranges[self.initialRight:self.finalRight])) * 0.3
            #     elif (minLeftSide <= self.LateralDistMin) and (minLeftSide < MinRightSide):   #parede detectada do lado esquerdo, deve se afastar para o lado direito
            #         # print("esquerda")
            #         # print(f"leftList: {leftList}")
            #         # print(f"leftList len: {len(leftList)}")
            #         # print(f"minLeftSide: {minLeftSide}")
            #         # print(f"indexLeftSide: {indexLeftSide}")
            #         # print("\n")
            #         self.vel.linear.x = self.velocity.linear.x * 0.7
            #         self.vel.angular.z = -1 * (indexLeftSide/len(self.laser.ranges[self.finalLeft-1:self.initialLeft-1:-1]))  * 0.3
            #     else:
            #         # print("else")
            #         self.vel.linear.x = self.velocity.linear.x
            #         self.vel.angular.z = self.velocity.angular.z
                

            #     # print(f"vel.linear.x: {vel.linear.x}")
            #     # print(f"vel.angular.z: {vel.angular.z}")
            #     # print("\n")
            #     # while self.cmd_vel_pub.get_num_connections() < 1:
            #     #     rospy.loginfo("TENTANDO se conectar ao tópico /cmd_vel")
            #     #     time.sleep(0.1)
            #     self.cmd_vel_pub.publish(self.vel)
            #     # time.sleep(0.1)  # Pequeno delay para garantir que o comando foi processado


if __name__ == '__main__':
    try:
        objt = AdjustVelocity()
        rate = rospy.Rate(50)
    except rospy.ROSInterruptException:
        rospy.loginfo("Erro ao iniciar o objeto AdjustVelocity()")
        pass
    
    while not rospy.is_shutdown():
        objt.main()
        rate.sleep()
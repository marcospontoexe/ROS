#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

class SafetyController:
    def adjust_for_obstacles(self, twist, laser_scan):
        """
        Ajusta o comando twist com base nas leituras do LIDAR, criando zonas:
          - Setor direito (0° a 75°): se obstáculo a <50cm, gera comando para virar à esquerda.
          - Setor esquerdo (105° a 180°): se obstáculo a <50cm, gera comando para virar à direita.
          - Setor frontal (75° a 105°): se obstáculo a <30cm, zera a velocidade linear (parada).
          
        O efeito é progressivo: a 50cm não há alteração, a 30cm há o máximo.
        """
        # Se não houver dados do scan, retorna sem alteração
        if laser_scan is None:
            return twist

        
        # Obter os ângulos (remapeando para que -45° se torne 0°)
        num_samples = len(laser_scan.ranges)
        # Cria um vetor de ângulos (em graus) a partir dos dados do LIDAR
        angles_rad = np.linspace(laser_scan.angle_min, laser_scan.angle_max, num_samples)
        angles_deg = np.degrees(angles_rad)

        
        # print(f"angles_rad: {angles_rad}")
        # print(f"angles_deg: {angles_deg}")
        
        ranges = np.array(laser_scan.ranges)
        
        # Define as zonas (em graus)
        right_mask = (angles_deg >= 0) & (angles_deg < 45)
        frontal_mask = (angles_deg >= 45) & (angles_deg <= 135)
        left_mask = (angles_deg > 135) & (angles_deg <= 180)

        # print(f"right_mask: {right_mask}")
        # print(f"frontal_mask: {frontal_mask}")
        # print(f"left_mask: {left_mask}")

        # menor valor para a máscara frontal
        frontal_min = np.min(ranges[frontal_mask]) if np.any(frontal_mask) else np.inf

        # Para zona frontal: efeito de parada se menor que 30cm.
        frontal_stop = 1   # metros

        # Se obstáculo frontal muito perto, força a parada linear
        if frontal_min < frontal_stop:
            twist.linear.x = 0  # anula velocidade linear
            rospy.loginfo("parando")
            return twist
        
        #print(f"laser_scan.ranges: {laser_scan.ranges}")
        right_min = np.min(ranges[right_mask]) if np.any(right_mask) else np.inf        
        # menor valor e seu angulo para a máscara da direita
        if np.any(right_mask):  # Verifica se há algum valor válido na zona direita
            right_indices = np.where(right_mask)[0]  # Índices dos pontos na zona direita
            #print(f"right_indices: {right_indices}")
            min_index_in_right_mask = np.argmin(ranges[right_mask])  # Índice do menor valor dentro da máscara
            right_min = ranges[right_indices[min_index_in_right_mask]]  # Menor valor da zona direita
            right_min_angle = angles_deg[right_indices[min_index_in_right_mask]]  # Ângulo correspondente
            #print(f"right_min: {right_min}")
            #print(f"right_min_angle: {right_min_angle}")
        else:
            right_min = np.inf
            right_min_angle = None  # Indica que não há ângulo válido
        

        # menor valor e seu angulo para a máscara da esquerda
        if np.any(left_mask):  # Verifica se há algum valor válido na zona esquerda
            left_indices = np.where(left_mask)[0]  # Índices dos pontos na zona esquerda
            min_index_in_left_mask = np.argmin(ranges[left_mask])  # Índice do menor valor dentro da máscara
            left_min = ranges[left_indices[min_index_in_left_mask]]  # Menor valor da zona esquerda
            left_min_angle = angles_deg[left_indices[min_index_in_left_mask]]  # Ângulo correspondente
        else:
            left_min = np.inf
            left_min_angle = None  # Indica que não há ângulo válido
        
        
        # Parâmetros para efeito progressivo:
        # Para zonas laterais: efeito começa em 50cm e é máximo em 30cm.
        lateral_start = 0.70  # metros
        lateral_middle = 0.55  # metros
        lateral_stop  = 0.40  # metros
        
        
        # Inicializa ajustes
        linear_adjust = 0.0
        angular_adjust = 0.0

        # Se obstáculo na zona direita:
        if right_min < lateral_middle:
            angular_adjust = twist.angular.z + (twist.angular.z * (1 / (right_min_angle / 60)))
            linear_adjust = -twist.linear.x * 0.4

        # Se obstáculo na zona esquerda:
        if left_min < lateral_middle:
            angular_adjust = twist.angular.z - (twist.angular.z / (1/(left_min_angle / 180)))
            linear_adjust = -twist.linear.x * 0.4
        
        twist.linear.x = max(0.0, twist.linear.x + linear_adjust)
        # Aplica os ajustes (somando aos comandos já calculados)
        twist.angular.z += angular_adjust
        
        return twist
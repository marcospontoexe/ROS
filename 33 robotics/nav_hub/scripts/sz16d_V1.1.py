#!/usr/bin/env python3
# encoding: utf-8
import rospy
import serial
import math
import os
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray

# serial_port = serial.Serial('/dev/lidar', 38400, timeout=5)
serial_port = serial.Serial('/dev/lidar', 57600, timeout=5)

command = b"\x90\x00\x18\xEB"          #x90 é leitura única
# command = b"\x91\x00\x2B\xDA"         #x91 é leitura contínua
# command = b"\xA0\x00\x1D\x7E"         #xA0 é para interromper a leitura

measures = []   # Lista para armazenar as medidas lidas via serial

def writeSerial():
    """
    Envia um comando para o LIDAR através da porta serial.

    Trata exceções que possam ocorrer durante a escrita na porta serial para evitar interromper a execução e imprime uma
    mensagem de erro se necessário.
    """
    try:
        #print(serial_port)
        serial_port.write(command)
    except:
        print("Writing error!\n")
    # print("executou write!!")

def readSerial():
    """
    Lê dados da porta serial e processa as medidas do LIDAR.

    """
    global measures
    data = serial_port.read(size=1513)
    
    # Verifica a versão do Python para aplicar a codificação correta
    if isinstance(data, str):   # Para Python 2
        data = data.encode("hex")
    else:                       # Para Python 3
        data = data.hex()
    
    # print(f"data: {data}")
    # Converte os dados hexadecimais lidos em medidas em metros
    measures = []
    # print(f"dados de data: {data}")
    for i in range(18,3022,4):
        data2 = data[i:(i+4)]   
        # print(f"data2: {data2}")        
        measures.append(float(int(data2, 16))/1000)
        # print(f"dados {i}: {float(int(data2, 16))/1000}")
    
    # print(f"tamanho antes: {len(measures)}")
    # usa apenas os ídices 22 ao 718, para não pegas as pilastras
    # measures = measures[22:718]
    # print(f"tamanho depois: {len(measures)}")


def publishScan():
    """
    Configura a mensagem `LaserScan` com os dados lidos do LIDAR e publica no tópico apropriado
    Publica os dados de `LaserScan` no tópico `/scan` e a menor distância no tópico `/scan_min`.    .
    """
    global measures
    minsures = []
    #print('a ' + str(measures))

    #caso tenhamos que trocar a porta, o measures só vai ser carregado na proxima iteração, então fazemos esse cara 
    #ser pulado também, para nao dar erro pela falta de items na lista
    if len(measures) == 0:
        return
    
    #Para ignorar as pilastras (COMENTAR O FOR QUANDO ESTIVER FAZENDO NOVOS MAPAS)
    # for nums in range(len(measures)):
    #    if (nums >= 210 and nums <= 250) or (nums >= 490 and nums <= 540):
    #        measures[nums] = 0
    
    # Dados do tópico /scan
    sz16d = LaserScan()
    sz16d.header.frame_id = "keyence_laser"
    sz16d.header.stamp = rospy.Time.now()
    sz16d.ranges = measures
    sz16d.range_min = 0.01  # Distância mínima de detecção
    sz16d.range_max = 16     # Distância máxima de detecção
    sz16d.angle_increment = 0.006283185 # Incremento angular em radiano

    # Para campo de visão completo (270°)
    sz16d.angle_min = -0.785398 + float((sz16d.angle_increment*0))        # Ângulo mínimo do laser (-0,785398 = -45°)
    sz16d.angle_max = float(sz16d.angle_min) + (float(sz16d.angle_increment) * 750)    # Ângulo máximo do laser
    
    #para os ídices 22 ao 718;
    # sz16d.angle_min = -0.785398 + float((sz16d.angle_increment*22))        # Ângulo mínimo do laser (-0,785398 = -45°)
    # sz16d.angle_max = float(sz16d.angle_min) + (float(sz16d.angle_increment) * 718)    # Ângulo máximo do laser

    pub_laserscan.publish(sz16d)


if __name__ == "__main__":  
   
    rospy.init_node("laserscan_sz16D_V1_1_node")
    rospy.loginfo("Init LaserScan - SZ-16D")
    pub_laserscan = rospy.Publisher("/scan",LaserScan, queue_size=1)
    rate = rospy.Rate(20)
    
    
    while not rospy.is_shutdown():
        # print("loop")
        writeSerial()
        readSerial()
        publishScan()
        rate.sleep()





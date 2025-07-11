#!/usr/bin/env python3
# encoding: utf-8
import math
import os

import rospy
import serial
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Float64

serial_port = serial.Serial("/dev/lidar", 125000, timeout=5)
command = b"\x90\x00\x18\xEB"  # Comando para iniciar o lidar

measures = []  # Lista para armazenar as medidas lidas via serial


def calculate_coordinates(measures, num_points=750, range_degrees=270):
    """
    Calcula as coordenadas (x, y) a partir das medidas de distância do LIDAR.

    Parâmetros:
    -----------
    measures : list
        Lista contendo as medidas de distância do LIDAR.

    num_points : int, opcional
        Número de pontos na varredura do LIDAR (default é 750).

    range_degrees : int, opcional
        Ângulo de varredura do LIDAR em graus (default é 270).

    Retorna:
    --------
    list
        Lista de tuplas (x, y) representando as coordenadas calculadas.

    """
    # Resolução dos ângulos definida pelo range total e o numero de pontos
    angular_resolution = range_degrees / num_points

    coordinates = []

    for i in range(num_points):
        angle = math.radians(i * angular_resolution)
        # Distância no ponto i
        distance = measures[i]
        # Convertem a distância para coordenadas no plano cartesiano.
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)

        coordinates.append((x, y))

    return coordinates


def writeSerial():
    """
    Envia um comando para o LIDAR através da porta serial.

    Trata exceções que possam ocorrer durante a escrita na porta serial para evitar interromper a execução e imprime uma
    mensagem de erro se necessário.
    """
    try:
        # print(serial_port)
        serial_port.write(command)
    except:
        print("Writing error!\n")


def readSerial():
    """
    Lê dados da porta serial e processa as medidas do LIDAR.

    """
    global measures
    data = serial_port.read(size=1513)

    # Verifica a versão do Python para aplicar a codificação correta
    if isinstance(data, str):  # Para Python 2
        data = data.encode("hex")
    else:  # Para Python 3
        data = data.hex()

    # Converte os dados hexadecimais lidos em medidas em metros
    measures = []
    # print(f"dados de data: {data}")
    for i in range(18, 3022, 4):
        data2 = data[i : (i + 4)]
        measures.append(float(int(data2, 16)) / 1000)
        # print(f"dados {i}: {float(int(data2, 16))/1000}")

    # print(f"tamanho antes: {len(measures)}")
    # usa apenas os ídices 22 ao 718, para não pegas as pilastras
    measures = measures[22:718]
    # print(f"tamanho depois: {len(measures)}")


def publishScan():
    """
    Configura a mensagem `LaserScan` com os dados lidos do LIDAR e publica no tópico apropriado
    Publica os dados de `LaserScan` no tópico `/scan` e a menor distância no tópico `/scan_min`.    .
    """
    global measures
    minsures = []
    # print('a ' + str(measures))

    # caso tenhamos que trocar a porta, o measures só vai ser carregado na proxima iteração, então fazemos esse cara
    # ser pulado também, para nao dar erro pela falta de items na lista
    if len(measures) == 0:
        return

    # Para ignorar as pilastras (COMENTAR O FOR QUANDO ESTIVER FAZENDO NOVOS MAPAS)
    # for nums in range(len(measures)):
    # #    print(f"índice: {nums} | destância: {measures[nums]}")
    #     if (nums <= 18) or (nums >= 724):
    #         measures[nums] = 0

    # Dados do tópico /scan
    sz16d = LaserScan()
    sz16d.header.frame_id = "keyence_laser"
    sz16d.header.stamp = rospy.Time.now()
    sz16d.ranges = measures
    # sz16d.angle_min = 1.5708        # Ângulo mínimo do laser
    # sz16d.angle_max = 6.28319       # Ângulo máximo do laser
    # sz16d.angle_increment = 0.006283185 # Incremento angular
    # para os ídices 22 ao 718;
    sz16d.angle_increment = 0.006283185  # Incremento angular
    sz16d.angle_min = -0.785398 + float(
        (sz16d.angle_increment * 22)
    )  # Ângulo mínimo do laser (-0,785398 = -45°)
    sz16d.angle_max = float(sz16d.angle_min) + (
        float(sz16d.angle_increment) * 718
    )  # Ângulo máximo do laser

    sz16d.range_min = 0.01  # Distância mínima de detecção
    sz16d.range_max = 16  # Distância máxima de detecção

    # encontra o menor valor
    for nums in range(len(measures)):
        # 313 e 439
        # 275 e 462
        if nums >= 303 and nums <= 447:
            minsures.append(measures[nums])
        else:
            pass

    min_measures = min(minsures)  # Encontra o menor valor na lista
    indice_menor_valor = minsures.index(
        min_measures
    )  # Encontra o índice do menor valor

    msg.data = [min_measures, float(indice_menor_valor)]

    pub_laserscan.publish(sz16d)
    min_laserscan.publish(msg)


def turn_to_enter(calc, draw_angle):
    """
    Determina a direção para o robô se virar com base no cálculo e no ângulo.

    Parâmetros:
    -----------
    calc : int
        Cálculo de posição do robô.

    draw_angle : float
        Ângulo de desenho para determinar a virada.

    Observações:
    ------------
    Esta função está atualmente incompleta e precisa de implementação.
    """
    if calc < (750 / 2) - (draw_angle / 2):
        pass
        # vire até entrar
    elif calc < (750 / 2) - (draw_angle / 2):
        # vire até entrar
        pass


# Função para determinar onde está uma gaveta ou objeto similar
def where_drawer():
    """
    Determina se o robô está posicionado corretamente para desenhar com base nas medidas do LIDAR.

    Usa as medições do LIDAR para identificar se o robô está na posição correta para encaixe e realiza cálculos para
    ajustar a direção se necessário.
    """
    global measures

    # attempt 1 - find the distance when it's in front of me

    # Inicializa listas para armazenar medições dentro e fora do ângulo de desenho
    minsures_draw_high = []
    minsures_draw_low = []

    # Define os parâmetros para a área de desenho
    open_angle = 90
    draw_distance = 0.4

    # Converte ângulos e define limites de ângulo
    draw_angle = open_angle * 270 / 750
    draw_angle_max = (750 / 2) + (draw_angle / 2)
    draw_angle_min = (750 / 2) - (draw_angle / 2)

    # Encontra distâncias em regiões específicas
    for nums in range(len(measures)):
        if nums > draw_angle_max:
            minsures_draw_high.append(measures[nums])

        if nums < draw_angle_min:
            minsures_draw_low.append(measures[nums])
        else:
            pass

    # Verifica se a menor distância dentro do ângulo de desenho é menor que a distância de desenho
    if (
        min(minsures_draw_high) < draw_distance
        and min(minsures_draw_high) < draw_distance
    ):
        print("do things")

    # attempt 2 - find the distance when it's not

    # Segunda tentativa para encontrar a distância entre as pernas do objeto
    coords = calculate_coordinates(measures)
    distance_of_legs = 40
    x_calc = 0
    y_calc = 0

    # Calcula a distância entre os pontos e decide como proceder
    for x in coords:
        for y in coords:
            distance = math.sqrt((y[0] - x[0]) ** 2 + (y[1] - x[1]) ** 2)

            # Se a distância calculada for igual à distância dos "pés"
            if distance == distance_of_legs:
                determine = x_calc - y_calc

                # Ajusta a direção do robô com base na diferença calculada
                if determine < 0:
                    turn_to_enter(y_calc, draw_angle)
                if determine > 0:
                    turn_to_enter(x_calc, draw_angle)

            y_calc += 1

        x_calc += 1


if __name__ == "__main__":
    """
    Inicializa o nó ROS chamado `laserscan_sz16D_node`.

    Configura os publicadores para os tópicos `/scan` e `/scan_min` e entra em um loop principal onde realiza leitura
    da porta serial, publicação dos dados processados e espera um curto período antes de repetir o processo.
    """
    rospy.init_node("laserscan_sz16D_node")
    rospy.loginfo("Init LaserScan - SZ-16D")
    pub_laserscan = rospy.Publisher("/scan", LaserScan, queue_size=10)
    min_laserscan = rospy.Publisher(
        "/scan_min", Float32MultiArray, queue_size=10
    )  # publica a menor distância e o seu índice
    rate = rospy.Rate(20)
    # Cria a mensagem do tipo Float32MultiArray
    msg = Float32MultiArray()
    while not rospy.is_shutdown():
        writeSerial()
        readSerial()
        publishScan()
        rate.sleep()

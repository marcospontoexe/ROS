#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from my_turtlebot_localization.srv import MyServiceMessage, MyServiceMessageResponse
import tf
import os

spots = {}

def handle_save_spot(req):
    global spots
    if req.label == "end":
        with open("spots.txt", "w") as f:
            for label, pose in spots.items():
                f.write(f"{label}: {pose}\n")
        return MyServiceMessageResponse(True, "File saved successfully")
    else:
        try:
            listener = tf.TransformListener() #Cria um objeto da biblioteca TF (Transform) do ROS. Este objeto é usado para ouvir e consultar transformações entre diferentes frames de referência no sistema.
            listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(4.0)) #Espera até que uma transformação entre os frames /map e /base_link esteja disponível. Esta chamada espera até 4 segundos para a transformação estar disponível.
            (trans, rot) = listener.lookupTransform("/map", "/base_link", rospy.Time(0)) #Obtém a transformação entre os frames /map e /base_link no momento atual (rospy.Time(0)). 
            spots[req.label] = {"position": trans, "orientation": rot}  #Armazena a posição e orientação do robô no dicionário spots com a chave sendo o rótulo fornecido (req.label). O valor é um dicionário contendo a posição (trans) e a orientação (rot).
            return MyServiceMessageResponse(True, f"Spot '{req.label}' saved")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            '''
            tf.LookupException: Lançada se a transformação solicitada não puder ser encontrada.
            tf.ConnectivityException: Lançada se houver um problema de conectividade que impede a obtenção da transformação.
            tf.ExtrapolationException: Lançada se a transformação solicitada não puder ser extrapolada (por exemplo, se for solicitada uma transformação para um tempo fora do intervalo de dados disponíveis).
            '''
            return MyServiceMessageResponse(False, "Failed to get robot position")

def spot_recorder_server():
    rospy.init_node('spot_recorder')
    s = rospy.Service('/save_spot', MyServiceMessage, handle_save_spot)
    rospy.spin()

if __name__ == "__main__":
    spot_recorder_server()

#!/usr/bin/env python3
import rospy
import tf
import subprocess
from geometry_msgs.msg import PoseWithCovarianceStamped

class wait_for_tf():
    def __init__(self):
        # Essa TF Representa a correção global da posição do robô baseada na correspondência do LIDAR com o mapa estático.
        rospy.loginfo("Aguardando transformação 'odom' -> 'map' do AMCL...")
    
        self.listener = tf.TransformListener()
    
        self.cont = 0

        #publisher
        self.position_publisher = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True) 

    def iniciar_em_zero(self):
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"  # Frame de referência
        pose.pose.pose.position.x = 0.0
        pose.pose.pose.position.y = 0.0
        pose.pose.pose.position.z = 0.0
        pose.pose.pose.orientation.w = 1.0  # Sem rotação (orientação nula)

        # Pequena incerteza na posição e orientação
        pose.pose.covariance = [
            0.25, 0,    0, 0, 0, 0,
            0,    0.25, 0, 0, 0, 0,
            0,    0,    0, 0, 0, 0,
            0,    0,    0, 0, 0, 0,
            0,    0,    0, 0, 0, 0,
            0,    0,    0, 0, 0, 0.068538919
        ]

        self.position_publisher.publish(pose)
        rospy.loginfo("Pose inicial enviada: x=0, y=0, orientação=w=1")

    def run(self):
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            if self.listener.canTransform("map", "odom", rospy.Time(0)):
                print(f"Tempo de carregamento do AMCL: {self.cont} segundos.")
                rospy.loginfo("Transformação detectada!")
                rospy.loginfo("Iniciando move_base_ponto_ponto...")
                self.iniciar_em_zero()
                subprocess.call(["roslaunch", "nav_hub", "move_base_ponto_ponto.launch"])
                break
            else:
                rospy.logwarn("Transformação 'odom' -> 'map' do AMCL ainda não disponível. Tentando novamente...")
                rate.sleep()
                self.cont += 1

if __name__ == "__main__":
    rospy.init_node("wait_for_tf_ponto_ponto_node", anonymous=True)
    objt = wait_for_tf()
    objt.run()
    





#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped

class TF_Listener:
    def __init__(self):       
        self.listener = tf.TransformListener()
        self.pub_pose = rospy.Publisher("keyence_pose", PoseWithCovarianceStamped, queue_size=1)

    def is_valid(self, values):
        trans, rot = values
        # Verifica se algum valor é NaN
        return not ( any(math.isnan(x) for x in trans) or any(math.isnan(x) for x in rot) )

    def run(self):
        try:
            # if self.listener.canTransform("map", "base_link", rospy.Time(0)): #retorna true  se existir um tf de base_link para map
                # Retorna a posição relativa em relação ao /map.
                values = self.listener.lookupTransform("map", "base_link", rospy.Time(0))
                # print(f"values: {values}")
                # if not self.is_valid(values):
                #     rospy.logwarn("Transformação contém NaN, ignorando...")
                #     return

                position = PoseWithCovarianceStamped()
                position.header.stamp = rospy.Time.now()
                position.header.frame_id = "map"
                
                position.pose.pose.position.x = values[0][0]
                position.pose.pose.position.y = values[0][1]
                position.pose.pose.position.z = values[0][2]
                position.pose.pose.orientation.x = values[1][0]
                position.pose.pose.orientation.y = values[1][1]
                position.pose.pose.orientation.z = values[1][2]
                position.pose.pose.orientation.w = values[1][3]
                

                self.pub_pose.publish(position)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as erro:
            # print(f"A classe do erro encontrado foi {erro.__class__}!")
            # print(f"O erro encontrado foi {erro.__cause__}!")
            pass

if __name__ == '__main__':
    rospy.init_node("tf_listener_node", anonymous=True)
    rospy.loginfo("TF Listener initialized!")
    tf_listener = TF_Listener()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        tf_listener.run()
        rate.sleep()



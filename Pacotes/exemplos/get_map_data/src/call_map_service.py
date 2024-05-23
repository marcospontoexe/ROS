#!/usr/bin/env python

import rospy
from nav_msgs.srv import GetMap

def call_map_service():
    rospy.wait_for_service('/static_map') # espera até que o serviço /static_map esteja disponível no ROS
    try:
        get_map = rospy.ServiceProxy('/static_map', GetMap)
        response = get_map()
        width = response.map.info.width
        height = response.map.info.height
        resolution = response.map.info.resolution
        rospy.loginfo(f"Map dimensions: {width}x{height}, Resolution: {resolution}")    # gera um Response para o serviço
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    rospy.init_node('map_service_client_node')
    call_map_service()

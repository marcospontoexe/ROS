#!/usr/bin/env python

import rospy
from services_quiz.srv import BB8CustomServiceMessage, BB8CustomServiceMessageRequest

def bb8_custom_service_client(side, repetitions):
    rospy.wait_for_service('/move_bb8_in_square_custom')
    try:
        bb8_service = rospy.ServiceProxy('/move_bb8_in_square_custom', BB8CustomServiceMessage)
        req = BB8CustomServiceMessageRequest(side, repetitions)
        resp = bb8_service(req)
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('service_move_bb8_in_square_client_node')
    rospy.loginfo("Client has started.")
    
    # Moving in small square (1 meter side) twice
    success = bb8_custom_service_client(1.0, 2)
    if success:
        rospy.loginfo("BB-8 completed the small square movements successfully.")
    else:
        rospy.loginfo("BB-8 failed to complete the small square movements.")

    # Moving in big square (2 meters side) once
    success = bb8_custom_service_client(2.0, 1)
    if success:
        rospy.loginfo("BB-8 completed the big square movement successfully.")
    else:
        rospy.loginfo("BB-8 failed to complete the big square movement.")

'''
#! /usr/bin/env python
import rospkg
import rospy
from services_quiz.srv import BB8CustomServiceMessage, BB8CustomServiceMessageRequest


rospy.init_node('service_move_bb8_in_square_client_node') # Initialise a ROS node with the name service_move_bb8_in_square_client_node
rospy.wait_for_service('/move_bb8_in_square_custom') # Wait for the service client /move_bb8_in_square_custom to be running
move_bb8_in_square_service_client = rospy.ServiceProxy('/move_bb8_in_circle_custom', BB8CustomServiceMessage) # Create the connection to the service
move_bb8_in_square_request_object = BB8CustomServiceMessageRequest() # Create an object of type EmptyRequest


"""
# BB8CustomServiceMessage
float64 side         # The distance of each side of the square
int32 repetitions    # The number of times BB-8 has to execute the square movement when the service is called
---
bool success         # Did it achieve it?
"""

# Movendo duas vezes num quadrado pequeno
#move_bb8_in_square_request_object.side = 1
#move_bb8_in_square_request_object.repetitions = 2
rospy.loginfo("Doing Service Call...")
#move_bb8_in_square_service_client(move_bb8_in_square_request_object) # Send through the connection the path to the trajectory file to be executed

# Movendo uma veze num quadrado grande
move_bb8_in_square_request_object.side = 2
move_bb8_in_square_request_object.repetitions = 1
result = move_bb8_in_square_service_client(move_bb8_in_square_request_object) # Send through the connection the path to the trajectory file to be executed

rospy.loginfo(str(result)) # Print the result given by the service called
rospy.loginfo("END of Service call...")
'''
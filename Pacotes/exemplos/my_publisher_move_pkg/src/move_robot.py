#! /usr/bin/env python

import rospy    # Import the Python library for ROS
from geometry_msgs.msg import Twist    # Import the Twist message from the geometry_msgs.msg package         


rospy.init_node('move_robot_node')  # Initiate a Node named 'move_robot_node'

# Create a Publisher object, that will publish on the /cmd_vel topic
# messages of type Twist
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)    
                                           
rate = rospy.Rate(2)    # Set a publish rate of 2 Hz
move = Twist()
move.linear.x = 0.5 #Move the robot with a linear velocity in the x axis
move.angular.z = 0.5 #Move the with an angular velocity in the z axis                        

# Create a loop that will go until someone stops the program execution
while not rospy.is_shutdown():  
  pub.publish(move)    # Publish the message within the 'move' object
  rate.sleep()  # Make sure the publish rate maintains at 2 Hz                           

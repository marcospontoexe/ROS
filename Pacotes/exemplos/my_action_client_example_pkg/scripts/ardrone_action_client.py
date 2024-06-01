#! /usr/bin/env python
import rospy
import time
import actionlib
from ardrone_as.msg import ArdroneAction, ArdroneGoal, ArdroneResult, ArdroneFeedback
# Se o arquivo de mensagem de ação se chama Ardrone.action, 
# então o tipo de mensagem de ação que você deve especificar é ArdroneAction, 

nImage = 1

# definition of the feedback callback. This will be called when feedback
# is received from the action server
# it just prints a message indicating a new message has been received
def feedback_callback(feedback):
    global nImage
    print('[Feedback] image n.%d received'%nImage)
    nImage += 1

# initializes the action client node
rospy.init_node('drone_action_client')

# create the connection to the action server
# O primeiro parâmetro é o nome do servidor de ações ao qual você deseja se conectar.
# O segundo parâmetro é o tipo de mensagem de ação que ele utiliza. A convenção segue da seguinte forma:
client = actionlib.SimpleActionClient('/ardrone_action_server', ArdroneAction)
# waits until the action server is up and running
client.wait_for_server()

# creates a goal to send to the action server.
# Se o arquivo de mensagem de ação se chama Ardrone.action,
# o tipo de mensagem goal que você deve especificar é ArdroneGoal().
goal = ArdroneGoal()
goal.nseconds = 10 # indicates, take pictures along 10 seconds

# sends the goal to the action server, specifying which feedback function
# to call when feedback received.
# Os parâmetros do objetivo
# Uma função de feedback a ser chamada de tempos em tempos para saber o status da ação.
client.send_goal(goal, feedback_cb=feedback_callback)

# Uncomment these lines to test goal preemption:
#time.sleep(3.0)
#client.cancel_goal()  # would cancel the goal 3 seconds after starting

# wait until the result is obtained
# you can do other stuff here instead of waiting
# and check for status from time to time 
# status = client.get_state()
# check the client API link below for more info

client.wait_for_result()

print('[Result] State: %d'%(client.get_state()))

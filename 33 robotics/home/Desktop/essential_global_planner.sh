#!/bin/bash

# Carrega o ambiente ROS
source /opt/ros/noetic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash


sleep 3  #  3 segundos
roslaunch nav_hub essential_global_planner.launch
# sleep 30
# roslaunch nav_hub botoeira.launch



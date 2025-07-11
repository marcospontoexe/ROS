#!/bin/bash
LOG_FILE="/home/ubuntu/Desktop/logs/Log_$(date +%d_%m_%Y).log"

# Adiciona o log quando o sistema liga
#echo "333333333333333333333333333333333333333333333" >> /home/ubuntu/logs/Log_$(date +%Y%m%d).log
#echo "3333      33333      33333333        33333333" >> /home/ubuntu/logs/Log_$(date +%Y%m%d).log
#echo "33333333   33333333   3333333  33333  3333333" >> /home/ubuntu/logs/Log_$(date +%Y%m%d).log
#echo "33333333   33333333   3333333  33333  3333333" >> /home/ubuntu/logs/Log_$(date +%Y%m%d).log
#echo "33333     333333     33333333        33333333" >> /home/ubuntu/logs/Log_$(date +%Y%m%d).log
#echo "33333333   333333333  3333333  333333  333333" >> /home/ubuntu/logs/Log_$(date +%Y%m%d).log
#echo "33333333   333333333  3333333  333333  333333" >> /home/ubuntu/logs/Log_$(date +%Y%m%d).log
#echo "3333      33333      33333333  333333  333333" >> /home/ubuntu/logs/Log_$(date +%Y%m%d).log
#echo "333333333333333333333333333333333333333333333" >> /home/ubuntu/logs/Log_$(date +%Y%m%d).log
echo "$(date +%d_%m_%Y %H:%M:%S");Info;Sistema ligado" >> $LOG_FILE
python /home/ubuntu/catkin_ws/src/nav_hub/scripts/flask/app.py 


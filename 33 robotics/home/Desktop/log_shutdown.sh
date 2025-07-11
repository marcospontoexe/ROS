#!/bin/bash
# Cria um log antes do desligamento
echo "$(date +"%d_%m_%Y %H:%M:%S");Info;Sistema desligado" >> /home/ubuntu/Desktop/logs/Log_$(date +%d_%m_%Y).log

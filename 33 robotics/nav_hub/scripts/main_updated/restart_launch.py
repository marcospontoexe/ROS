#!/usr/bin/env python3

import subprocess
import time
import os
# import RPi.GPIO as GPIO

# Se o serviço estiver rodando como root;
# - permissão de execução ao systemctl sem pedir senha:
#   - which systemctl
#   - sudo visudo
#   - Adicione as seguintes linha no final do arquivo sudoers: ubuntu ALL=(ALL) NOPASSWD: /usr/bin/systemctl


class LaunchManager:
    def __init__(self):
        self.start_time = 0.0
        self.elapsed_time = 0.0

    def killLaunch(self):
        """Encerra a launch botoeira.launch"""
        print("Encerrando launch...")
        os.system('rostopic pub -1 /cor std_msgs/String "Apagar"')  #publica uma unica vez
        time.sleep(1)   # para dar tempo de apagar o led antes de entrar no processo "subprocess.run"
        # subprocess.run(["sudo", "pkill", "-f", "botoeira.launch"])
        # time.sleep(5)
        os.system("sudo systemctl restart start_essentials_ponto_ponto.service")
        os.system("sudo systemctl restart start_essentials_global_planner.service")
        time.sleep(5)
        os.system("sudo systemctl restart start_global_planner.service")
        os.system("sudo systemctl restart start_ponto_ponto.service")
        # os.system(self.command)


if __name__ == "__main__":
    manager = LaunchManager()
    # manager.killLaunch()

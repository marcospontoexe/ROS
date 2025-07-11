#!/bin/bash

# Carrega o ambiente ROS
source /opt/ros/noetic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash

# Certifique-se de que as variáveis ROS estão corretas
#export ROS_MASTER_URI=http://localhost:11311
#export ROS_IP=10.42.0.1

# Espera alguns segundos para garantir que o ambiente esteja pronto
sleep 3

# Defina o MAC address do dispositivo Bluetooth
MAC_ADDRESS="00:90:E1:A8:95:05"

# Defina os comandos roslaunch 
LAUNCH_NAV_DEMO_ESSENTIAL="roslaunch nav_hub essential_ponto_ponto.launch"
LAUNCH_NAV_DEMO="roslaunch nav_hub ponto_ponto.launch"
#LAUNCH_NAV_DEMO_ESSENTIAL="roslaunch nav_hub essential_global_planner.launch"
#LAUNCH_NAV_DEMO="roslaunch nav_hub global_planner.launch"
LAUNCH_JOY="roslaunch nav_hub joy.launch"

# Defina os comandos para parar os lançamentos (usando `rosnode kill` para finalizar os nós específicos)
STOP_NAV_DEMO_ESSENTIAL="pkill -f essential_ponto_ponto.launch"  
STOP_NAV_DEMO="pkill -f ponto_ponto.launch"
#STOP_NAV_DEMO_ESSENTIAL="pkill -f essential_global_planner.launch"  
#STOP_NAV_DEMO="pkill -f global_planner.launch"  
STOP_JOY="pkill -f joy.launch"  

# Função para verificar a conexão Bluetooth
is_connected() {
    hcitool con | grep -q "$MAC_ADDRESS"
    return $?
}

# Função para iniciar o lançamento ponto_ponto e parar o joy
start_ponto_ponto() {
    # Verifica se o launch joy está rodando e, se sim, mata o processo
    pgrep -f "$LAUNCH_JOY" && $STOP_JOY
    sleep 10
    # Inicia o launch ponto_ponto
    $LAUNCH_NAV_DEMO_ESSENTIAL &
    sleep 20
    $LAUNCH_NAV_DEMO &
}

# Função para iniciar o joy
start_joy() {
    # Verifica se o launch ponto_ponto está rodando e, se sim, mata o processo
    pgrep -f "$LAUNCH_NAV_DEMO_ESSENTIAL" && $STOP_NAV_DEMO_ESSENTIAL
    pgrep -f "$LAUNCH_NAV_DEMO" && $STOP_NAV_DEMO
    sleep 10
    # Inicia o launch joy
    $LAUNCH_JOY &
}

# Variável para rastrear o estado anterior
flagPonto=false
flagJoy=false

while true; do
    if is_connected; then
        if [ "$flagJoy" = false ]; then
            echo "Dispositivo conectado. Iniciando joy.launch..."
            start_joy
            flagJoy=true
            flagPonto=false
        fi
    else
        if [ "$flagPonto" = false ]; then
            echo "Dispositivo desconectado. Encerrando joy.launch..."
            start_ponto_ponto
            flagPonto=true
            flagJoy=false
        fi
    fi
    sleep 1  # Verifica a cada 100 milisegundos
done


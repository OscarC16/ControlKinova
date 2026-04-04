#!/bin/bash
# Script para enviar una coordenada al robot y recibir la imagen

# Validacin de parmetros
if [ "$#" -ne 6 ]; then
    echo "Uso: ./enviar_coordenada.sh X Y Z ROLL PITCH YAW"
    echo "Ejemplo: ./enviar_coordenada.sh 0.4 0.1 0.4 180.0 0.0 90.0"
    exit 1
fi

X=$1
Y=$2
Z=$3
ROLL=$4
PITCH=$5
YAW=$6

# 1. Configuracin de entorno
export FASTRTPS_DEFAULT_PROFILES_FILE=/mnt/d/Ros/fastdds_config.xml
source /opt/ros/jazzy/setup.bash
source /home/oscar/kinova_ws/install/setup.bash

echo "[INFO] Enviando coordenadas: X=$X, Y=$Y, Z=$Z, ROLL=$ROLL, PITCH=$PITCH, YAW=$YAW"

# 2. Llamada al servicio
ros2 service call /move_and_capture kinova_apps/srv/MoveAndCapture "{x: $X, y: $Y, z: $Z, roll: $ROLL, pitch: $PITCH, yaw: $YAW}"

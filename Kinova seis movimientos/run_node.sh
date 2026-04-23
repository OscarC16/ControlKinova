#!/bin/bash
# Script para ejecutar el servidor de seis movimientos
export FASTRTPS_DEFAULT_PROFILES_FILE=/mnt/d/Ros/Kinova\ seis\ movimientos/fastdds_config.xml
source /opt/ros/jazzy/setup.bash
source /home/oscar/kinova_ws/install/setup.bash

echo "[INFO] Iniciando servidor de Seis Movimientos..."
python3 "/mnt/d/Ros/Kinova seis movimientos/six_moves_node.py"

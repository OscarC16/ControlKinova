#!/bin/bash
# Script para probar el servicio de seis movimientos
export FASTRTPS_DEFAULT_PROFILES_FILE=/mnt/d/Ros/Kinova\ seis\ movimientos/fastdds_config.xml
source /opt/ros/jazzy/setup.bash
source /home/oscar/kinova_ws/install/setup.bash

echo "[INFO] Enviando secuencia de prueba..."
python3 "/mnt/d/Ros/Kinova seis movimientos/test_six_moves.py"

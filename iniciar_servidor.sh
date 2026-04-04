#!/bin/bash
# Script para iniciar el servidor de ROS 2

# 1. Configuracin de entorno
export FASTRTPS_DEFAULT_PROFILES_FILE=/mnt/d/Ros/fastdds_config.xml
source /opt/ros/jazzy/setup.bash

WS_DIR="/home/oscar/kinova_ws"
PACKAGE_DIR="/mnt/d/Ros/kinova_apps"

echo "[INFO] Iniciando configuracin del servidor..."

# 2. Asegurar que el paquete est en el workspace
if [ ! -d "$WS_DIR/src/kinova_apps" ]; then
    echo "[INFO] Vinculando el paquete al workspace ROS 2..."
    ln -s "$PACKAGE_DIR" "$WS_DIR/src/"
fi

# 3. Compilar el paquete (necesario para el servicio personalizado)
cd "$WS_DIR"
echo "[INFO] Compilando el servicio. Por favor, espera..."
colcon build --packages-select kinova_apps

# 4. Cargar el entorno local
source "$WS_DIR/install/setup.bash"

# 5. Ejecutar el nodo del servidor
echo "[INFO] Servidor listo. Esperando llamadas..."
ros2 run kinova_apps robot_service_node.py

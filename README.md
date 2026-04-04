# Control de Robot Kinova Gen3 - ROS 2

Este repositorio contiene la implementación de un nodo de servicio para el control automatizado de un brazo robótico Kinova Gen3 utilizando ROS 2 Jazzy y MoveIt 2.

## Características
- **Servicio `move_and_capture`**: Un único servicio que mueve el robot a una posición cartesiana, captura una imagen y regresa al robot a su posición inicial (Home).
- **Captura Inteligente**: Intenta obtener imágenes vía RTSP directamente del robot con respaldo (fallback) a los tópicos de ROS.
- **Integración con MoveIt 2**: Planificación de trayectorias segura para evitar obstáculos.

## Estructura del Proyecto
- `kinova_apps/`: Paquete de ROS 2 que contiene el nodo servidor, definiciones de servicio (.srv) y archivos launch.
- `scripts/`: Clientes de prueba para llamar al servicio.
- `*.xacro`: Modelos URDF optimizados para la comunicación hardware.

## Guía de Ejecución (Paso a Paso)

Para que el sistema funcione correctamente, debes abrir **3 terminales** de WSL y ejecutar los comandos en el siguiente orden:

### 🚩 Terminal 1: Driver del Robot y MoveIt
Esta terminal establece la comunicación con el robot físico y prepara el sistema de planificación de movimiento.
```bash
# Cargar entorno
source /opt/ros/jazzy/setup.bash
source /home/oscar/kinova_ws/install/setup.bash

# Iniciar driver y MoveIt (ejemplo para Gen3 7DOF)
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py robot_ip:=192.168.1.10 use_fake_hardware:=false
```

### 🤖 Terminal 2: Servidor de Aplicación
Ejecuta el nodo `robot_service_node`, que gestiona la lógica central.
```bash
# Cargar entorno
source /opt/ros/jazzy/setup.bash
source /home/oscar/kinova_ws/install/setup.bash

# Iniciar nuestro servicio
ros2 launch kinova_apps robot_service.launch.py
```

### 📸 Terminal 3: Cliente
Usa esta terminal para enviar órdenes al robot.
```bash
# Ejecutar cliente de prueba
ros2 run kinova_apps test_service_client.py
```

## Requisitos
- ROS 2 Jazzy
- MoveIt 2
- OpenCV y CV Bridge
- Kinova Kortex Driver

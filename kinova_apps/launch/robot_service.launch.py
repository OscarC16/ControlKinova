import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Parámetros configurables
    robot_ip_launch_arg = {'robot_ip': '192.168.1.10'}

    return LaunchDescription([
        Node(
            package='kinova_apps',
            executable='robot_service_node.py',
            name='robot_service_node',
            output='screen',
            parameters=[robot_ip_launch_arg],
            emulate_tty=True
        )
    ])

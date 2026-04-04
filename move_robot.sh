#!/bin/bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/mnt/d/Ros/fastdds_config.xml
source /opt/ros/jazzy/setup.bash
source /home/oscar/kinova_ws/install/setup.bash
python3 /mnt/d/Ros/kinova_move.py "$@"

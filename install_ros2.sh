#!/bin/bash
set -e

echo "Configuring locales..."
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "Enabling required repositories..."
apt update && apt install locales -y
apt install software-properties-common -y
add-apt-repository universe -y

echo "Adding ROS 2 GPG key..."
apt update && apt install curl -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "Adding ROS 2 repository..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "Updating system..."
apt update
apt upgrade -y

echo "Installing ROS 2 Jazzy Desktop..."
apt install ros-jazzy-desktop -y

echo "Installation complete!"

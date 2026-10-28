#!/usr/bin/env bash
# Install ROS2 Jazzy's rviz2 (and its deps: tf2, Fast DDS, message types) in
# WSL Ubuntu 24.04. Jazzy is the ROS2 distro for noble; it talks to the laptop's
# Lyrical over Fast DDS for standard messages (JointState, TF, robot_description).
set -euo pipefail

echo ">>> apt update + prerequisites"
sudo apt update
sudo apt install -y software-properties-common curl

echo ">>> enable universe + add the ROS2 apt repository"
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
CODENAME=$(. /etc/os-release && echo "$UBUNTU_CODENAME")
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${CODENAME} main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo ">>> apt update + install rviz2 (this is the big download)"
sudo apt update
sudo apt install -y ros-jazzy-rviz2

if [ -x /opt/ros/jazzy/bin/rviz2 ]; then
  echo ">>> SUCCESS: rviz2 installed at /opt/ros/jazzy/bin/rviz2"
else
  echo ">>> WARNING: rviz2 not found where expected" >&2
  exit 1
fi

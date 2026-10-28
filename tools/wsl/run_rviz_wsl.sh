#!/usr/bin/env bash
# Launch RViz2 in WSL, subscribing to the laptop's topics over the LAN.
# Works around the broken prefix setup.sh by eval-ing the env util directly.

# Build the ROS2 Jazzy environment (PATH, LD_LIBRARY_PATH, AMENT_PREFIX_PATH, ...)
eval "$(AMENT_CURRENT_PREFIX=/opt/ros/jazzy python3 /opt/ros/jazzy/_local_setup_util.py sh)"

# Add the local mesh prefix so package://so_arm_100_description resolves
export AMENT_PREFIX_PATH="/home/danb/so100_viz:${AMENT_PREFIX_PATH}"

# Match the laptop's ROS graph
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISTRO=jazzy
export ROS_VERSION=2

# WSLg display
export DISPLAY="${DISPLAY:-:0}"

echo "AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH"
echo "rviz2 path: $(command -v rviz2)"
exec rviz2 -d /mnt/f/git/ros-so100/ros2_ws/src/so_arm_100_description/rviz/so_arm_100.rviz

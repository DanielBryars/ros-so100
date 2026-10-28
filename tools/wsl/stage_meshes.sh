#!/usr/bin/env bash
# Stage SO100 meshes into a WSL-local ament prefix so RViz can resolve
# package://so_arm_100_description/... when the URDF arrives over the network.
set -euo pipefail

PREFIX="$HOME/so100_viz"
SRC=/mnt/f/git/ros-so100/ros2_ws/src/so_arm_100_description
DEST="$PREFIX/share/so_arm_100_description/models/so_arm_100_5dof/meshes"

mkdir -p "$PREFIX/share/ament_index/resource_index/packages"
touch "$PREFIX/share/ament_index/resource_index/packages/so_arm_100_description"
mkdir -p "$DEST"
cp "$SRC"/models/so_arm_100_5dof/meshes/*.STL "$DEST/"

echo "Prefix: $PREFIX"
echo "Meshes staged:"
ls -1 "$DEST"

#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/install/setup.bash"
echo "Starting Dual Delta Robot ROS 2 simulation in RViz2..."
ros2 launch delta_robot_pkg dual_robot_rviz.launch.py

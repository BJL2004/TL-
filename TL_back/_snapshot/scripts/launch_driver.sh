#!/usr/bin/env bash
set -euo pipefail

DRIVER_DIR="${TL_DRIVER_DIR:-$HOME/Desktop/tl_robot_ros2_py-arm}"
source /opt/ros/humble/setup.bash
source "${DRIVER_DIR}/install/setup.bash"
ros2 launch tl_bringup tl_tcb605_bringup.launch.py

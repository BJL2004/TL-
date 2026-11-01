#!/usr/bin/env bash
# One-command robot bringup: sources the ROS2 and driver workspaces, then
# launches the TL driver (with optional ROH hand guard node).
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source /opt/ros/humble/setup.bash
driver_dir="${TL_DRIVER_DIR:-$HOME/Desktop/tl_robot_ros2_py-arm}"
source "$driver_dir/install/setup.bash"
export AMENT_PREFIX_PATH="$project_dir/install/tl_hand_deploy:${AMENT_PREFIX_PATH:-}"

echo "Bringing up the TL robot: driver (+ hand guard if with_hand:=true)"
exec ros2 launch tl_hand_deploy tl_robot_bringup.launch.py "$@"

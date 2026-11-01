#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
dataset="${1:-$HOME/tl_handeye_data/session_20260808_022900}"

source /opt/ros/humble/setup.bash
source "${TL_DRIVER_DIR:-$HOME/Desktop/tl_robot_ros2_py-arm}/install/setup.bash"
export PYTHONPATH="$project_dir/src/tl_hand_deploy:${PYTHONPATH:-}"

python3 -m tl_hand_deploy.handeye_accuracy "$dataset"

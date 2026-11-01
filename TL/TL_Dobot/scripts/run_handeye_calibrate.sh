#!/usr/bin/env bash
# 手眼标定求解（直跑源码，绕开 install/launch 环境解析问题）
# 用法：bash run_handeye_calibrate.sh ~/tl_handeye_data/session_xxx
# 退出码 0 = 质量门全部通过（生成 calibration/handeye_accepted.yaml）
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
export PYTHONPATH="$project_dir/src/tl_hand_deploy:${PYTHONPATH:-}"

source /opt/ros/humble/setup.bash
driver_dir="${TL_DRIVER_DIR:-$HOME/Desktop/tl_robot_ros2_py-arm}"
if [[ -f "$driver_dir/install/setup.bash" ]]; then
    source "$driver_dir/install/setup.bash"
fi

DATASET="${1:?用法: bash run_handeye_calibrate.sh <session目录>}"
exec python3 -m tl_hand_deploy.handeye_calibrate "$DATASET"

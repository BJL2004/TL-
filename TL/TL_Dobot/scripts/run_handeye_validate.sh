#!/usr/bin/env bash
# 手眼标定九点绝对精度验证（直跑源码，绕开 install/launch 环境解析问题）
# 用法：bash run_handeye_validate.sh <九点记录.jsonl> <标定文件.yaml>
# 退出码 0 且 .acceptance.json 的 quality_checks 全 true = 通过
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
export PYTHONPATH="$project_dir/src/tl_hand_deploy:${PYTHONPATH:-}"

source /opt/ros/humble/setup.bash
driver_dir="${TL_DRIVER_DIR:-$HOME/Desktop/tl_robot_ros2_py-arm}"
if [[ -f "$driver_dir/install/setup.bash" ]]; then
    source "$driver_dir/install/setup.bash"
fi

LOG="${1:?用法: bash run_handeye_validate.sh <log.jsonl> <calibration.yaml>}"
CALIB="${2:?用法: bash run_handeye_validate.sh <log.jsonl> <calibration.yaml>}"
exec python3 -m tl_hand_deploy.handeye_validate_report "$LOG" --calibration "$CALIB"

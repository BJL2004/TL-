#!/usr/bin/env bash
set -eo pipefail

# MoveJ 鼠标点击测试 — 使用 factory MoveJ 而非 ServoJ
# 用法: ./scripts/run_movej_click.sh

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
vendor_dir="${TL_DRIVER_DIR:-$HOME/Desktop/tl_robot_ros2_py-arm}"
calibration_file="${CALIBRATION_FILE:-$HOME/tl_handeye_data/session_20260808_022900/calibration/handeye_candidate_rejected.yaml}"

source /opt/ros/humble/setup.bash
source "$vendor_dir/install/setup.bash"
export PYTHONPATH="$project_dir/src/tl_hand_deploy:$project_dir/build/tl_hand_deploy:${PYTHONPATH:-}"

echo "MoveJ mode: click → calculate → MoveJ to hover above target."
echo "No ServoJ, no safety proxy. Keep E-stop ready."

python3 -m tl_hand_deploy.movej_click \
    --calibration "$calibration_file" \
    --allow-rejected \
    --hover-mm "${TL_HOVER_MM:-150}" \
    --velocity "${TL_VELOCITY:-20}" \
    --acceleration "${TL_ACCELERATION:-20}" \
    --deceleration "${TL_DECELERATION:-20}" \
    --execute

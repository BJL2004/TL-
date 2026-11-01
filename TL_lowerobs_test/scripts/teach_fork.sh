#!/usr/bin/env bash
# ==================================================
# 叉子示教 · 下排观测位验证版（独立运行，不读旧 TL 目录）
# 新功能：
#   v = A/B 侧来回切换（起始位 -83.668… / 96.332…，即 J1+180）
#   d = 把当前手臂位姿记录为“当前侧”的下排观测位（写 YAML，重复按覆盖）
#   g = 回到当前侧已记录的下排观测位（验证用）
# ==================================================
set -eo pipefail
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source /opt/ros/humble/setup.bash
driver_dir="${TL_DRIVER_DIR:-$HOME/Desktop/tl_robot_ros2_py-arm}"
if [[ -f "$driver_dir/install/setup.bash" ]]; then
    source "$driver_dir/install/setup.bash"
else
    echo "ERROR: TL driver workspace is missing: $driver_dir/install/setup.bash" >&2
    exit 2
fi
export PYTHONPATH="$root_dir/src/tl_fork_teach:${PYTHONPATH:-}"

calibration="${TL_CALIBRATION:-$HOME/tl_handeye_data/best/handeye_best.yaml}"
output="${TL_FORK_TEACH_OUTPUT:-$HOME/tl_fork_teach/fork_basket_teach.yaml}"
serial_args=()
if [[ -n "${TL_CAMERA_SERIAL:-}" ]]; then
    serial_args+=(--camera-serial "$TL_CAMERA_SERIAL")
fi

echo "Standalone U-fork basket teaching [LowerObs verification bundle]."
echo "This program does not use the hand, gripper, MQTT, YOLO, or safety proxy."
echo "Only v / g / R keys send robot motion. Keep the physical E-stop ready."
echo "New keys: v switch side | d record lower-observation pose | g goto it."
echo "Calibration: $calibration"
echo "Output: $output"

python3 -m tl_fork_teach.fork_basket_teach \
    --calibration "$calibration" \
    --output "$output" \
    --marker-length-m "${TL_MARKER_LENGTH_M:-0.054}" \
    --reversed-rows ${TL_REVERSED_ROWS:-A3} \
    --servoj-speed-deg-s "${TL_SERVOJ_SPEED_DEG_S:-8}" \
    --servoj-rate-hz "${TL_SERVOJ_RATE_HZ:-100}" \
    --maximum-cross-axis-mm "${TL_FORK_MAX_CROSS_MM:-60}" \
    --allow-rejected \
    --execute \
    "${serial_args[@]}" \
    "$@"

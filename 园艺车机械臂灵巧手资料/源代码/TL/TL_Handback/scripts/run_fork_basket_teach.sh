#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source /opt/ros/humble/setup.bash
driver_dir="${TL_DRIVER_DIR:-$HOME/Desktop/tl_robot_ros2_py-arm}"
if [[ -f "$driver_dir/install/setup.bash" ]]; then
    source "$driver_dir/install/setup.bash"
else
    echo "ERROR: TL driver workspace is missing: $driver_dir/install/setup.bash" >&2
    exit 2
fi
if [[ -f "$project_dir/install/setup.bash" ]]; then
    source "$project_dir/install/setup.bash"
fi
export PYTHONPATH="$project_dir/src/tl_fork_teach:${PYTHONPATH:-}"

calibration="${TL_CALIBRATION:-$HOME/tl_handeye_data/best/handeye_best.yaml}"
output="${TL_FORK_TEACH_OUTPUT:-$HOME/tl_fork_teach/fork_basket_teach.yaml}"
serial_args=()
if [[ -n "${TL_CAMERA_SERIAL:-}" ]]; then
    serial_args+=(--camera-serial "$TL_CAMERA_SERIAL")
fi

echo "Standalone U-fork basket teaching."
echo "This program does not use the hand, gripper, MQTT, YOLO, or safety proxy."
echo "Only R sends a direct ServoJ return command. Keep the physical E-stop ready."
echo "Calibration: $calibration"
echo "Output: $output"

python3 -m tl_fork_teach.fork_basket_teach \
    --calibration "$calibration" \
    --output "$output" \
    --marker-length-m "${TL_MARKER_LENGTH_M:-0.054}" \
    --servoj-speed-deg-s "${TL_SERVOJ_SPEED_DEG_S:-8}" \
    --servoj-rate-hz "${TL_SERVOJ_RATE_HZ:-100}" \
    --maximum-cross-axis-mm "${TL_FORK_MAX_CROSS_MM:-60}" \
    --allow-rejected \
    --execute \
    "${serial_args[@]}"

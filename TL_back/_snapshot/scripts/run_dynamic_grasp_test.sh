#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
export PYTHONPATH="$project_dir/src/tl_hand_deploy:${PYTHONPATH:-}"

# Load ROS2 and the TL driver message workspace; override with TL_DRIVER_DIR.
source /opt/ros/humble/setup.bash
driver_dir="${TL_DRIVER_DIR:-$HOME/Desktop/tl_robot_ros2_py-arm}"
if [[ -f "$driver_dir/install/setup.bash" ]]; then
    source "$driver_dir/install/setup.bash"
fi

export TL_BASE_XY_BIAS_MM="${TL_BASE_XY_BIAS_MM:--53.2 -34.2}"
export TL_GRASP_RPY_RAD="${TL_GRASP_RPY_RAD:--2.043 1.471 -1.025}"
export TL_GRASP_SURFACE_OFFSET_MM="${TL_GRASP_SURFACE_OFFSET_MM:-15}"
export TL_OBSERVE_JOINT_DEG="${TL_OBSERVE_JOINT_DEG:--1.521 -1.225 35.678 3.856 31.471 -3.533}"
export TL_USE_HAND="${TL_USE_HAND:-0}"
export TL_HAND_GRASP_POSITIONS="${TL_HAND_GRASP_POSITIONS:-35000 35000 35000 35000 35000 25000}"
export TL_FINE_STEP_MM="${TL_FINE_STEP_MM:-1}"
export TL_FINE_Z_STEP_MM="${TL_FINE_Z_STEP_MM:-5}"

echo "Dynamic-height grasp test:"
echo "  TL_GRASP_SURFACE_OFFSET_MM=${TL_GRASP_SURFACE_OFFSET_MM} (tool0 Z above the camera-measured surface)"
echo "  TL_USE_HAND=${TL_USE_HAND}"
echo "  TL_OBSERVE_JOINT_DEG=${TL_OBSERVE_JOINT_DEG}"

extra_args=()
if [[ "$TL_USE_HAND" == "1" ]]; then
    extra_args+=(--use-hand --hand-grasp-positions $TL_HAND_GRASP_POSITIONS)
fi

python3 -m tl_hand_deploy.dynamic_grasp_test \
    --calibration "${TL_CALIBRATION:-$HOME/tl_handeye_data/best/handeye_best.yaml}" \
    --base-xy-bias-mm $TL_BASE_XY_BIAS_MM \
    --grasp-rpy-rad $TL_GRASP_RPY_RAD \
    --grasp-surface-offset-mm "$TL_GRASP_SURFACE_OFFSET_MM" \
    --observe-joint-deg $TL_OBSERVE_JOINT_DEG \
    --fine-step-mm "$TL_FINE_STEP_MM" \
    --fine-z-step-mm "$TL_FINE_Z_STEP_MM" \
    "${extra_args[@]}" \
    --allow-rejected

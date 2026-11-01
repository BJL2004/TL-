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

extra_args=()
export TL_TEACH_START_JOINT_DEG="${TL_TEACH_START_JOINT_DEG:--83.668 67.725 3.956 -2.101 -5.484 -1.431}"
if [[ -n "${TL_TEACH_START_JOINT_DEG:-}" ]]; then
    read -r t_j1 t_j2 t_j3 t_j4 t_j5 t_j6 <<<"$TL_TEACH_START_JOINT_DEG"
    if [[ -z "${t_j1:-}" || -z "${t_j2:-}" || -z "${t_j3:-}" || -z "${t_j4:-}" || -z "${t_j5:-}" || -z "${t_j6:-}" ]]; then
        echo "TL_TEACH_START_JOINT_DEG must contain six numbers: J1..J6 in degrees" >&2
        exit 2
    fi
    extra_args+=(--teach-start-joint-deg "$t_j1" "$t_j2" "$t_j3" "$t_j4" "$t_j5" "$t_j6")
    echo "Teach start pose[deg]=$TL_TEACH_START_JOINT_DEG"
fi
# B 侧示教起始位（A 侧 J1+180° 的镜像；可在示教窗口按 v 一键转过去）
export TL_TEACH_START_JOINT_DEG_B="${TL_TEACH_START_JOINT_DEG_B:-96.332 67.725 3.956 -2.101 -5.484 -1.431}"
if [[ -n "${TL_TEACH_START_JOINT_DEG_B:-}" ]]; then
    read -r t_b1 t_b2 t_b3 t_b4 t_b5 t_b6 <<<"$TL_TEACH_START_JOINT_DEG_B"
    if [[ -z "${t_b1:-}" || -z "${t_b2:-}" || -z "${t_b3:-}" || -z "${t_b4:-}" || -z "${t_b5:-}" || -z "${t_b6:-}" ]]; then
        echo "TL_TEACH_START_JOINT_DEG_B must contain six numbers: J1..J6 in degrees" >&2
        exit 2
    fi
    extra_args+=(--teach-start-joint-deg-b "$t_b1" "$t_b2" "$t_b3" "$t_b4" "$t_b5" "$t_b6")
    echo "B-side teach start pose[deg]=$TL_TEACH_START_JOINT_DEG_B (press v to move there)"
fi

echo "ArUco 192-basket teach (8 rows x 24 baskets). Only R sends robot motion to return."
echo "Number 1..192 maps to ArUco ID 0..191; black square must measure 54 mm."
echo "Rows: A1-01..24 / A2 / A3 / A4 / B1 / B2 / B3 / B4 (DICT_4X4_250 codes)."
echo "Type the basket number (1..192), center one marker, B select, A approach, P release, R return."

python3 -m tl_hand_deploy.aruco_basket_teach \
    --calibration "${TL_CALIBRATION:-$HOME/tl_handeye_data/best/handeye_best.yaml}" \
    --output "${TL_BASKET_ARUCO_FILE:-$HOME/tl_mock_pick_place/basket_aruco.yaml}" \
    --marker-length-m "${TL_MARKER_LENGTH_M:-0.054}" \
    "${extra_args[@]}" \
    --interp-points "${TL_SERVOJ_POINTS:-0}" \
    --rate-hz "${TL_SERVOJ_RATE_HZ:-100}" \
    --servoj-speed-deg-s "${TL_SERVOJ_SPEED_DEG_S:-15}" \
    --servoj-precision-speed-deg-s "${TL_SERVOJ_PRECISION_SPEED_DEG_S:-8}" \
    --allow-rejected

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

echo "ArUco independent-basket teach. Only R sends robot motion to return."
echo "Number 1..6 maps to ArUco ID 0..5; black square must measure 58 mm."
echo "Center one marker: B select, A approach, P release, R return, then translate to the next basket."

python3 -m tl_hand_deploy.aruco_basket_teach \
    --calibration "${TL_CALIBRATION:-$HOME/tl_handeye_data/best/handeye_best.yaml}" \
    --output "${TL_BASKET_ARUCO_FILE:-$HOME/tl_mock_pick_place/basket_aruco.yaml}" \
    --marker-length-m "${TL_MARKER_LENGTH_M:-0.058}" \
    --marker-ids 0 1 2 3 4 5 \
    "${extra_args[@]}" \
    --interp-points "${TL_SERVOJ_POINTS:-0}" \
    --rate-hz "${TL_SERVOJ_RATE_HZ:-100}" \
    --servoj-speed-deg-s "${TL_SERVOJ_SPEED_DEG_S:-15}" \
    --servoj-precision-speed-deg-s "${TL_SERVOJ_PRECISION_SPEED_DEG_S:-8}" \
    --allow-rejected

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
export TL_MOCK_GRASP_OFFSET_MM="${TL_MOCK_GRASP_OFFSET_MM:--6.3 34.0 42.5}"
export TL_PICK_APPROACH_CLEARANCE_MM="${TL_PICK_APPROACH_CLEARANCE_MM:-130}"
export TL_PICK_GRASP_CLEARANCE_MM="${TL_PICK_GRASP_CLEARANCE_MM:-50}"
export TL_FINAL_GRASP_Z_MM="${TL_FINAL_GRASP_Z_MM:-69.62}"
export TL_MIN_MOCK_CLEARANCE_MM="${TL_MIN_MOCK_CLEARANCE_MM:-}"
export TL_SERVOJ_POINTS="${TL_SERVOJ_POINTS:-0}"
export TL_SERVOJ_RATE_HZ="${TL_SERVOJ_RATE_HZ:-100}"
export TL_SERVOJ_SPEED_DEG_S="${TL_SERVOJ_SPEED_DEG_S:-20}"
export TL_SERVOJ_PRECISION_SPEED_DEG_S="${TL_SERVOJ_PRECISION_SPEED_DEG_S:-8}"
export TL_SERVOJ_MIN_DURATION_SEC="${TL_SERVOJ_MIN_DURATION_SEC:-1.0}"
export TL_MOTION_BACKEND="${TL_MOTION_BACKEND:-servoj}"
export TL_MOVEJ_VELOCITY="${TL_MOVEJ_VELOCITY:-5}"
export TL_MOVEJ_ACCELERATION="${TL_MOVEJ_ACCELERATION:-5}"
export TL_MOVEJ_DECELERATION="${TL_MOVEJ_DECELERATION:-5}"
export TL_MOVEJ_MAX_RETRIES="${TL_MOVEJ_MAX_RETRIES:-2}"
export TL_MOVEJ_RETRY_MAX_ERROR_DEG="${TL_MOVEJ_RETRY_MAX_ERROR_DEG:-10}"
export TL_MOVEJ_STATUS_GRACE_SEC="${TL_MOVEJ_STATUS_GRACE_SEC:-2}"
export TL_MOVEJ_STALL_SEC="${TL_MOVEJ_STALL_SEC:-2}"
export TL_MOVEJ_PROGRESS_LOG_SEC="${TL_MOVEJ_PROGRESS_LOG_SEC:-5}"
export TL_MOTION_TIMEOUT="${TL_MOTION_TIMEOUT:-90}"
export TL_MAX_JOINT_DELTA_DEG="${TL_MAX_JOINT_DELTA_DEG:-220}"
export TL_AUTO_LIFT_TO_REACHABLE="${TL_AUTO_LIFT_TO_REACHABLE:-1}"
export TL_MAX_AUTO_LIFT_MM="${TL_MAX_AUTO_LIFT_MM:-200}"
export TL_PLACE_FILE="${TL_PLACE_FILE:-$HOME/tl_mock_pick_place/place_pose.yaml}"
export TL_BASKET_ARUCO_FILE="${TL_BASKET_ARUCO_FILE:-}"
export TL_BASKET_NUMBER="${TL_BASKET_NUMBER:-}"
export TL_BASKET_MOTION_MODE="${TL_BASKET_MOTION_MODE:-translation}"
export TL_MAX_BASKET_SHIFT_MM="${TL_MAX_BASKET_SHIFT_MM:-200}"
export TL_USE_HAND="${TL_USE_HAND:-0}"
export TL_HAND_GRASP_POSITIONS="${TL_HAND_GRASP_POSITIONS:-35000 35000 35000 35000 35000 25000}"
export TL_HAND_OPEN_POSITIONS="${TL_HAND_OPEN_POSITIONS:-0 0 0 0 0 0}"
export TL_TARGET_ONLY="${TL_TARGET_ONLY:-0}"
export TL_PLACE_ONLY="${TL_PLACE_ONLY:-0}"
export TL_USE_FIXED_OBSERVATION="${TL_USE_FIXED_OBSERVATION:-1}"
export TL_OBSERVATION_JOINTS="${TL_OBSERVATION_JOINTS:--83.668 67.725 3.956 -2.101 -5.484 -1.431}"
# Home pose: start here, return here after pick and after the place cycle.
export TL_HOME_JOINT_DEG="${TL_HOME_JOINT_DEG:--1.521 -1.225 35.678 3.856 31.471 -3.533}"

extra_args=()
if [[ "$TL_AUTO_LIFT_TO_REACHABLE" == "1" ]]; then
    extra_args+=(--auto-lift-to-reachable --max-auto-lift-mm "$TL_MAX_AUTO_LIFT_MM")
fi
if [[ "$TL_USE_HAND" == "1" ]]; then
    extra_args+=(--use-hand --hand-grasp-positions $TL_HAND_GRASP_POSITIONS --hand-open-positions $TL_HAND_OPEN_POSITIONS)
fi
if [[ "$TL_TARGET_ONLY" == "1" ]]; then
    extra_args+=(--target-only)
fi
if [[ "$TL_PLACE_ONLY" == "1" ]]; then
    extra_args+=(--place-only)
fi
if [[ -n "$TL_BASKET_ARUCO_FILE" ]]; then
    extra_args+=(--basket-aruco-file "$TL_BASKET_ARUCO_FILE")
fi
if [[ -n "$TL_BASKET_NUMBER" ]]; then
    extra_args+=(--basket-number "$TL_BASKET_NUMBER")
fi
if [[ "$TL_USE_FIXED_OBSERVATION" == "1" ]]; then
    extra_args+=(--observation-joints $TL_OBSERVATION_JOINTS)
fi
if [[ -n "$TL_FINAL_GRASP_Z_MM" ]]; then
    extra_args+=(--final-grasp-z-mm "$TL_FINAL_GRASP_Z_MM")
fi
if [[ -n "$TL_MIN_MOCK_CLEARANCE_MM" ]]; then
    extra_args+=(--min-mock-clearance-mm "$TL_MIN_MOCK_CLEARANCE_MM")
fi
if [[ -n "$TL_HOME_JOINT_DEG" ]]; then
    read -r home_j1 home_j2 home_j3 home_j4 home_j5 home_j6 <<<"$TL_HOME_JOINT_DEG"
    if [[ -z "${home_j1:-}" || -z "${home_j2:-}" || -z "${home_j3:-}" || -z "${home_j4:-}" || -z "${home_j5:-}" || -z "${home_j6:-}" ]]; then
        echo "TL_HOME_JOINT_DEG must contain six numbers: J1..J6 in degrees" >&2
        exit 2
    fi
    extra_args+=(--home-joint-deg "$home_j1" "$home_j2" "$home_j3" "$home_j4" "$home_j5" "$home_j6")
fi

echo "Mock pick-place to basket:"
echo "  TL_BASE_XY_BIAS_MM=${TL_BASE_XY_BIAS_MM}"
echo "  TL_GRASP_RPY_RAD=${TL_GRASP_RPY_RAD}"
echo "  TL_MOCK_GRASP_OFFSET_MM=${TL_MOCK_GRASP_OFFSET_MM}"
echo "  TL_PICK_APPROACH_CLEARANCE_MM=${TL_PICK_APPROACH_CLEARANCE_MM}"
echo "  TL_PICK_GRASP_CLEARANCE_MM=${TL_PICK_GRASP_CLEARANCE_MM}"
echo "  TL_FINAL_GRASP_Z_MM=${TL_FINAL_GRASP_Z_MM:-unset}"
echo "  TL_MIN_MOCK_CLEARANCE_MM=${TL_MIN_MOCK_CLEARANCE_MM:-unset}"
echo "  TL_PLACE_FILE=${TL_PLACE_FILE}"
echo "  TL_BASKET_ARUCO_FILE=${TL_BASKET_ARUCO_FILE:-fixed taught basket mode}"
echo "  TL_BASKET_NUMBER=${TL_BASKET_NUMBER:-live vision auto-selection}"
echo "  TL_BASKET_MOTION_MODE=${TL_BASKET_MOTION_MODE}"
echo "  TL_MAX_BASKET_SHIFT_MM=${TL_MAX_BASKET_SHIFT_MM}"
echo "  TL_USE_HAND=${TL_USE_HAND}"
echo "  TL_HAND_GRASP_POSITIONS=${TL_HAND_GRASP_POSITIONS}"
echo "  TL_HAND_OPEN_POSITIONS=${TL_HAND_OPEN_POSITIONS}"
echo "  TL_TARGET_ONLY=${TL_TARGET_ONLY}"
echo "  TL_PLACE_ONLY=${TL_PLACE_ONLY}"
echo "  TL_USE_FIXED_OBSERVATION=${TL_USE_FIXED_OBSERVATION}"
echo "  TL_OBSERVATION_JOINTS=${TL_OBSERVATION_JOINTS}"
echo "  TL_HOME_JOINT_DEG=${TL_HOME_JOINT_DEG}"
echo "  TL_MOTION_BACKEND=${TL_MOTION_BACKEND}"
echo "  ServoJ speed/precision=${TL_SERVOJ_SPEED_DEG_S}/${TL_SERVOJ_PRECISION_SPEED_DEG_S} deg/s"
if [[ "$TL_MOTION_BACKEND" == "movej" ]]; then
    echo "  MoveJ velocity/acc/dec=${TL_MOVEJ_VELOCITY}/${TL_MOVEJ_ACCELERATION}/${TL_MOVEJ_DECELERATION}"
    echo "  MoveJ timeout/retries=${TL_MOTION_TIMEOUT}s/${TL_MOVEJ_MAX_RETRIES}"
elif [[ "$TL_MOTION_BACKEND" != "servoj" ]]; then
    echo "ERROR: TL_MOTION_BACKEND must be servoj or movej"
    exit 2
fi
echo "  expected observation TCP about: -8.693 27.696 414.135 1.795 0.071 1.517"
echo "Fake-hand mode is default. Set TL_USE_HAND=1 only after hand guard limits are verified."
echo "The arm moves to the configured observation joints immediately after startup."

if [[ "$TL_MOTION_BACKEND" == "servoj" ]] && command -v ros2 >/dev/null 2>&1; then
    active_nodes="$(ros2 node list 2>/dev/null || true)"
    if printf '%s\n' "$active_nodes" | grep -Eq '(^|/)safe_arm_node$'; then
        echo "ERROR: /safe_arm_node is running. Direct ServoJ mode cannot share /tl_driver/set_servoj_pos."
        echo "Stop safety_only.launch.py (Ctrl+C in its terminal), then run this script again."
        exit 2
    fi
fi

motion_module="tl_hand_deploy.mock_pick_place"
if [[ "$TL_MOTION_BACKEND" == "movej" ]]; then
    motion_module="tl_hand_deploy.mock_pick_place_movej"
fi

python3 -m "$motion_module" \
    --calibration "${TL_CALIBRATION:-$HOME/tl_handeye_data/best/handeye_best.yaml}" \
    --place-file "$TL_PLACE_FILE" \
    --base-xy-bias-mm $TL_BASE_XY_BIAS_MM \
    --grasp-rpy-rad $TL_GRASP_RPY_RAD \
    --mock-grasp-offset-mm $TL_MOCK_GRASP_OFFSET_MM \
    --pick-approach-clearance-mm "$TL_PICK_APPROACH_CLEARANCE_MM" \
    --pick-grasp-clearance-mm "$TL_PICK_GRASP_CLEARANCE_MM" \
    --basket-motion-mode "$TL_BASKET_MOTION_MODE" \
    --max-basket-shift-mm "$TL_MAX_BASKET_SHIFT_MM" \
    --interp-points "$TL_SERVOJ_POINTS" \
    --rate-hz "$TL_SERVOJ_RATE_HZ" \
    --servoj-speed-deg-s "$TL_SERVOJ_SPEED_DEG_S" \
    --servoj-precision-speed-deg-s "$TL_SERVOJ_PRECISION_SPEED_DEG_S" \
    --servoj-min-duration-sec "$TL_SERVOJ_MIN_DURATION_SEC" \
    --movej-velocity "$TL_MOVEJ_VELOCITY" \
    --movej-acceleration "$TL_MOVEJ_ACCELERATION" \
    --movej-deceleration "$TL_MOVEJ_DECELERATION" \
    --movej-max-retries "$TL_MOVEJ_MAX_RETRIES" \
    --movej-retry-max-error-deg "$TL_MOVEJ_RETRY_MAX_ERROR_DEG" \
    --movej-status-grace-sec "$TL_MOVEJ_STATUS_GRACE_SEC" \
    --movej-stall-sec "$TL_MOVEJ_STALL_SEC" \
    --movej-progress-log-sec "$TL_MOVEJ_PROGRESS_LOG_SEC" \
    --timeout-sec "$TL_MOTION_TIMEOUT" \
    --max-joint-delta-deg "$TL_MAX_JOINT_DELTA_DEG" \
    "${extra_args[@]}" \
    --allow-rejected \
    --execute

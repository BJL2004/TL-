#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
vendor_dir="${TL_DRIVER_DIR:-$HOME/Desktop/tl_robot_ros2_py-arm}"
calibration_file="${CALIBRATION_FILE:-$HOME/tl_handeye_data/best/handeye_best.yaml}"
install_prefix="$project_dir/install/tl_hand_deploy"
extra_args=()

if [[ -n "${TL_GRASP_RPY_RAD:-}" && -n "${TL_GRASP_RPY_DEG:-}" ]]; then
    echo "Use only one of TL_GRASP_RPY_RAD or TL_GRASP_RPY_DEG" >&2
    exit 2
fi
if [[ -n "${TL_GRASP_RPY_RAD:-}" ]]; then
    read -r grasp_rx grasp_ry grasp_rz <<<"$TL_GRASP_RPY_RAD"
    if [[ -z "${grasp_rx:-}" || -z "${grasp_ry:-}" || -z "${grasp_rz:-}" ]]; then
        echo "TL_GRASP_RPY_RAD must contain three numbers" >&2
        exit 2
    fi
    extra_args+=(--grasp-rpy-rad "$grasp_rx" "$grasp_ry" "$grasp_rz")
fi
if [[ -n "${TL_GRASP_RPY_DEG:-}" ]]; then
    read -r grasp_rx grasp_ry grasp_rz <<<"$TL_GRASP_RPY_DEG"
    if [[ -z "${grasp_rx:-}" || -z "${grasp_ry:-}" || -z "${grasp_rz:-}" ]]; then
        echo "TL_GRASP_RPY_DEG must contain three numbers" >&2
        exit 2
    fi
    extra_args+=(--grasp-rpy-deg "$grasp_rx" "$grasp_ry" "$grasp_rz")
fi
if [[ -n "${TL_BASE_XY_BIAS_MM:-}" ]]; then
    read -r bias_x bias_y <<<"$TL_BASE_XY_BIAS_MM"
    if [[ -z "${bias_x:-}" || -z "${bias_y:-}" ]]; then
        echo "TL_BASE_XY_BIAS_MM must contain two numbers, for example: -20.7 5.3" >&2
        exit 2
    fi
    extra_args+=(--base-xy-bias-mm "$bias_x" "$bias_y")
fi
if [[ -n "${TL_BASE_Z_BIAS_MM:-}" ]]; then
    extra_args+=(--base-z-bias-mm "$TL_BASE_Z_BIAS_MM")
fi
if [[ -n "${TL_OBSERVE_JOINT_DEG:-}" ]]; then
    read -r obs_j1 obs_j2 obs_j3 obs_j4 obs_j5 obs_j6 <<<"$TL_OBSERVE_JOINT_DEG"
    if [[ -z "${obs_j1:-}" || -z "${obs_j2:-}" || -z "${obs_j3:-}" || -z "${obs_j4:-}" || -z "${obs_j5:-}" || -z "${obs_j6:-}" ]]; then
        echo "TL_OBSERVE_JOINT_DEG must contain six numbers: J1..J6 in degrees" >&2
        exit 2
    fi
    extra_args+=(--observe-joint-deg "$obs_j1" "$obs_j2" "$obs_j3" "$obs_j4" "$obs_j5" "$obs_j6")
fi
if [[ -n "${TL_MIN_TOOL_Z_MM:-}" ]]; then
    extra_args+=(--min-tool-z-mm "$TL_MIN_TOOL_Z_MM")
fi
if [[ -n "${TL_MIN_MOCK_CLEARANCE_MM:-}" ]]; then
    extra_args+=(--min-mock-clearance-mm "$TL_MIN_MOCK_CLEARANCE_MM")
fi
if [[ -n "${TL_FINAL_GRASP_Z_MM:-}" ]]; then
    extra_args+=(--final-grasp-z-mm "$TL_FINAL_GRASP_Z_MM")
fi
if [[ -n "${TL_MOCK_GRASP_OFFSET_MM:-}" ]]; then
    read -r mock_x mock_y mock_z <<<"$TL_MOCK_GRASP_OFFSET_MM"
    if [[ -z "${mock_x:-}" || -z "${mock_y:-}" || -z "${mock_z:-}" ]]; then
        echo "TL_MOCK_GRASP_OFFSET_MM must contain three numbers: X Y Z in tool0 frame" >&2
        exit 2
    fi
    extra_args+=(--mock-grasp-offset-mm "$mock_x" "$mock_y" "$mock_z")
    extra_args+=(--mock-grasp-clearance-mm "${TL_MOCK_GRASP_CLEARANCE_MM:-30}")
fi
if [[ "${TL_AUTO_LIFT_TO_REACHABLE:-0}" == "1" ]]; then
    extra_args+=(--auto-lift-to-reachable)
    extra_args+=(--max-auto-lift-mm "${TL_MAX_AUTO_LIFT_MM:-200}")
fi

source /opt/ros/humble/setup.bash
source "$vendor_dir/install/setup.bash"
export AMENT_PREFIX_PATH="$install_prefix:${AMENT_PREFIX_PATH:-}"
export PATH="$install_prefix/lib/tl_hand_deploy:$PATH"
export PYTHONPATH="$project_dir/src/tl_hand_deploy:$project_dir/build/tl_hand_deploy:${PYTHONPATH:-}"

if [[ ! -f "$calibration_file" ]]; then
    echo "Calibration file not found: $calibration_file" >&2
    exit 2
fi

topic_list="$(ros2 topic list)"
for topic_name in /tcp_pose /joint_states; do
    if ! grep -Fxq "$topic_name" <<<"$topic_list"; then
        echo "Required topic is missing: $topic_name" >&2
        exit 3
    fi
done

service_list="$(ros2 service list)"
for service_name in /tl_driver/coord_transform /tl_driver/open_servoj /tl_driver/close_servoj; do
    if ! grep -Fxq "$service_name" <<<"$service_list"; then
        echo "Required service is missing: $service_name" >&2
        exit 4
    fi
done

cleanup() {
    set +e
    ros2 service call /tl_driver/close_servoj std_srvs/srv/Trigger "{}" >/dev/null 2>&1
}
trap cleanup EXIT INT TERM

if grep -Fxq /tl_driver/set_speed <<<"$service_list"; then
    ros2 service call /tl_driver/set_speed \
        tl_ros2_interface/srv/SetSpeed "{speed: ${TL_GLOBAL_SPEED:-20.0}}" >/dev/null
fi

echo "Direct TL_TCB-compatible mode: CoordTransform + adaptive minimum-jerk/100-Hz ServoJ."
echo "Window: click object, c calculate, inspect output, m move, r return, q quit."
echo "No safety lock is used. Keep the physical E-stop ready."
if [[ -n "${TL_MOCK_GRASP_OFFSET_MM:-}" ]]; then
    echo "Mock-hand mode: virtual grasp TCP offset=${TL_MOCK_GRASP_OFFSET_MM}, clearance=${TL_MOCK_GRASP_CLEARANCE_MM:-30} mm."
else
    echo "No gripper; hover defaults to 100 mm."
fi
if [[ ${#extra_args[@]} -gt 0 ]]; then
    echo "Extra overhead-test args: ${extra_args[*]}"
else
    echo "Using current observation RPY as no-hand target RPY."
fi

python3 -m tl_hand_deploy.overhead_test \
    --calibration "$calibration_file" \
    --allow-rejected \
    --hover-mm "${TL_HOVER_MM:-100}" \
    --interp-points "${TL_SERVOJ_POINTS:-0}" \
    --rate-hz "${TL_SERVOJ_RATE_HZ:-100}" \
    --servoj-speed-deg-s "${TL_SERVOJ_SPEED_DEG_S:-20}" \
    --servoj-precision-speed-deg-s "${TL_SERVOJ_PRECISION_SPEED_DEG_S:-8}" \
    --servoj-min-duration-sec "${TL_SERVOJ_MIN_DURATION_SEC:-1.0}" \
    --timeout-sec "${TL_MOTION_TIMEOUT:-60}" \
    --max-joint-delta-deg "${TL_MAX_JOINT_DELTA_DEG:-120}" \
    --fine-step-mm "${TL_FINE_STEP_MM:-1}" \
    --fine-z-step-mm "${TL_FINE_Z_STEP_MM:-2}" \
    "${extra_args[@]}" \
    --execute

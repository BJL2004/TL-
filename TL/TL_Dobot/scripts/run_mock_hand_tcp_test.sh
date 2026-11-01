#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
vendor_dir="${TL_DRIVER_DIR:-$HOME/Desktop/tl_robot_ros2_py-arm}"
calibration_file="${CALIBRATION_FILE:-$HOME/tl_handeye_data/session_20260808_022900/calibration/handeye_candidate_rejected.yaml}"
mock_offset_text="${MOCK_GRASP_OFFSET_MM:-0 0 -150}"
mock_clearance="${MOCK_GRASP_CLEARANCE_MM:-20}"
extra_args=()

read -r mock_x mock_y mock_z <<<"$mock_offset_text"
if [[ -z "${mock_x:-}" || -z "${mock_y:-}" || -z "${mock_z:-}" ]]; then
    echo "MOCK_GRASP_OFFSET_MM must contain three numbers, for example: '0 0 -150'" >&2
    exit 2
fi
if [[ -n "${MOCK_GRASP_RPY_RAD:-}" && -n "${MOCK_GRASP_RPY_DEG:-}" ]]; then
    echo "Use only one of MOCK_GRASP_RPY_RAD or MOCK_GRASP_RPY_DEG" >&2
    exit 2
fi
if [[ -n "${MOCK_GRASP_RPY_RAD:-}" ]]; then
    read -r grasp_rx grasp_ry grasp_rz <<<"$MOCK_GRASP_RPY_RAD"
    if [[ -z "${grasp_rx:-}" || -z "${grasp_ry:-}" || -z "${grasp_rz:-}" ]]; then
        echo "MOCK_GRASP_RPY_RAD must contain three numbers" >&2
        exit 2
    fi
    extra_args+=(--grasp-rpy-rad "$grasp_rx" "$grasp_ry" "$grasp_rz")
fi
if [[ -n "${MOCK_GRASP_RPY_DEG:-}" ]]; then
    read -r grasp_rx grasp_ry grasp_rz <<<"$MOCK_GRASP_RPY_DEG"
    if [[ -z "${grasp_rx:-}" || -z "${grasp_ry:-}" || -z "${grasp_rz:-}" ]]; then
        echo "MOCK_GRASP_RPY_DEG must contain three numbers" >&2
        exit 2
    fi
    extra_args+=(--grasp-rpy-deg "$grasp_rx" "$grasp_ry" "$grasp_rz")
fi

source /opt/ros/humble/setup.bash
source "$vendor_dir/install/setup.bash"
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

echo "Mock hand TCP mode: virtual grasp point offset tool0=[$mock_x, $mock_y, $mock_z] mm."
echo "Clearance from clicked target: ${mock_clearance} mm."
if [[ ${#extra_args[@]} -gt 0 ]]; then
    echo "Using fixed grasp RPY: ${extra_args[*]}"
else
    echo "Using current observation RPY as grasp RPY."
fi
echo "Window: click target center, c calculate, inspect output, m move, a/d/s/w fine tune, v save, r return, q quit."
echo "No safety lock is used. Keep the physical E-stop ready."

python3 -m tl_hand_deploy.overhead_test \
    --calibration "$calibration_file" \
    --allow-rejected \
    --mock-grasp-offset-mm "$mock_x" "$mock_y" "$mock_z" \
    --mock-grasp-clearance-mm "$mock_clearance" \
    --interp-points "${TL_SERVOJ_POINTS:-500}" \
    --rate-hz "${TL_SERVOJ_RATE_HZ:-100}" \
    --timeout-sec "${TL_MOTION_TIMEOUT:-60}" \
    "${extra_args[@]}" \
    --execute

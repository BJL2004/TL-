#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
export PYTHONPATH="$project_dir/src/tl_hand_deploy:${PYTHONPATH:-}"
source /opt/ros/humble/setup.bash
driver_dir="${TL_DRIVER_DIR:-$HOME/Desktop/tl_robot_ros2_py-arm}"
if [[ -f "$driver_dir/install/setup.bash" ]]; then
    source "$driver_dir/install/setup.bash"
fi

export TL_MQTT_HOST="${TL_MQTT_HOST:-127.0.0.1}"
export TL_MQTT_PORT="${TL_MQTT_PORT:-1883}"
export TL_BASKET_ARUCO_FILE="${TL_BASKET_ARUCO_FILE:-$HOME/tl_mock_pick_place/basket_aruco.yaml}"
export TL_OBSERVATION_JOINTS="${TL_OBSERVATION_JOINTS:--83.668 67.725 3.956 -2.101 -5.484 -1.431}"
export TL_MQTT_TARGET_MAP="${TL_MQTT_TARGET_MAP:-}"

read -r o1 o2 o3 o4 o5 o6 <<<"$TL_OBSERVATION_JOINTS"
for value in "$o1" "$o2" "$o3" "$o4" "$o5" "$o6"; do
    [[ -n "$value" ]] || { echo "TL_OBSERVATION_JOINTS must contain six degrees" >&2; exit 2; }
done

echo "U-fork basket MQTT test (direct ServoJ; no safety proxy; no hand)."
echo "Teach file: $TL_BASKET_ARUCO_FILE"
echo "trim_pick -> approach -> insert -> lift -> transfer"
echo "trim_place -> original lift -> insert -> retract -> observation"
echo "Manual keys: B localize, K pick, P place, 1..6 select, R return, Q quit."

extra_args=()
if [[ -n "${TL_BASKET_NUMBER:-}" ]]; then
    extra_args+=(--basket-number "$TL_BASKET_NUMBER")
fi
if [[ -n "$TL_MQTT_TARGET_MAP" ]]; then
    extra_args+=(--mqtt-target-map "$TL_MQTT_TARGET_MAP")
fi

python3 -m tl_hand_deploy.fork_basket_controller \
    --calibration "${TL_CALIBRATION:-$HOME/tl_handeye_data/best/handeye_best.yaml}" \
    --basket-aruco-file "$TL_BASKET_ARUCO_FILE" \
    --observation-joints "$o1" "$o2" "$o3" "$o4" "$o5" "$o6" \
    --mqtt-enabled \
    --mqtt-host "$TL_MQTT_HOST" \
    --mqtt-port "$TL_MQTT_PORT" \
    --mqtt-command-topic "${TL_MQTT_COMMAND_TOPIC:-tl/robot/command}" \
    --mqtt-feedback-topic "${TL_MQTT_FEEDBACK_TOPIC:-tl/robot/feedback}" \
    --mqtt-client-id "${TL_MQTT_CLIENT_ID:-tl-tcb605-fork}" \
    --mqtt-username "${TL_MQTT_USERNAME:-}" \
    --mqtt-password "${TL_MQTT_PASSWORD:-}" \
    --mqtt-keepalive "${TL_MQTT_KEEPALIVE:-60}" \
    --servoj-speed-deg-s "${TL_SERVOJ_SPEED_DEG_S:-15}" \
    --servoj-precision-speed-deg-s "${TL_SERVOJ_PRECISION_SPEED_DEG_S:-8}" \
    --servoj-min-duration-sec "${TL_SERVOJ_MIN_DURATION_SEC:-1.0}" \
    --rate-hz "${TL_SERVOJ_RATE_HZ:-100}" \
    --timeout-sec "${TL_MOTION_TIMEOUT:-180}" \
    --max-joint-delta-deg "${TL_MAX_JOINT_DELTA_DEG:-220}" \
    --marker-samples "${TL_MARKER_SAMPLES:-30}" \
    --marker-minimum-samples "${TL_MARKER_MINIMUM_SAMPLES:-8}" \
    --basket-motion-mode translation \
    --allow-rejected \
    --execute \
    "${extra_args[@]}"

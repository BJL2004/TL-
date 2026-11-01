#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"

export TL_USE_HAND=0
export TL_PLACE_ONLY=1
export TL_MQTT_ENABLED=1
export TL_MQTT_HOST="${TL_MQTT_HOST:-127.0.0.1}"
export TL_MQTT_PORT="${TL_MQTT_PORT:-1883}"

if ! python3 -c "import paho.mqtt.client" >/dev/null 2>&1; then
    echo "ERROR: Python package paho-mqtt is missing." >&2
    echo "Install it with: sudo apt install python3-paho-mqtt" >&2
    exit 2
fi

echo "MQTT mouse-guided pick-place test (no gripper/hand commands)."
echo "Broker: ${TL_MQTT_HOST}:${TL_MQTT_PORT}"
echo "Wait for 'MQTT connected and subscribed' before publishing seed."
echo "Workflow: publish seed -> click object -> C -> R -> G."

exec bash "$project_dir/scripts/run_dynamic_aruco_basket_test.sh"

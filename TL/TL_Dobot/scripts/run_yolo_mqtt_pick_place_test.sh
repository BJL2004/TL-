#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"

export TL_USE_HAND="${TL_USE_HAND:-0}"
export TL_PLACE_ONLY="${TL_PLACE_ONLY:-0}"
export TL_MQTT_ENABLED=1
export TL_MQTT_HOST="${TL_MQTT_HOST:-127.0.0.1}"
export TL_MQTT_PORT="${TL_MQTT_PORT:-1883}"
export TL_FIXED_PICK_BASE_MM="${TL_FIXED_PICK_BASE_MM:-380.37 74.1 52.31}"
export TL_YOLO_MODEL="${TL_YOLO_MODEL:-$HOME/yolo_models/best.pt}"
export TL_YOLO_CONF="${TL_YOLO_CONF:-0.5}"
export TL_YOLO_IMGSZ="${TL_YOLO_IMGSZ:-640}"

if ! python3 -c "import ultralytics" >/dev/null 2>&1; then
    echo "ERROR: Python package ultralytics is missing." >&2
    echo "Install it with: pip install ultralytics" >&2
    exit 2
fi

if [[ ! -f "$TL_YOLO_MODEL" ]]; then
    echo "ERROR: YOLO model file not found: $TL_YOLO_MODEL" >&2
    echo "Copy your trained best.pt to this path or set TL_YOLO_MODEL." >&2
    exit 2
fi

echo "YOLO + MQTT pick-place test."
echo "Broker: ${TL_MQTT_HOST}:${TL_MQTT_PORT}"
echo "Fixed pick base[mm]: ${TL_FIXED_PICK_BASE_MM}"
echo "YOLO model: ${TL_YOLO_MODEL}"
echo "YOLO conf=${TL_YOLO_CONF}, imgsz=${TL_YOLO_IMGSZ}"
echo "Wait for 'MQTT connected and subscribed' before publishing seed."
echo "Workflow: publish seed -> YOLO auto-detect -> auto pick -> R localize -> G place."

exec bash "$project_dir/scripts/run_dynamic_aruco_basket_test.sh"
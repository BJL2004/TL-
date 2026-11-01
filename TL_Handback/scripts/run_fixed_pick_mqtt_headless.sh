#!/usr/bin/env bash
# 无画面版 MQTT 固定点抓取节点（无预览窗口/无键盘/无鼠标，纯 MQTT 驱动）
# ==================================================
# 与 run_fixed_pick_mqtt.sh 功能完全一致，只是不弹画面窗口。
# 先在一个终端启动驱动:
#   ros2 launch tl_driver tl_tcb605_driver.launch.py
# 再在本脚本终端启动控制节点:
#   bash run_fixed_pick_mqtt_headless.sh
#
# 流程：MQTT seed 命令 -> 固定点抓取(力度5) -> ArUco 定位花篮
#       -> 放置释放(力度1保护) -> 回检测位 -> 回观测位
# MQTT: 公网 broker 116.205.245.172:1883（shapandevice/qwer1234!!）
#       topic: node/arm/01/down|up；estop 急停命令可用
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"

# Load ROS2 and the TL driver message workspace; override with TL_DRIVER_DIR.
source /opt/ros/humble/setup.bash
driver_dir="${TL_DRIVER_DIR:-$HOME/Desktop/tl_robot_ros2_py-arm}"
if [[ -f "$driver_dir/install/setup.bash" ]]; then
    source "$driver_dir/install/setup.bash"
fi

# TL_Handback 自己的编译产物（提供 tl_hand 包）
if [[ -d "$project_dir/install" ]]; then
    source "$project_dir/install/setup.bash" 2>/dev/null || true
fi

# 显式注册 TL_Handback 的 install 前缀
if [[ -d "$project_dir/install/tl_hand/share/tl_hand" ]]; then
    export AMENT_PREFIX_PATH="$project_dir/install/tl_hand:$project_dir/install:${AMENT_PREFIX_PATH:-}"
elif [[ -d "$project_dir/install/share/tl_hand" ]]; then
    export AMENT_PREFIX_PATH="$project_dir/install:${AMENT_PREFIX_PATH:-}"
else
    echo "ERROR: TL_Handback 编译产物缺失，请先执行:" >&2
    echo "  cd $project_dir && colcon build --packages-select tl_hand" >&2
    exit 2
fi

# 剔除旧 TL_Hand/install 残留
export AMENT_PREFIX_PATH="$(printf '%s\n' "$AMENT_PREFIX_PATH" \
    | tr ':' '\n' \
    | grep -v '/TL_Hand/install' \
    | grep -v '^$' \
    | paste -sd: -)"

# 验证 tl_hand 包确实来自 TL_Handback
tl_hand_prefix="$(ros2 pkg prefix tl_hand 2>/dev/null || true)"
echo "tl_hand package prefix: ${tl_hand_prefix:-<not found>}"
if [[ -z "$tl_hand_prefix" ]]; then
    echo "ERROR: tl_hand 包未找到；检查: ls $project_dir/install/tl_hand/share/tl_hand" >&2
    exit 2
fi

# 凭据：用户名/密码都非空时才传给节点（MqttClient 用户名空则不发密码）
# 默认走公网 broker 的账号；本地匿名测试时显式置空即可
TL_MQTT_USERNAME="${TL_MQTT_USERNAME:-shapandevice}"
TL_MQTT_PASSWORD="${TL_MQTT_PASSWORD:-qwer1234!!}"
mqtt_auth_args=()
if [[ -n "$TL_MQTT_USERNAME" ]]; then
    mqtt_auth_args+=(--mqtt-username "$TL_MQTT_USERNAME")
fi
if [[ -n "$TL_MQTT_PASSWORD" ]]; then
    mqtt_auth_args+=(--mqtt-password "$TL_MQTT_PASSWORD")
fi

# 无画面版：直接运行 headless 可执行文件（与窗口版共用同一份 launch 参数）
exec ros2 run tl_hand tl_mock_pick_place_headless \
    --calibration "${TL_CALIBRATION:-$HOME/tl_handeye_data/best/handeye_best.yaml}" \
    --place-file "$HOME/tl_mock_pick_place/place_pose.yaml" \
    --basket-aruco-file "$HOME/tl_mock_pick_place/basket_aruco.yaml" \
    --base-xy-bias-mm -53.2 -34.2 \
    --grasp-rpy-rad 1.839 1.239 1.343 \
    --mock-grasp-offset-mm -6.3 34.0 42.5 \
    --pick-approach-clearance-mm 130 \
    --pick-grasp-clearance-mm 50 \
    --final-grasp-z-mm 68.937 \
    --basket-motion-mode translation \
    --max-basket-shift-mm 200 \
    --max-place-reachability-shift-mm 80 \
    --marker-sample-spread-mm 60 \
    --interp-points 0 \
    --rate-hz 100 \
    --servoj-speed-deg-s 40 \
    --servoj-precision-speed-deg-s 20 \
    --servoj-min-duration-sec 0.3 \
    --hand-settle-sec 1.0 \
    --auto-lift-to-reachable \
    --max-auto-lift-mm 200 \
    --mqtt-enabled \
    --mqtt-host "${TL_MQTT_HOST:-116.205.245.172}" \
    --mqtt-port "${TL_MQTT_PORT:-1883}" \
    "${mqtt_auth_args[@]}" \
    --mqtt-command-topic "${TL_MQTT_CMD_TOPIC:-node/arm/01/down}" \
    --mqtt-feedback-topic "${TL_MQTT_FB_TOPIC:-node/arm/01/up}" \
    --mqtt-target-map "" \
    --mqtt-client-id tl-tcb605-arm \
    --mqtt-keepalive 60 \
    --timeout-sec 90 \
    --max-joint-delta-deg 220 \
    --allow-rejected \
    --execute \
    --fixed-pick-base-mm 500.0 -250.94 58.1 \
    --observation-joints -23.246 -14.220 53.738 4.389 26.746 -3.532 \
    --home-joint-deg -23.246 -14.220 53.738 4.389 26.746 -3.532 \
    --use-hand \
    --hand-grasp-positions 35000 35000 35000 35000 35000 25000 \
    --hand-open-positions 0 0 0 0 0 0 \
    --hand-hold-positions 8000 8000 8000 8000 8000 5000 \
    --hand-level5-ratio 0.85 \
    "$@"

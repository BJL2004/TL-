#!/usr/bin/env bash
# MQTT 驱动的 U 叉修剪循环节点（无窗口，纯 MQTT 命令驱动）
# ==================================================
# 协议（车 <-> 机械臂，broker 默认公网 116.205.245.172:1883）：
#   车->机械臂  node/arm/01/down  {"cmd":"trim_pick","targetId":"A1-01"}
#   机械臂->车  node/arm/01/up    {"cmd":"trim_pick","targetId":"A1-01","retCode":0|1|2|3}
#   车->机械臂  node/arm/01/down  {"cmd":"trim_place","targetId":"A1-01"}
#   机械臂->车  node/arm/01/up    {"cmd":"trim_place","targetId":"A1-01","retCode":0|1|2|3}
#   车->机械臂  node/arm/01/down  {"cmd":"estop"}   （无 targetId）
#   机械臂->车  node/arm/01/up    {"cmd":"estop","retCode":0}
#   retCode: 0=成功 1=响应命令开始动作(先发) 2=位置被占 3=识别不成功/放置失败
#   targetId: A1-01..A1-06（叉子只教了 A1 排）；也兼容旧数字 1..6
#   estop: 收到后立即中止当前运动（ServoJ 流停止），之后其他命令全部拒绝
#          （retCode 3），需重启节点清除；真正下电请用物理 E-stop
# 修剪位置暂时用示教 T（固定暂放位）代替。
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source /opt/ros/humble/setup.bash
driver_dir="${TL_DRIVER_DIR:-$HOME/Desktop/tl_robot_ros2_py-arm}"
if [[ -f "$driver_dir/install/setup.bash" ]]; then
    source "$driver_dir/install/setup.bash"
fi
if [[ -d "$project_dir/install" ]]; then
    source "$project_dir/install/setup.bash" 2>/dev/null || true
fi
export PYTHONPATH="$project_dir/src/tl_fork_teach:${PYTHONPATH:-}"

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

exec python3 -m tl_fork_teach.fork_trim_mqtt \
    --calibration "${TL_CALIBRATION:-$HOME/tl_handeye_data/best/handeye_best.yaml}" \
    --output "${TL_FORK_TEACH_OUTPUT:-$HOME/tl_fork_teach/fork_basket_teach.yaml}" \
    --marker-length-m "${TL_MARKER_LENGTH_M:-0.056}" \
    --mqtt-host "${TL_MQTT_HOST:-116.205.245.172}" \
    --mqtt-port "${TL_MQTT_PORT:-1883}" \
    --mqtt-command-topic "${TL_MQTT_CMD_TOPIC:-node/arm/01/down}" \
    --mqtt-feedback-topic "${TL_MQTT_FB_TOPIC:-node/arm/01/up}" \
    --mqtt-client-id "${TL_MQTT_CLIENT_ID:-tl-tcb605-fork}" \
    "${mqtt_auth_args[@]}" \
    --servoj-speed-deg-s "${TL_SERVOJ_SPEED_DEG_S:-30}" \
    --servoj-min-duration-sec "${TL_SERVOJ_MIN_DURATION_SEC:-0.3}" \
    --trim-hold-speed-deg-s "${TL_TRIM_HOLD_SPEED_DEG_S:-12}" \
    --trim-hold-segments "${TL_TRIM_HOLD_SEGMENTS:-6}" \
    --trim-lower-clearance-mm "${TL_TRIM_LOWER_CLEARANCE_MM:-150}" \
    --allow-rejected \
    --execute \
    "$@"

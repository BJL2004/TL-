#!/usr/bin/env bash
# ==================================================
# 叉子修剪 MQTT 节点 · 下排观测位验证版（独立运行，不读旧 TL 安装包）
#   trim_pick  下排盆（A1/A4/B1/B4）→ 先去对应侧【下排观测位】识别码 →
#              再到检索位叉取、抬升、搬运、修剪位；
#              上排盆 → 行为与旧版一致（检索位识别）。
#   trim_place → 返回路径不变。
#   待机位（启动/作业结束回位）：与种子程序统一（-23.246 …，2026-10-07 起）。
#   协议：车->机械臂 node/arm/01/down {"cmd":"trim_pick","targetId":"A1-01"}
#        retCode: 0=成功 1=已接受 2=位置被占 3=失败（含“观测位未记录”提示）
# 提醒：
#   1) 驱动需先在另一终端启动
#   2) 同一时间不要与旧版 run_fork_mqtt.sh 同时运行（MQTT client-id 会冲突）
# ==================================================
set -eo pipefail
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source /opt/ros/humble/setup.bash
driver_dir="${TL_DRIVER_DIR:-$HOME/Desktop/tl_robot_ros2_py-arm}"
if [[ -f "$driver_dir/install/setup.bash" ]]; then
    source "$driver_dir/install/setup.bash"
fi
export PYTHONPATH="$root_dir/src/tl_fork_teach:$root_dir/src/tl_hand:${PYTHONPATH:-}"

# 凭据：用户名/密码都非空时才传给节点；本地匿名测试时显式置空
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
    --reversed-rows ${TL_REVERSED_ROWS:-A3} \
    --output "${TL_FORK_TEACH_OUTPUT:-$HOME/tl_fork_teach/fork_basket_teach.yaml}" \
    --observation-joints -23.246 -14.220 53.738 4.389 26.746 -3.532 \
    --marker-length-m "${TL_MARKER_LENGTH_M:-0.054}" \
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

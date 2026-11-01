#!/usr/bin/env bash
# ==================================================
# 种子抓取放置（MQTT）· 下排观测位验证版（独立运行，不读旧 TL 安装包）
# 与 ros2 launch tl_hand tl_hand_pick_place.launch.py use_hand:=true 参数一致，
# 但直接跑本目录源码：
#   下排盆（A1/A4/B1/B4）→ 先去对应侧【下排观测位】识别码 → 再去检索位放置；
#   上排盆（A2/A3/B2/B3）→ 行为与旧版完全一致（检索位识别）。
# 提醒：
#   1) 驱动需先在另一终端启动：ros2 launch tl_driver tl_tcb605_driver.launch.py
#   2) 同一时间不要与旧版 run_seed_mqtt.sh 同时运行（MQTT client-id 会冲突）
# ==================================================
set -eo pipefail
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source /opt/ros/humble/setup.bash
driver_dir="${TL_DRIVER_DIR:-$HOME/Desktop/tl_robot_ros2_py-arm}"
if [[ -f "$driver_dir/install/setup.bash" ]]; then
    source "$driver_dir/install/setup.bash"
else
    echo "ERROR: TL driver workspace is missing: $driver_dir/install/setup.bash" >&2
    exit 2
fi
# 本目录源码优先（PYTHONPATH 在 site-packages / 旧 install 之前）
export PYTHONPATH="$root_dir/src/tl_hand:${PYTHONPATH:-}"
export RCUTILS_LOGGING_MIN_SEVERITY="${RCUTILS_LOGGING_MIN_SEVERITY:-WARN}"

exec python3 -m tl_hand.mock_pick_place \
    --calibration "${TL_CALIBRATION:-$HOME/tl_handeye_data/best/handeye_best.yaml}" \
    --reversed-rows ${TL_REVERSED_ROWS:-A3} \
    --place-file "${TL_PLACE_FILE:-$HOME/tl_mock_pick_place/place_pose.yaml}" \
    --basket-aruco-file "${TL_BASKET_ARUCO_FILE:-$HOME/tl_mock_pick_place/basket_aruco.yaml}" \
    --base-xy-bias-mm -53.2 -34.2 \
    --grasp-rpy-rad 1.607 1.337 1.553 \
    --mock-grasp-offset-mm -6.3 34.0 42.5 \
    --pick-approach-clearance-mm 130 \
    --pick-grasp-clearance-mm 50 \
    --final-grasp-z-mm 218.880 \
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
    --mqtt-username "${TL_MQTT_USERNAME:-shapandevice}" \
    --mqtt-password "${TL_MQTT_PASSWORD:-qwer1234!!}" \
    --mqtt-command-topic node/arm/01/down \
    --mqtt-feedback-topic node/arm/01/up \
    --mqtt-target-map "" \
    --mqtt-client-id tl-tcb605-arm \
    --mqtt-keepalive 60 \
    --timeout-sec 90 \
    --max-joint-delta-deg 220 \
    --allow-rejected \
    --execute \
    --fixed-pick-base-mm 398.53 -193.69 211.63 \
    --observation-joints -23.246 -14.220 53.738 4.389 26.746 -3.532 \
    --home-joint-deg -23.246 -14.220 53.738 4.389 26.746 -3.532 \
    --use-hand \
    --hand-grasp-positions 35000 35000 35000 35000 35000 25000 \
    --hand-open-positions 0 0 0 0 0 0 \
    --hand-hold-positions 8000 8000 8000 8000 8000 5000 \
    --hand-level5-ratio 0.85 \
    "$@"

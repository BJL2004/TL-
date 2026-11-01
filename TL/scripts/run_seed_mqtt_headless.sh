#!/usr/bin/env bash
# 种子节点：无画面版（灵巧手放种子，MQTT 驱动，公网 broker）
# 与 run_seed_mqtt.sh 行为完全一致，只是不弹预览窗口（适合车端/无显示环境）。
# 支持 seed / seed_place / estop 命令；反馈 cmd 回显收到的命令名。
set -eo pipefail
tl_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
exec bash "$tl_dir/TL_Handback/scripts/run_fixed_pick_mqtt_headless.sh" "$@"

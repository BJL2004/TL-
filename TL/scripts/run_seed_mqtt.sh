#!/usr/bin/env bash
# 种子节点：灵巧手放种子（MQTT 驱动，公网 broker）
set -eo pipefail
tl_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
exec bash "$tl_dir/TL_Handback/scripts/run_fixed_pick_mqtt.sh" "$@"

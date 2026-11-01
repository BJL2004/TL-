#!/usr/bin/env bash
# 叉子修剪节点：叉花篮（MQTT 驱动，公网 broker）
set -eo pipefail
tl_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
exec bash "$tl_dir/TL_Handback/scripts/run_fork_trim_mqtt.sh" "$@"

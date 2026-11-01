#!/usr/bin/env bash
# 列出所有已示教的盆和点位（只读 YAML，不动机械臂）
#   bash list_taught.sh                    全部列出
#   bash list_taught.sh --full             附 TCP 坐标
#   bash list_taught.sh --basket A2-06     只看一盆（A3 贴反换算默认启用）
set -e
python3 "$(dirname "${BASH_SOURCE[0]}")/list_taught.py" \
    --reversed-rows ${TL_REVERSED_ROWS:-A3} \
    "$@"

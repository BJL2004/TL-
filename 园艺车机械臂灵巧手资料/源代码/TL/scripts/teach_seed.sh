#!/usr/bin/env bash
# 种子示教（192 花盆，自动去默认检测位；v 键转 B 侧）
set -eo pipefail
tl_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
exec bash "$tl_dir/TL_Dobot/scripts/teach_aruco_basket.sh" "$@"

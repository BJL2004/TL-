#!/usr/bin/env bash
# 叉花篮示教（192 花盆）
set -eo pipefail
tl_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
exec bash "$tl_dir/TL_Handback/scripts/run_fork_basket_teach.sh" "$@"

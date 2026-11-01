#!/usr/bin/env bash
# 手眼标定采集（直跑源码，绕开 install/launch 环境解析问题）
# 参数取自源码默认：棋盘格 9x6 内角点、方格 25mm（config/handeye.yaml 同值）
# 用法：bash run_handeye_collect.sh
# 提示：驱动必须先在跑；s=保存 q=退出（按在预览窗口上）
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
export PYTHONPATH="$project_dir/src/tl_hand_deploy:${PYTHONPATH:-}"

source /opt/ros/humble/setup.bash
driver_dir="${TL_DRIVER_DIR:-$HOME/Desktop/tl_robot_ros2_py-arm}"
if [[ -f "$driver_dir/install/setup.bash" ]]; then
    source "$driver_dir/install/setup.bash"
fi

exec python3 -m tl_hand_deploy.handeye_collect_node "$@"

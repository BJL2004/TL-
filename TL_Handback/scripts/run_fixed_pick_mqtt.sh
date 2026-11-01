#!/usr/bin/env bash
# 启动 MQTT 固定点抓取节点（无 YOLO，灵巧手模式）—— 仅控制节点，驱动单独启动
# ==================================================
# 先在一个终端启动驱动:
#   ros2 launch tl_driver tl_tcb605_driver.launch.py
# 再在本脚本终端启动控制节点:
#   bash run_fixed_pick_mqtt.sh
#
# 流程：MQTT seed 命令 -> 固定点抓取(力度5) -> ArUco 定位花篮(实时画面)
#       -> 放置释放 -> 回 home；失败时 MQTT 反馈带原因(message 字段)
# 固定抓取点：见 launch/tl_hand_pick_place.launch.py 的 fixed_pick_base_mm 参数
#   （当前固化：500.0 -250.94 58.1，由新 tool0 抓取位 471.067 -205.693 68.937 换算）
# 观测位/home（新臂示教）：-23.246 -14.220 53.738 4.389 26.746 -3.532
# 抓取高度：TL_FINAL_GRASP_Z_MM=68.937（launch 内固化，tool0 绝对 Z）
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"

# Load ROS2 and the TL driver message workspace; override with TL_DRIVER_DIR.
source /opt/ros/humble/setup.bash
driver_dir="${TL_DRIVER_DIR:-$HOME/Desktop/tl_robot_ros2_py-arm}"
if [[ -f "$driver_dir/install/setup.bash" ]]; then
    source "$driver_dir/install/setup.bash"
fi

# TL_Handback 自己的编译产物（提供 tl_hand 包）
if [[ -d "$project_dir/install" ]]; then
    source "$project_dir/install/setup.bash" 2>/dev/null || true
fi

# 显式注册 TL_Handback 的 install 前缀（不依赖 setup.bash 是否生效；
# isolated 布局: install/tl_hand/share/tl_hand，merge 布局: install/share/tl_hand）
if [[ -d "$project_dir/install/tl_hand/share/tl_hand" ]]; then
    export AMENT_PREFIX_PATH="$project_dir/install/tl_hand:$project_dir/install:${AMENT_PREFIX_PATH:-}"
elif [[ -d "$project_dir/install/share/tl_hand" ]]; then
    export AMENT_PREFIX_PATH="$project_dir/install:${AMENT_PREFIX_PATH:-}"
else
    echo "ERROR: TL_Handback 编译产物缺失，请先执行:" >&2
    echo "  cd $project_dir && colcon build --packages-select tl_hand" >&2
    exit 2
fi

# 剔除旧 TL_Hand/install 残留（防止 ros2 解析到旧包/旧参数）
export AMENT_PREFIX_PATH="$(printf '%s\n' "$AMENT_PREFIX_PATH" \
    | tr ':' '\n' \
    | grep -v '/TL_Hand/install' \
    | grep -v '^$' \
    | paste -sd: -)"

# 验证 tl_hand 包确实来自 TL_Handback
tl_hand_prefix="$(ros2 pkg prefix tl_hand 2>/dev/null || true)"
echo "tl_hand package prefix: ${tl_hand_prefix:-<not found>}"
if [[ -z "$tl_hand_prefix" ]]; then
    echo "ERROR: tl_hand 包未找到；检查: ls $project_dir/install/tl_hand/share/tl_hand" >&2
    exit 2
fi

# 用绝对路径启动 launch，彻底绕开包解析歧义
exec ros2 launch \
    "$tl_hand_prefix/share/tl_hand/launch/tl_hand_pick_place.launch.py" \
    use_hand:=true "$@"

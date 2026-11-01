#!/usr/bin/env bash
# 启动 TL 机械臂驱动（必须最先运行，保持常驻）
set -eo pipefail
driver_dir="${TL_DRIVER_DIR:-$HOME/Desktop/tl_robot_ros2_py-arm}"
if [[ ! -f "$driver_dir/install/setup.bash" ]]; then
    echo "ERROR: 驱动未找到: $driver_dir" >&2
    echo "请把驱动放到该路径，或设置 TL_DRIVER_DIR 环境变量" >&2
    exit 2
fi
source /opt/ros/humble/setup.bash
source "$driver_dir/install/setup.bash"
exec ros2 launch tl_driver tl_tcb605_driver.launch.py "$@"

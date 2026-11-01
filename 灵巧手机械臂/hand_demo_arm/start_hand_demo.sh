#!/bin/bash
# ============================================================
# hand_demo_arm 启动脚本 (Jetson Orin Nano)
# ============================================================
# 用法:
#   cd ~/Desktop/hand_demo_arm
#   bash start_hand_demo.sh
# ============================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$HOME/Desktop/tl_robot_ros2_py-arm"

echo "============================================"
echo "  hand_demo_arm 启动脚本"
echo "============================================"

# 1. 检查 ROS2 环境
if [ -z "$ROS_DISTRO" ]; then
    echo "[ERROR] ROS2 环境未加载，请先 source /opt/ros/humble/setup.bash"
    exit 1
fi
echo "[INFO] ROS2 Distro: $ROS_DISTRO"

# 2. 检查 tl_robot workspace
if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    echo "[ERROR] tl_robot workspace 未编译: $WORKSPACE_DIR"
    echo "  请先编译: cd $WORKSPACE_DIR && colcon build --packages-select tl_ros2_interface tl_driver"
    exit 1
fi

# 3. Source workspace
echo "[INFO] 加载 tl_robot workspace..."
source "$WORKSPACE_DIR/install/setup.bash"

# 4. 检查 tl_driver 是否运行
echo "[INFO] 检查 tl_driver 节点..."
if ! ros2 node list 2>/dev/null | grep -q "tl_driver"; then
    echo "[WARN] tl_driver 节点未检测到！"
    echo "  请先在另一个终端启动 tl_driver:"
    echo "    source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash"
    echo "    ros2 launch tl_driver tl_driver.launch.py arm_type:=TCB605"
    echo ""
    echo "  然后重新运行本脚本。"
    echo ""
    read -p "按 Enter 继续（假设 tl_driver 已运行）..."
fi

# 5. 检查 ModBus 主站是否已配置（可选，不强制）
echo "[INFO] 提示: 灵巧手通过机械臂 RS485 端口2 连接"
echo "  参数: slave_id=2, port=2, baudrate=115200"
echo "  首次使用前需配置 ModBus 主站:"
echo "    ros2 service call /tl_driver/modbus_write tl_ros2_interface/srv/ModbusWrite '...'"
echo ""

# 6. 启动 control_node
echo "[INFO] 启动 control_node (ARM版)..."
echo "  配置文件: $SCRIPT_DIR/control_node.yaml"
echo ""

cd "$SCRIPT_DIR"
python3 "$SCRIPT_DIR/control_node.py" \
    --ros-args \
    --params-file "$SCRIPT_DIR/control_node.yaml"

echo ""
echo "============================================"
echo "  hand_demo_arm 已退出"
echo "============================================"

# TCB605 机械臂 + D435 视觉 + ROH 灵巧手 — Jetson 部署包

## 文件清单

| 文件 | 功能 | 依赖 |
|------|------|------|
| `vision_grasp_demo.py` | **主程序**：D435 实时检测 + YOLO + 坐标变换 + ServoJ 抓取 | D435, YOLO, ROS2 |
| `arm_hand_demo.py` | 机械臂 + 灵巧手键盘控制（无摄像头） | ROS2 |
| `collect_data_arm.py` | 手眼标定数据采集（D435 + /tcp_pose） | D435, ROS2 |
| `compute_in_hand_arm.py` | 手眼标定求解（纯 numpy+opencv，离线运行） | numpy, opencv, scipy |
| `test_hand.py` | 灵巧手独立测试（不启动机械臂运动） | ROS2 |
| `hand_controller.py` | ROH-LiteS001 灵巧手 ModBus 控制库 | ROS2 |
| `control_node.yaml` | 参数配置参考文件 | — |

> 复制到 Jetson 的路径：`~/Desktop/TL_TCB/`

---

## 环境要求

### Jetson 端

```bash
# 操作系统
Ubuntu 22.04 + ROS2 Humble + JetPack 6.x

# 天链驱动 (ARM ctypes 版)
~/Desktop/tl_robot_ros2_py-arm/

# Python 包
sudo apt install python3-opencv python3-yaml python3-numpy python3-pip
pip3 install ultralytics pyrealsense2 scipy
```

### YOLO 模型

```bash
mkdir -p ~/.config/Ultralytics
wget -O ~/.config/Ultralytics/yolov8n.pt \
  https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt
```

### 机械臂连接

- 机械臂 IP：`192.168.1.13`
- Jetson 需能 ping 通机械臂

---

## 一、启动 tl_driver

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
ros2 launch tl_driver tl_tcb605_driver.launch.py
```

看到 `servo_state = 3` 即为上电成功。如果上电失败，在示教器上手动上电后重启驱动。

验证数据在线：

```bash
ros2 topic echo /joint_states --once
ros2 topic echo /tcp_pose --once
```

---

## 二、vision_grasp_demo.py（检测 + 抓取）

### 启动

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
cd ~/Desktop/TL_TCB
python3 vision_grasp_demo.py
```

### 按键

| 键 | 功能 |
|----|------|
| `t` | 测试坐标变换（TCP 移到检测到的物体上方 100mm，不抓） |
| `g` | 完整抓取：接近 → 下降 → 闭合 → 抬起 → 回零 |
| `z` | 回零位 |
| `o` | 灵巧手张开 |
| `c` | 灵巧手闭合 |
| `q` | 退出 |

### 操作流程

1. 先按 `t` 测试——观察法兰盘中心是否停在物体正上方
2. 确认偏差在可接受范围后（±5cm 正常），再按 `g` 抓取
3. 抓取时：接近到物体上方 100mm → 降到物体上方 30mm → 手闭合 → 抬起 → 回零

### 当前手眼矩阵

```
方法: daniilidis / intrinsic_xyz (ARM Jetson 标定)
样本: 23 组
平移 RMS: 9.87mm | 旋转 RMS: 0.91°
运动跨度: 平移 513mm | 旋转 149°
```

> 如需更新矩阵：重新采集后用 `compute_in_hand_arm.py` 求解，将结果填入 `vision_grasp_demo.py` 第 54 行附近的 `handeye_matrix` 参数。

---

## 三、arm_hand_demo.py（无摄像头控制）

```bash
cd ~/Desktop/TL_TCB
python3 arm_hand_demo.py
```

| 键 | 功能 |
|----|------|
| `z` | 回零 |
| `1-4` | 预设关节位姿 |
| `o` | 手张开 |
| `c` | 手闭合 |
| `p` | 捏合 |
| `f` | 握拳 |
| `5` | 自动序列演示 |
| `q` | 退出 |

---

## 四、test_hand.py（灵巧手独立测试）

不启动机械臂运动，仅测试灵巧手通信和控制：

```bash
cd ~/Desktop/TL_TCB
python3 test_hand.py
```

| 键 | 功能 |
|----|------|
| `0` | 张开 |
| `1` | 握拳 |
| `i` | 初始化（首次上电必须执行，手会咔咔响） |
| `r` | 软复位 |
| `q` | 退出 |

---

## 五、手眼标定（如需更新矩阵）

### 采集数据

驱动运行中，开另一个终端：

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
cd ~/Desktop/TL_TCB
python3 collect_data_arm.py
```

操作：
1. 棋盘格固定在工作台上不动
2. 按住示教器 **P 键**拖动机械臂到不同姿态（D435 拍摄完整棋盘格）
3. 松开 P 键静止，按 **s** 保存
4. 至少采集 **15 组**——覆盖图像中心和四角、不同距离、绕不同轴的旋转
5. 按 **q** 退出

数据保存在 `~/eye_hand_data/<时间戳>/`。

### 求解

```bash
cd ~/Desktop/TL_TCB
python3 compute_in_hand_arm.py ~/eye_hand_data/<时间戳>
```

输出 4×4 手眼矩阵，填入 `vision_grasp_demo.py` 中。

---

## 关键注意事项

1. **`/joint_states` 单位是角度（度）**——程序直接使用，不做 rad/deg 转换。这是天链驱动的实际行为。
2. **J6 关节**——如已修复，零位 `[0,0,0,0,0,0,0]` 包含 J6。`arm_hand_demo.py` 的 7 元素零位包含 J6 的值。
3. **灵巧手首次上电必须初始化**——`test_hand.py` 按 `i`，或 `vision_grasp_demo.py` 自动执行 `start_init()`（手会咔咔响，正常）。
4. **手眼矩阵平台绑定**——x86 VM 标定的矩阵不能用于 Jetson ARM，必须用 Jetson 端数据重新标定。
5. **旋转约定**——ROS2 `/tcp_pose` 的 rpy 使用 intrinsic XYZ（Rx→Ry→Rz）。所有坐标变换已按此约定实现。
6. **ServoJ 延迟激活**——程序不会在启动时自动进入 ServoJ 模式，首次按键（`t`/`g`/`z`）时才激活并初始化到当前关节位置，防止启动时机械臂乱动。

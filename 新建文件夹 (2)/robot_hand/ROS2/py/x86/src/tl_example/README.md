<div align="center">

# tl_example — 示例脚本功能包

</div>

## 目录
* 1.[tl_example 功能包说明](#1-tl_example-功能包说明)
* 2.[现有脚本列表](#2-现有脚本列表)
  * 2.1.[Servoj 轨迹回放](#21-servoj-轨迹回放)
  * 2.2.[队列轨迹回放](#22-队列轨迹回放)
* 3.[功能包使用](#3-功能包使用)
  * 3.1.[功能包编译](#31-功能包编译)
  * 3.2.[配置文件说明](#32-配置文件说明)
* 4.[功能包架构说明](#4-功能包架构说明)
  * 4.1.[功能包文件总览](#41-功能包文件总览)
  * 4.2.[全局可调参数](#42-全局可调参数)

---

## 1. tl_example 功能包说明

`tl_example` 是天链机械臂 ROS2 工作空间中的**示例脚本功能包**，用于存放供客户参考的 ROS2 使用示例。

目前包含 **2 个轨迹回放脚本**，分别演示了两种不同的运动控制模式。后续会根据需求持续增加更多示例。

## 2. 现有脚本列表

### 2.1. Servoj 轨迹回放

**节点名**：`servoj_trajectory_playback`

通过打开关节跟踪模式（7000 端口辅助通道），将 `saved_points.json` 中的笛卡尔轨迹点转换为关节角度后，以 servoj 模式逐点实时发送给机械臂执行。

**原理**：逐点通过 `/tl_driver/set_servoj_pos` 话题发布关节角度，实时跟踪，频率高，适合连续平滑轨迹。

**执行流程**：
```
设置运行速度 → MoveJ 回到初始位 → 打开 servoj 模式
→ 逐点：坐标转换 → 发送关节角度
→ 关闭 servoj 模式 → MoveJ 回到初始位
```

### 2.2. 队列轨迹回放

**节点名**：`queue_trajectory_playback`

通过打开队列运动模式，将 `saved_points.json` 中的笛卡尔轨迹点转换为关节角度后，以队列模式逐条插入 MoveJ 指令执行。

**原理**：通过 `/tl_driver/queue_motion_movej` 服务逐条插入 MoveCommand，由控制器内部规划并平滑执行整段轨迹。

**执行流程**：
```
设置运行速度 → MoveJ 回到初始位 → 打开队列模式
→ 逐点：坐标转换 → 插入队列 MoveJ（首条 is_continue=false，后续 true）
→ 等待队列运动完成 → 关闭队列模式 → 重新上电 → MoveJ 回到初始位
```

> ⚠️ **注意**：关闭队列模式时机械臂会自动下电，脚本会自动重新上电并回到零位。

两种脚本共享同一份 `saved_points.json` 配置文件，在终端菜单中可选择要执行的轨迹组。

## 3. 功能包使用

### 3.1. 功能包编译

在编译 `tl_example` 前，需要先完成 `tl_ros2_interface` 和 `tl_driver` 功能包的编译。

```bash
cd ~/tl_robot_ros2_py

# 先编译接口包
colcon build --packages-select tl_ros2_interface
source install/setup.bash

# 再编译其他包（包含 tl_example）
colcon build
source install/setup.bash
```

单独编译 `tl_example`：

```bash
colcon build --packages-select tl_example
source install/setup.bash
```

**Servoj 轨迹回放：**

1. 首先启动 `tl_driver` 节点并连接机械臂：
   ```bash
   ros2 launch tl_driver tl_driver.launch.py arm_type:=<arm_type>
   ```

2. 启动 Servoj 轨迹回放节点：
   ```bash
   ros2 run tl_example servoj_trajectory_playback
   ```
   或：
   ```bash
   ros2 launch tl_example servoj_trajectory_playback.launch.py
   ```

3. 在终端菜单中选择要执行的轨迹组。

**队列轨迹回放：**

1. 首先启动 `tl_driver` 节点并连接机械臂：
   ```bash
   ros2 launch tl_driver tl_driver.launch.py arm_type:=<arm_type>
   ```

2. 启动队列轨迹回放节点：
   ```bash
   ros2 run tl_example queue_trajectory_playback
   ```
   或：
   ```bash
   ros2 launch tl_example queue_trajectory_playback.launch.py
   ```

3. 在终端菜单中选择要执行的轨迹组。

### 3.2. 配置文件说明

`saved_points.json` 保存在 `config/` 目录下，格式如下：

```json
{
  "轨迹组名称1": [
    [x1, y1, z1],
    [x2, y2, z2],
    ...
  ],
  "轨迹组名称2": [
    [x1, y1, z1],
    [x2, y2, z2],
    ...
  ]
}
```

每个数组元素是一个笛卡尔坐标点 `[x, y, z]`（单位：mm），脚本会使用固定的末端姿态（ROLL、PITCH、YAW）拼合成完整 7 维位姿，再通过 `coord_transform` 服务转换为关节角度。

## 4. 功能包架构说明

### 4.1. 功能包文件总览

```
tl_example/
├── config/                                # 配置文件
│   └── saved_points.json                  # 保存的笛卡尔轨迹点（JSON 格式）
├── launch/                                # 启动文件
│   ├── servoj_trajectory_playback.launch.py   # Servoj 模式启动文件
│   └── queue_trajectory_playback.launch.py    # 队列模式启动文件
├── tl_example/                            # Python 源代码
│   ├── __init__.py                        # 包标记
│   ├── servoj_trajectory_playback.py      # Servoj 轨迹回放主脚本
│   └── queue_trajectory_playback.py       # 队列轨迹回放主脚本
├── doc/                                   # 说明文档（预留）
├── package.xml                            # 依赖说明文件
├── README.md                              # 本说明文档
├── resource/                              # 资源标记文件
│   └── tl_example
├── setup.cfg                              # Python 包配置文件
├── setup.py                               # Python 编译规则文件
└── test/                                  # 测试脚本
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

### 4.2. 全局可调参数

两个脚本均在文件顶部集中定义了可调参数，方便客户根据实际需求快速调整：

**servoj_trajectory_playback.py 可调参数：**

| 参数 | 默认值 | 说明 |
|:---|:---:|:---|
| `ZERO_JOINT` | `[-34.381, 5.992, ..., -62.214]` | 初始/归零关节角度（度） |
| `ROLL` / `PITCH` / `YAW` | `3.13` / `-0.027` / `-0.485` | 固定末端姿态（弧度） |
| `ARM_ANGLE` | `0.0` | 臂角 |
| `MOVE_SPEED` | `20.0` | 运行速度（%） |
| `PUBLISH_INTERVAL` | `0.1` | 相邻关节角度发布间隔（秒） |
| `OPEN_SERVOJ_VMAX` | `[10.0] * 7` | Servoj 最大速度 |
| `OPEN_SERVOJ_AMAX` | `[3000.0] * 7` | Servoj 最大加速度 |
| `OPEN_SERVOJ_JMAX` | `[50000.0] * 7` | Servoj 最大加加速度 |

**queue_trajectory_playback.py 可调参数：**

| 参数 | 默认值 | 说明 |
|:---|:---:|:---|
| `ZERO_JOINT` | `[-34.381, 5.992, ..., -62.214]` | 初始/归零关节角度（度） |
| `ROLL` / `PITCH` / `YAW` | `3.13` / `-0.027` / `-0.485` | 固定末端姿态（弧度） |
| `ARM_ANGLE` | `0.0` | 臂角 |
| `MOVE_SPEED` | `20.0` | 全局运行速度（%，同时影响 set_speed 和 MoveCommand） |
| `QUEUE_ACC` | `20.0` | 队列运动加速度 |
| `QUEUE_DEC` | `20.0` | 队列运动减速度 |
| `QUEUE_WAIT_TIME` | `3.0` | 插入完成后等待队列运动完成的延时（秒） |

---

## 附录

### 依赖关系

- `rclpy` — ROS2 Python 客户端库
- `std_msgs` — 标准消息类型
- `sensor_msgs` — 传感器消息类型（`JointState`）
- `tl_ros2_interface` — 天链自定义消息与服务
- `ament_index_python` — 查找功能包共享目录

### 相关话题与服务

| 名称 | 类型 | 用途 |
|:---|:---|:---|
| `/tl_driver/set_servoj_pos` | `Float64MultiArray` | 话题-发送 Servoj 跟踪关节位置 |
| `/tl_driver/moveJ` | `MoveCommand` | 话题-发布 MoveJ 运动指令（回零） |
| `/joint_states` | `JointState` | 话题-订阅关节状态（确认到位） |
| `/tl_driver/set_speed` | `SetSpeed` | 服务-设置运行速度 |
| `/tl_driver/coord_transform` | `CoordTransform` | 服务-坐标转换（笛卡尔→关节） |
| `/tl_driver/open_servoj` | `OpenServoJ` | 服务-打开关节跟踪模式（Servoj 模式用） |
| `/tl_driver/close_servoj` | `Trigger` | 服务-关闭关节跟踪模式（Servoj 模式用） |
| `/tl_driver/queue_motion_set_status` | `QueueMotionSetStatus` | 服务-打开/关闭队列模式（队列模式用） |
| `/tl_driver/queue_motion_movej` | `QueueMotionMoveJ` | 服务-插入队列 MoveJ 运动（队列模式用） |
| `/tl_driver/connect_arm` | `Trigger` | 服务-连接机械臂 |
| `/tl_driver/power_on` | `Trigger` | 服务-上电 |

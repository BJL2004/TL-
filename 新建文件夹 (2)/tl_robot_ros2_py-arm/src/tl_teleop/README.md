<div align="center">

# 天链机器人 tl_teleop 使用说明书

</div>

## 目录
* 1.[tl_teleop 功能包说明](#1-tl_teleop-功能包说明)
* 2.[tl_teleop 功能包使用](#2-tl_teleop-功能包使用)
* 3.[tl_teleop 功能包架构说明](#3-tl_teleop-功能包架构说明)
* 4.[tl_teleop 功能包话题与服务说明](#4-tl_teleop-功能包话题与服务说明)

## 1 tl_teleop 功能包说明
tl_teleop 功能包实现了机械臂的遥操作（Teleoperation）功能，通过 PXREA Robot SDK 与 VR 遥操作手柄设备通信，将手柄的位姿映射为机械臂末端的运动指令，实现对机械臂的实时跟随控制。

通过以下三部分内容的介绍可以帮助大家：
* 1.了解该功能包的使用。
* 2.熟悉功能包中的文件构成及作用。
* 3.熟悉功能包相关的话题，方便开发和使用。

### 1.1 功能特性
- **VR 手柄遥操作**：通过 `xrobotoolkit_sdk`（PXREA Robot SDK 的 Python 绑定）读取 VR 手柄位姿（位置 + 四元数姿态），实时驱动机械臂末端运动
- **握紧触发机制**：只有握紧 VR 手柄扳机（握力 > 0.9）时才触发遥操作，松开即停止，握下瞬间自动记录当前机械臂 TCP 位姿为基准
- **姿态最短路径**：通过四元数插值计算 VR 手柄与机械臂之间的姿态差值，自动选择最短旋转路径，支持万向锁安全处理
- **位置死区滤波**：手柄微小位移（默认 < 5mm）被过滤，避免手部抖动导致机械臂振荡
- **单步增量限制**：每次位置增量上限（默认 300mm），防止异常跳变
- **奇异点保护**：6 轴模式检测 J5/J6、7 轴模式检测 J6/J7，角度超过 160° 时自动减速至 20%，防止关节速度爆炸
- **关节跳变检测**：相邻两次指令的关节角度差超过 30° 时拒绝执行，防止异常指令或通信错误
- **硬限位裁剪**：所有关节目标在发送前被裁剪到配置的硬限位范围内
- **A 键复位**：按下 VR 手柄 A 键将所有关节复位到零点
- **100Hz 控制循环**：主控制循环以 100Hz 频率运行，通过 tl_driver 的 ServoJ 模式实现高速位置跟随
- **RViz 可视化**：通过 `/joint_states` 话题发布目标关节状态，可在 RViz 中实时显示遥操作指令轨迹
- **6/7 轴自适应**：通过 `arm_axis_mode` 参数切换轴数，自动适配不同型号机械臂的关节限位和奇异点检测逻辑

### 1.2 系统依赖关系

tl_teleop 运行时依赖 **tl_driver** 功能包提供以下服务与话题：

| 依赖项 | 类型 | 说明 |
|--------|------|------|
| `/tl_driver/set_current_mode` | 服务 | 切换到运行模式（模式 2） |
| `/tl_driver/set_speed` | 服务 | 设置运动速度 |
| `/tl_driver/open_servoj` | 服务 | 开启 ServoJ 模式（含动力学参数 vmax/amax/jmax） |
| `/tl_driver/close_servoj` | 服务 | 关闭 ServoJ 模式 |
| `/tl_driver/coord_transform` | 服务 | 逆运动学求解（笛卡尔 → 关节） |
| `/tl_driver/get_rpy2quat` | 服务 | RPY → 四元数转换 |
| `/tcp_pose` | 话题 | 获取当前末端 TCP 位姿 |
| `/tl_driver/set_servoj_pos` | 话题 | 发送目标关节位置 |

> **注意**：tl_teleop 不直接调用上电/下电服务——上下电和连接管理由 tl_driver 负责。使用 tl_teleop 前请确保 tl_driver 已正常启动并完成上电。

因此使用 tl_teleop 前应先启动 tl_driver。

### 1.3 外部环境依赖

tl_teleop 的 PXREA Robot SDK 依赖 **XRobotToolKit-PC_Service** 作为 PC 端后台服务，用于与 VR 遥操作设备建立连接并传输手柄数据。使用 tl_teleop 前必须安装并启动该服务。

**下载地址**：[XRoboToolkit-PC-Service v1.0.0](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/tag/v1.0.0)

根据平台架构选择对应的安装包：

| 架构 | 安装包 |
|------|--------|
| ARM64 | `XRoboToolkit-PC-Service-headless_1.0.0.0_arm64.deb` |
| x86_64 | `XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb` |

安装并启动 XRobotToolKit-PC_Service 后，再启动 tl_teleop 节点，PXREA SDK 才能正常连接 VR 设备并接收手柄位姿数据。

### 1.4 xrobotoolkit_sdk 安装

`xrobotoolkit_sdk` 是 XRoboToolkit PC Service SDK 的 Python 绑定库（基于 pybind11），为 tl_teleop 提供 VR 手柄数据读取接口。

> **自动安装**：`tl_teleop` 的 `setup.py` 已集成 `xrobotoolkit_sdk` 的构建安装逻辑——执行 `colcon build` 时会自动检测缺失的原生库和 Python 绑定，并根据平台自动运行 `setup_ubuntu.sh`（x86_64）或 `setup_orin.sh`（ARM64）完成全流程构建。若自动构建失败，请按下方步骤手动安装。

本仓库中已包含该 SDK 的 Git 子模块（`depend/XRoboToolkit-PC-Service-Pybind/`），克隆后需先初始化：

```bash
cd ~/tl_robot_ros2_py
git submodule update --init
```

根据平台架构选择对应的安装方式：

#### x86_64 平台

**方式一（不推荐，安装到系统环境）**：使用 `setup_ubuntu.sh` 自动下载并编译 XRoboToolkit-PC-Service 原生 SDK，然后安装 Python 绑定。

```bash
cd src/tl_teleop/depend/XRoboToolkit-PC-Service-Pybind
bash setup_ubuntu.sh
```

> ⚠️ 方式一会将 `xrobotoolkit_sdk` 安装到系统 Python 路径（`/usr/local/lib/`），`pip list` 中可见，但脱离 colcon 的隔离环境，可能与工作空间其他包不协调。

**方式二（推荐，安装到 ROS2 install 目录）**：如果已通过 deb 包安装 XRoboToolkit-PC-Service（参见上方「外部环境依赖」），只需将原生库复制到 SDK 目录，随后 `colcon build` 会自动完成 Python 绑定的编译与安装：

```bash
# 将原生库复制到 SDK 的 lib 目录
cp /opt/apps/roboticsservice/lib/libPXREARobotSDK.so src/tl_teleop/depend/XRoboToolkit-PC-Service-Pybind/lib/

# colcon build 会自动检测并安装 xrobotoolkit_sdk
cd ~/tl_robot_ros2_py
colcon build
```

#### ARM64 平台（Nvidia Orin）

**方式一（不推荐，安装到系统环境）**：使用 `setup_orin.sh` 自动下载并编译 XRoboToolkit-PC-Service 原生 SDK（orin 分支），然后安装 Python 绑定。

```bash
cd src/tl_teleop/depend/XRoboToolkit-PC-Service-Pybind
bash setup_orin.sh
```

> ⚠️ 方式一会将 `xrobotoolkit_sdk` 安装到系统 Python 路径。

**方式二（推荐，安装到 ROS2 install 目录）**：如果已通过 deb 包安装 XRoboToolkit-PC-Service（参见上方「外部环境依赖」），只需将原生库复制到 SDK 目录，随后 `colcon build` 会自动完成 Python 绑定的编译与安装：

```bash
# 将原生库复制到 SDK 的 lib/aarch64 目录
cp /path/to/libPXREARobotSDK.so src/tl_teleop/depend/XRoboToolkit-PC-Service-Pybind/lib/aarch64/

# colcon build 会自动检测并安装 xrobotoolkit_sdk
cd ~/tl_robot_ros2_py
colcon build
```

> 编译依赖：`pybind11`、`cmake`、C++ 编译工具链。

安装完成后可通过以下命令验证：

```bash
python3 -c "import xrobotoolkit_sdk; print('xrobotoolkit_sdk 安装成功')"
```

## 2 tl_teleop 功能包使用

### 2.1 编译

```bash
cd ~/tl_robot_ros2_py
colcon build
source install/setup.bash
```

> `tl_teleop` 的 `setup.py` 会在 `colcon build` 时自动检测并安装 `xrobotoolkit_sdk`（需先执行 `git submodule update --init` 且原生库已就绪）。若自动安装失败，参见上方「xrobotoolkit_sdk 安装」章节手动安装。

### 2.2 启动方式

tl_teleop 依赖 XRobotToolKit-PC_Service 和 tl_driver，启动前需**按顺序**依次启动以下服务与节点：

**第一步：启动 XRobotToolKit-PC_Service**

```bash
source /opt/apps/roboticsservice/runService.sh
```

该命令启动 PC 端后台服务，PXREA SDK 通过该服务与 VR 遥操作设备通信。

> 注意：不同平台的 XRobotToolKit-PC_Service 启动命令可能不同，具体查阅 XRobotToolKit-PC_Service 的相关说明。

**第二步：启动 tl_driver（以 TCB710 为例）**

```bash
ros2 launch tl_driver tl_tcb710_driver.launch.py
```

> 其他臂型的启动命令参见 [tl_driver 说明文档](../tl_driver/README.md)。

**第三步：启动 tl_teleop**

6 轴机械臂（TCB605 / TCB610 系列）：
```bash
ros2 launch tl_teleop tl_teleop_6axis.launch.py
```

7 轴机械臂（TCB705 / TCB710 系列）：
```bash
ros2 launch tl_teleop tl_teleop_7axis.launch.py
```

启动成功后，节点将依次执行：
1. 初始化 PXREA Robot SDK（`xrt.init()`），连接 VR 遥操作设备
2. 启动 VR 数据读取线程（100Hz）
3. 等待 tl_driver 所有必要服务就绪
4. 调用 tl_driver 服务：设置运行模式 → 设置速度 → 开启 ServoJ
5. 进入遥操作控制循环

### 2.3 配置参数说明

配置文件位于 `config/` 下，按轴数分为两套：

- **6 轴**：`tl_teleop_6axis.yaml`（TCB605 / TCB610 系列），位于 `config/tl_teleop_6axis.yaml`
- **7 轴**：`tl_teleop_7axis.yaml`（TCB705 / TCB710 系列），位于 `config/tl_teleop_7axis.yaml`

以 7 轴配置为例：

```yaml
tl_teleop_node:
  ros__parameters:
    # ========== 轴数配置（必须为 6 或 7，运行时校验）==========
    arm_axis_mode: 7

    # ========== 位置控制参数 ==========
    pos_scale: 0.5               # 位置缩放系数（VR位移 1m → 末端移动 0.5m）
    pos_deadzone: 0.005           # 位置死区（m），小于此值的位移被忽略
    max_pos_delta_mm: 300.0       # 单步最大位置增量（mm），防止异常跳变

    # ========== 日志 ==========
    log_interval: 1.0             # 关节日志打印间隔（秒）

    # ========== 奇异点防护参数 ==========
    joint_jump_threshold: 30.0    # 关节跳变阈值（度），超过此值拒绝执行
    singular_angle: 160.0         # 奇异点判定角度（度），超过此角自动减速
    singular_scale: 0.2           # 奇异点区域速度缩放系数
    # 7轴关节硬限位（度）：J1~J5 ±180°，J6~J7 ±170°
    # 扁平数组，每两个数一组：[min1, max1, min2, max2, ...]
    joint_limits: [-180.0, 180.0, -180.0, 180.0, -180.0, 180.0, -180.0, 180.0, -180.0, 180.0, -170.0, 170.0, -170.0, 170.0]

    # ========== ServoJ 初始化参数 ==========
    servo_speed: 25.0             # ServoJ 运动速度
    servo_vmax: 80.0              # ServoJ 最大速度（度/秒）
    servo_amax: 3000.0            # ServoJ 最大加速度（度/秒²）
    servo_jmax: 50000.0           # ServoJ 最大加加速度（度/秒³）
```

参数详解：

| 参数 | 类型 | 说明 |
|------|------|------|
| `arm_axis_mode` | int | **必填**。机械臂轴数（6 或 7），必须与 YAML 文件名对应轴的配置匹配。运行时强制校验，不匹配则拒绝启动 |
| `pos_scale` | double | 手柄位移到机械臂末端移动的缩放系数。手柄 1m 位 → 末端移动 `pos_scale` 米 |
| `pos_deadzone` | double | 位置死区（m），手柄微小抖动被过滤，避免手部抖动导致机械臂振荡 |
| `max_pos_delta_mm` | double | 单步位置增量上限（mm），防止异常跳变导致机械臂失控 |
| `log_interval` | double | 关节角度日志打印间隔（秒），设为 0 则不打印关节日志 |
| `joint_jump_threshold` | double | 关节跳变阈值（度），相邻两次指令的关节角差超过此值则丢弃 |
| `singular_angle` | double | 奇异点判定角度（度）。6轴检测 J5/J6，7轴检测 J6/J7。超过此角自动减速 |
| `singular_scale` | double | 奇异点区域的速度缩放系数（越小越安全，但运动越慢） |
| `joint_limits` | double[] | **必填**。关节硬限位（度），扁平数组格式，长度必须 = `arm_axis_mode * 2`。超出范围的目标值被裁剪 |
| `servo_speed` | double | ServoJ 模式运动速度（百分比） |
| `servo_vmax` | double | ServoJ 各轴最大速度（度/秒），同时应用于全部 7 个关节 |
| `servo_amax` | double | ServoJ 各轴最大加速度（度/秒²），同时应用于全部 7 个关节 |
| `servo_jmax` | double | ServoJ 各轴最大加加速度（度/秒³），同时应用于全部 7 个关节 |

> **注意**：tl_teleop 不再包含 `arm_ip`、`arm_port`、`arm_port_aux` 等控制器通信参数——这些由 tl_driver 管理。tl_teleop 通过 ROS2 话题和服务接口与 tl_driver 通信，不直接连接控制器。

## 3 tl_teleop 功能包架构说明

### 3.1 功能包文件总览

```
tl_teleop/
├── package.xml                        # ROS2 包元信息（build_type: ament_python）
├── setup.py                           # Python 包构建脚本（含自动安装 xrobotoolkit_sdk 的自定义 build_py）
├── setup.cfg                          # ament_python 标准配置文件
├── resource/tl_teleop                 # ament 包索引标记文件
├── config/                            # 参数配置文件
│   ├── tl_teleop_6axis.yaml           # 6 轴机械臂遥操作参数（TCB605 / TCB610 系列）
│   └── tl_teleop_7axis.yaml           # 7 轴机械臂遥操作参数（TCB705 / TCB710 系列）
├── launch/                            # ROS2 启动文件
│   ├── tl_teleop_6axis.launch.py      # 6 轴快捷启动（加载 tl_teleop_6axis.yaml 配置）
│   └── tl_teleop_7axis.launch.py      # 7 轴快捷启动（加载 tl_teleop_7axis.yaml 配置）
├── tl_teleop/                         # Python 包源码目录
│   ├── __init__.py                    # 包标记文件（空文件）
│   └── tl_teleop.py                   # 遥操作主节点实现（ArmTeleopNode 类 + main 入口）
├── depend/                            # 第三方 SDK Git 子模块
│   └── XRoboToolkit-PC-Service-Pybind/
│       ├── lib/libPXREARobotSDK.so    # x86_64 预编译原生库
│       ├── lib/aarch64/               # ARM64 预编译原生库
│       ├── bindings/py_bindings.cpp   # pybind11 C++ 绑定源码（colcon build 时自动编译）
│       ├── setup.py                   # xrobotoolkit_sdk 安装脚本
│       ├── setup_ubuntu.sh            # x86_64 自动构建脚本
│       └── setup_orin.sh              # ARM64 (Orin) 自动构建脚本
├── test/                              # ament 代码质量测试
│   ├── test_copyright.py              # 版权声明检查
│   ├── test_flake8.py                 # 代码风格检查 (PEP 8)
│   └── test_pep257.py                 # 文档字符串检查 (PEP 257)
└── README.md                          # 本说明文档
```

### 3.2 代码架构说明

tl_teleop 基于 `rclpy.node.Node`（类 `ArmTeleopNode`）实现，采用 **Python 多线程架构**：

| 线程 | 频率 | 职责 |
|------|------|------|
| **VR 读取线程**（`device_read_thread`） | ~100Hz | 循环调用 `xrobotoolkit_sdk` 读取右手柄位姿（位置 + 四元数）和握力数据，写入模块级全局状态缓存（`pose_data` / `grip_data`），通过 `threading.Lock` 保护 |
| **ROS2 Spin 线程**（`rclpy.spin`） | 事件驱动 | 处理 `/tcp_pose` 话题订阅回调（`_tcp_pose_cb` 缓存末端位姿）、服务客户端异步响应 |
| **主控制循环**（`main_control_loop`） | 100Hz | 读取 VR 位姿缓存 → 计算笛卡尔偏差 → 调用 IK 服务 → 安全校验 → 发布关节目标到 `/tl_driver/set_servoj_pos` |
| **定时器回调**（`_publish_joints`） | 20Hz | 将最新目标关节角度（度→弧度）发布到 `/joint_states`，供 RViz 中实时显示遥操作指令轨迹 |

**核心类 `ArmTeleopNode` 主要成员：**

| 类别 | 成员 | 说明 |
|------|------|------|
| 服务客户端 | `set_current_mode_client` 等 6 个 | 调用 tl_driver 提供的模式切换、速度设置、ServoJ 启停、逆运动学、位姿转换服务 |
| 话题发布 | `servoj_pos_pub` | 发布 `std_msgs/Float64MultiArray` 关节目标（度）到 `/tl_driver/set_servoj_pos` |
| 话题发布 | `joint_pub` | 发布 `sensor_msgs/JointState` 到 `/joint_states`，用于 RViz 可视化 |
| 话题订阅 | `tcp_pose_sub` | 订阅 `tl_ros2_interface/CartesianPose`，缓存当前末端 TCP 位姿 |
| TCP 缓存 | `_tcp_position` / `_tcp_rpy` | 线程安全（`_tcp_lock`）缓存最新 TCP 位置和 RPY 欧拉角 |
| 状态变量 | `last_joints` | 上一次成功下发的关节角，用于跳变检测和奇异点判定 |
| 状态变量 | `_latest_target_joints` | 最新目标关节角（供可视化定时器发布到 `/joint_states`） |
| 初始化序列 | `wait_for_services()` | 阻塞等待 6 个必要服务就绪（超时 10s） |
| 初始化序列 | `init_servoj()` | 依次调用 set_current_mode（模式 2）→ set_speed → open_servoj |
| 运动解算 | `call_inverse_kinematics()` | 通过 `/tl_driver/coord_transform` 做笛卡尔坐标 → 关节角逆解 |
| 姿态转换 | `call_get_rpy2quat()` | 通过 `/tl_driver/get_rpy2quat` 做 RPY 欧拉角 → 四元数转换 |
| 安全下发 | `servoJ_send()` | 硬限位裁剪 → 跳变检测 → 补零至 7 维（6 轴模式）→ 话题发布 |
| 限位裁剪 | `clamp_joints()` | 将关节目标裁剪到 `joint_limits` 范围 |

**模块级函数（独立于类的纯 Python 本地计算）：**

| 函数 | 说明 |
|------|------|
| `device_read_thread()` | VR 手柄数据读取线程主函数，循环调用 xrobotoolkit_sdk |
| `main_control_loop(node)` | 遥操作主控制循环（100Hz），单次迭代含握紧检测、偏差计算、IK、安全下发 |
| `quat2rpy(q)` | 四元数 → RPY 欧拉角（w,x,y,z 顺序） |
| `quat_multiply(q1, q2)` | 四元数乘法 q1 × q2 |
| `quat_inverse(q)` | 四元数共轭（逆，假设输入为单位四元数） |
| `clamp_joints(joints, limits)` | 关节硬限位裁剪（独立函数版本） |

**控制流程：**

```
VR手柄位姿(pose_data) → 握紧触发(grip>0.9)
    → 记录基准TCP位姿(位置mm + RPY rad) → 记录基准VR位姿(位置 + 四元数)
    → 计算VR偏差(位置Δ + 姿态Δ) → 坐标映射(VR→机器人坐标系)
    → 位置缩放 × 奇异点减速 × 单步增量限幅(300mm)
    → 调用 /tl_driver/coord_transform 逆运动学求解
    → 关节跳变检测(>30°) → 硬限位裁剪(clamp_joints) → 补零至7维(6轴模式)
    → 发布到 /tl_driver/set_servoj_pos → tl_driver 执行伺服控制
    → 同步发布 /joint_states 用于 RViz 可视化
A键触发 → 下发全零关节角 → 清除基准 → 1秒冷却
```

## 4 tl_teleop 功能包话题与服务说明

tl_teleop 作为客户端节点，通过 tl_driver 提供的话题与服务接口间接控制机械臂，tl_teleop 本身不提供服务端。

### 4.1 发布的话题

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/tl_driver/set_servoj_pos` | `std_msgs/msg/Float64MultiArray` | ~100Hz | 目标关节角度数组（单位：度），7 维固定长度（6 轴模式第 7 维补 0） |
| `/joint_states` | `sensor_msgs/msg/JointState` | 20Hz | 最新目标关节状态（弧度），用于 RViz 中实时显示遥操作指令轨迹 |

### 4.2 订阅的话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/tcp_pose` | `tl_ros2_interface/msg/CartesianPose` | 获取机械臂当前末端 TCP 位姿。position（x/y/z）单位 mm，rpy（x/y/z）单位 rad。用于握紧时记录基准位姿 |

### 4.3 调用的服务（客户端）

| 服务 | 类型 | 调用时机 | 说明 |
|------|------|------|------|
| `/tl_driver/set_current_mode` | `tl_ros2_interface/srv/SetCurrentMode` | 启动初始化 | 切换到运行模式（mode=2） |
| `/tl_driver/set_speed` | `tl_ros2_interface/srv/SetSpeed` | 启动初始化 | 设置 ServoJ 运动速度 |
| `/tl_driver/open_servoj` | `tl_ros2_interface/srv/OpenServoJ` | 启动初始化 | 开启 ServoJ 模式，传入 vmax/amax/jmax 动力学限制 |
| `/tl_driver/close_servoj` | `std_srvs/srv/Trigger` | 安全退出 | 关闭 ServoJ 模式 |
| `/tl_driver/coord_transform` | `tl_ros2_interface/srv/CoordTransform` | 控制循环（每次 IK） | 笛卡尔坐标（x/y/z mm + rx/ry/rz rad）→ 关节角度（度）逆解 |
| `/tl_driver/get_rpy2quat` | `tl_ros2_interface/srv/GetPosTransform` | 握下扳机时 | RPY 欧拉角 → 四元数（w/x/y/z）转换，用于姿态基准标定 |

以上 6 个服务均为调用 tl_driver 提供的接口。**tl_teleop 不调用 `/tl_driver/power_on`、`/tl_driver/power_off`、`/tl_driver/clear_error`**——上下电和错误清除由 tl_driver 或其启动文件负责。

### 4.4 控制逻辑说明

1. **启动初始化** → 等待所有 6 个必要 tl_driver 服务就绪 → 依次调用 `set_current_mode`（模式 2）→ `set_speed` → `open_servoj`（附动力学参数 vmax/amax/jmax）
2. **遥操作循环**（100Hz）→ 读取 VR 手柄位姿 → 握下扳机（grip > 0.9）时以当前 TCP 位姿为基准 → 手柄偏移映射为笛卡尔位置增量（VR → 机器人坐标系）→ 四元数最短路径姿态插值 → 奇异点减速 → 单步增量限幅（300mm）→ 调用 `/tl_driver/coord_transform` 做逆运动学 → 关节跳变检测（30°）→ 硬限位裁剪 → 发布到 `/tl_driver/set_servoj_pos`
3. **A 键回零** → 控制循环中轮询 `xrt.get_A_button()`，按下时立即下发全零关节角并清除所有基准状态，冷却 1 秒后恢复控制
4. **安全退出** → 捕获 `KeyboardInterrupt` → 关闭 ServoJ → PXREA SDK 反初始化（`xrt.close()`）→ rclpy 关闭

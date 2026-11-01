<div align="center">

# 天链机械臂 ROS2 功能包使用说明书

四川天链机器人股份有限公司

文件修订记录：

| 版本号 | 时间 | 备注 |
| :---: | :---- | :--- |
| V1.0 | 2026-5-22 | 拟制 |

</div>

## 目录

* 1 [功能包概览](#1-功能包概览)
* 2 [功能包说明](#2-功能包说明)
  * 2.1 [tl_bringup — 启动聚合](#21-tl_bringup--启动聚合)
  * 2.2 [tl_description — 模型描述](#22-tl_description--模型描述)
  * 2.3 [tl_driver — 硬件驱动](#23-tl_driver--硬件驱动)
  * 2.4 [tl_example — 使用示例](#24-tl_example--使用示例)
  * 2.5 [tl_gazebo — Gazebo 仿真](#25-tl_gazebo--gazebo-仿真)
  * 2.6 [tl_hardware — ros2_control 硬件接口](#26-tl_hardware--ros2_control-硬件接口)
  * 2.7 [tl_moveit2_config — MoveIt2 配置](#27-tl_moveit2_config--moveit2-配置)
  * 2.8 [tl_ros2_interface — 消息与接口定义](#28-tl_ros2_interface--消息与接口定义)
  * 2.9 [tl_teleop — VR 手柄遥操作](#29-tl_teleop--vr-手柄遥操作)
  * 2.10 [tl_teleop_f710 — F710 手柄遥操作](#210-tl_teleop_f710--f710-手柄遥操作)

## 1 功能包概览

`src/` 目录包含 **10 个功能包目录**（其中 `tl_moveit2_config` 内含 14 个子包），每个功能包（及子包）的作用如下。

```
src/
├── tl_bringup/              # 启动聚合
│   ├── launch/              # 一键启动 launch 文件
│   └── doc/
├── tl_description/          # 模型描述
│   ├── config/              # 关节名称配置
│   ├── launch/              # 模型与 TF 发布 launch
│   ├── meshes/              # 14 套 STL 网格文件
│   ├── rviz/                # RViz 配置文件
│   └── urdf/                # 14 套 URDF 模型
├── tl_driver/               # 硬件驱动
│   ├── config/              # 14 套通信参数配置
│   ├── launch/              # 驱动节点 launch
│   ├── lib/                 # NexMotion SWIG 封装
│   └── tl_driver/           # 驱动节点源码
├── tl_example/              # 使用示例
│   ├── launch/
│   └── tl_example/
├── tl_gazebo/               # Gazebo 仿真
│   ├── config/
│   └── launch/              # 14 套 Gazebo 仿真 launch
├── tl_hardware/             # ros2_control 硬件接口（C++）
│   ├── include/
│   ├── src/
│   └── tl_hardware_interface.xml
├── tl_moveit2_config/       # MoveIt2 配置（内含 14 个子包）
│   ├── tl_tcb605_config/    # 14 套 MoveIt2 配置（每型号一套）
│   │   ├── config/          # SRDF、限位、运动学等
│   │   └── launch/          # demo、move_group、rviz
│   ├── tl_tcb605f_config/
│   ├── ...
│   └── tl_tcb710v_config/
├── tl_ros2_interface/       # ROS2 消息与服务接口
│   ├── msg/                 # 11 个 msg 定义文件
│   └── srv/                 # 45 个 srv 定义文件
├── tl_teleop/               # VR 手柄遥操作
│   ├── launch/
│   └── tl_teleop/
└── tl_teleop_f710/          # F710 手柄遥操作
    ├── launch/
    └── tl_teleop_f710/
```

## 2 功能包说明

### 2.1 tl_bringup — 启动聚合

一键启动机械臂驱动和模型发布的 launch 文件。通过组合 `tl_driver` 和 `tl_description` 的 launch 文件，实现快速启动。

详细说明请参考 [tl_bringup/README.md](tl_bringup/README.md)。

### 2.2 tl_description — 模型描述

提供各型号机械臂的 URDF 模型文件和网格文件（STL），并为其他功能包提供机械臂关节间的坐标变换关系。

- 支持 14 种型号的 URDF 模型
- 包含碰撞检测网格和可视化网格
- 提供 RViz 配置文件
- 支持通过 `robot_state_publisher` 发布 TF 变换

详细说明请参考 [tl_description/README.md](tl_description/README.md)。

### 2.3 tl_driver — 硬件驱动

通过 TCP/IP 协议与机械臂控制器通信，实现以下功能：

- 状态反馈：实时发布关节状态（位置、速度、力矩）、末端位姿以及机械臂整体运行状态，供上层模块订阅使用。
- 连接管理：提供与机械臂控制器的建立连接与断开连接功能。
- 电源控制：支持机械臂的上电与下电操作。
- 速度调节：支持设定与读取机械臂运动速度。
- 状态监控：支持查询机器人运行状态、关节温度、关节电压、电机电流、当前力矩、关节速度等实时状态参数。
- 模式切换：支持设定与读取当前控制模式。
- 坐标系配置：支持设置工具参数和用户坐标系。
- Modbus 通信：提供 Modbus 读写功能，用于与外部设备交互。
- 队列运动：支持将运动指令加入队列依次执行。

驱动基于 NexMotion SDK 的 Python 封装（SWIG），通过 TCP 连接控制器（默认 IP：`192.168.1.13`，端口：`6001`）。

详细说明请参考 [tl_driver/README.md](tl_driver/README.md)。

### 2.4 tl_example — 使用示例

提供 ROS2 接口调用示例，展示如何与 `tl_driver` 的各类服务和话题交互，帮助用户快速上手开发自己的应用。

详细说明请参考 [tl_example/README.md](tl_example/README.md)。

### 2.5 tl_gazebo — Gazebo 仿真

在 Gazebo 仿真环境中加载机械臂模型，并可通过 MoveIt2 对仿真的机械臂进行规划控制。

详细说明请参考 [tl_gazebo/README.md](tl_gazebo/README.md)。

### 2.6 tl_hardware — ros2_control 硬件接口

位于 `src/tl_hardware/`。该包是 ros2_control 的 `SystemInterface` 硬件接口插件（C++ 实现），作为 ros2_control 控制器层与 `tl_driver` 驱动节点之间的数据桥。

**作用**：
- 接收 `joint_trajectory_controller` 输出的位置指令（弧度）
- 转换为角度后发布到 `/tl_driver/set_servoj_pos` 话题
- 订阅 `tl_driver` 发布的 `/joint_states`，获取关节状态反馈
- 调用 `tl_driver` 的 `open_servoj` / `close_servoj` 服务管理伺服流模式

**数据链路**：MoveIt2 → ros2_control（joint_trajectory_controller）→ `tl_hardware`（话题/服务）→ `tl_driver`（TCP）→ 机械臂

**注意事项**：
- 该包是工作空间中**唯一的 C++ 包**，因为 ros2_control 的插件加载机制（`pluginlib`）要求硬件接口使用 C++ 实现
- 构建时必须**先构建 `tl_ros2_interface`**，否则链接失败

详细说明请参考 [tl_hardware/README.md](tl_hardware/README.md)。

### 2.7 tl_moveit2_config — MoveIt2 配置

为各系列机械臂提供运动规划控制功能，包括虚拟和真实机械臂控制两部分。

- 运动学求解器：KDL
- 包含 14 套独立配置（每个型号一套）
- 提供关节限位、初始位姿、控制器配置等
- 支持 RViz 可视化规划

详细说明请参考 [tl_moveit2_config/README.md](tl_moveit2_config/README.md)。

### 2.8 tl_ros2_interface — 消息与接口定义

为 TL 系列机械臂在 ROS2 框架下提供消息（msg）和服务（srv）接口定义，供上层驱动或应用调用。

- 消息（msg）：定义了机械臂状态、末端位姿、运动指令、Modbus 参数模版、DH 参数、关节参数、工具参数等数据格式，为话题通信提供统一的数据结构。
- 服务（srv）：定义了坐标变换、作业文件管理、DH 参数读写、关节参数查询与设置、位姿转换、Modbus 读写、队列运动等功能的请求/响应格式。

该包不包含可执行代码，仅提供接口定义（.msg 和 .srv 文件），其他功能包通过编译生成的头文件引用这些接口类型。

详细说明请参考 [tl_ros2_interface/README.md](tl_ros2_interface/README.md)。

### 2.9 tl_teleop — VR 手柄遥操作

通过 VR 手柄（需配合 xrobotoolkit_sdk 和 VR 设备）实现机械臂的远程遥操作控制，将手柄姿态实时映射为机械臂运动指令。

详细说明请参考 [tl_teleop/README.md](tl_teleop/README.md)。

### 2.10 tl_teleop_f710 — F710 手柄遥操作

通过罗技 F710 游戏手柄实现机械臂的遥操作控制，通过话题下发运动指令。

详细说明请参考 [tl_teleop_f710/README.md](tl_teleop_f710/README.md)。

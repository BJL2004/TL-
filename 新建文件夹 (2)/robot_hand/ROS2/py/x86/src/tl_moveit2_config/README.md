# 天链机器人 tl_moveit2_config 使用说明书

## 目录

- [1. tl_moveit2_config 说明](#1-tl_moveit2_config-说明)
- [2. tl_moveit2_config 使用](#2-tl_moveit2_config-使用)
  - [2.1 MoveIt2 控制虚拟机械臂](#21-moveit2-控制虚拟机械臂)
  - [2.2 MoveIt2 控制真实机械臂](#22-moveit2-控制真实机械臂)
- [3. tl_moveit2_config 架构说明](#3-tl_moveit2_config-架构说明)
  - [3.1 功能包文件总览](#31-功能包文件总览)
- [4. tl_moveit2_config 话题说明](#4-tl_moveit2_config-话题说明)

---

## 1. tl_moveit2_config 说明

`tl_moveit2_config` 功能包集为实现 MoveIt2 控制天链（TianLian）机械臂的功能包集合，其主要作用为调用官方的 MoveIt2 框架，结合 `tl_description` 提供的 URDF 模型，生成适配于天链各型号机械臂的 MoveIt2 配置和启动文件。通过该功能包集可以实现 MoveIt2 控制虚拟机械臂和控制真实机械臂。

目前支持的天链机械臂型号：tcb605、tcb605f、tcb605l、tcb605lv、tcb605v、tcb610、tcb610v、tcb705、tcb705f、tcb705l、tcb705lv、tcb705v、tcb710、tcb710v。

本文档从以下三方面介绍该功能包集：
1. **使用说明** — 启动虚拟/真实机械臂控制
2. **架构说明** — 配置文件结构和作用
3. **话题说明** — 节点间通信关系

---

## 2. tl_moveit2_config 使用

### 2.1 MoveIt2 控制虚拟机械臂

首先配置好环境完成连接后我们可以通过以下命令直接启动节点。

```bash
ros2 launch tl_<arm_type>_config demo.launch.py
```

在实际使用时需要将 `<arm_type>` 更换为实际的机械臂型号，可选的机械臂型号见第 1 节。

例如 tcb710 机械臂的启动命令：

```bash
ros2 launch tl_tcb710_config demo.launch.py
```

节点启动成功后，将显示以下画面。

![image](doc/image1.png)

接下来我们可以通过拖动控制球使机械臂到达目标位置，然后点击规划执行。

![image](doc/image2.png)

规划执行。

![image](doc/image3.png)


### 2.2 MoveIt2 控制真实机械臂

通过 `tl_hardware`（ros2_control 硬件接口插件）桥接 MoveIt2 与真实机械臂，利用 `use_real_hardware:=true` 参数切换硬件接口后端。

**前置条件**：
- 已完成工作空间编译（`colcon build`）
- 真实机械臂已上电且网络可达（默认 IP `192.168.1.13`，端口 `6001`）
- `tl_driver` 可正常连接机械臂（可通过 `ros2 launch tl_driver tl_<arm_type>_driver.launch.py` 验证）

**启动命令**：

```bash
ros2 launch tl_<arm_type>_config real_hardware_demo.launch.py
```

例如 tcb605 机械臂：

```bash
ros2 launch tl_tcb605_config real_hardware_demo.launch.py
```

**数据链路**：

```
MoveIt2 (move_group)
  │  规划结果 (trajectory)
  ▼
ros2_control (joint_trajectory_controller)
  │  write(): position commands (rad)
  ▼
tl_hardware (SystemInterface 插件)
  │  ├─ 将弧度转为角度，发布到 /tl_driver/set_servoj_pos
  │  ├─ 订阅 /joint_states 获取关节状态反馈
  │  └─ 调用 open_servoj / close_servoj 管理伺服流
  ▼
tl_driver (TCP/IP)
  │
  ▼
机械臂控制器
```

**工作原理**：`real_hardware_demo.launch.py` 在启动 URDF 时传入 `use_real_hardware:=true` 参数，Xacro 据此加载 `tl_hardware/TLHardwareInterface` 插件替代默认的 `mock_components/GenericSystem`，从而将关节轨迹指令转发到 `tl_driver` 所连接的真实机械臂。

---

## 3. tl_moveit2_config 架构说明

### 3.1 功能包文件总览

`tl_moveit2_config` 为 ROS2 功能包集合，由 MoveIt2 Setup Assistant 生成，包含 14 个臂型子功能包。每个子功能包均独立生成且结构一致。

```
tl_moveit2_config/
├── .setup_assistant                    # MoveIt2 Setup Assistant 全局配置
├── README.md                           # 本说明文档
├── tl_tcb605_config/                   # tcb605 机械臂 MoveIt2 配置功能包
│   ├── .setup_assistant                # Setup Assistant 配置记录
│   ├── CMakeLists.txt                  # 编译规则
│   ├── package.xml                     # 包描述文件
│   ├── config/                         # 配置参数文件夹
│   │   ├── initial_positions.yaml      # 初始化位姿（各关节默认角度）
│   │   ├── joint_limits.yaml           # 关节运动限制（速度/加速度/位置）
│   │   ├── kinematics.yaml             # 运动学求解器配置（KDL）
│   │   ├── moveit_controllers.yaml     # MoveIt2 控制器配置
│   │   ├── moveit.rviz                 # RViz2 显示配置文件
│   │   ├── pilz_cartesian_limits.yaml  # Pilz 笛卡尔规划器限制
│   │   ├── ros2_controllers.yaml       # ros2_control 控制器配置
│   │   ├── tl_tcb605.ros2_control.xacro # ros2_control Xacro 描述
│   │   ├── tl_tcb605.srdf              # SRDF（语义机器人描述格式）
│   │   └── tl_tcb605.urdf.xacro        # URDF Xacro 描述（含 ros2_control）
│   └── launch/                         # 启动文件文件夹
│       ├── demo.launch.py              # 虚拟机械臂 MoveIt2 启动文件
│       ├── gazebo_moveit_demo_tcb605.launch.py   # Gazebo 仿真 MoveIt2 启动文件
│       ├── real_hardware_demo.launch.py          # 真实机械臂 MoveIt2 启动文件
│       ├── move_group.launch.py        # move_group 启动文件
│       ├── moveit_rviz.launch.py       # RViz2 可视化启动文件
│       ├── rsp.launch.py               # robot_state_publisher 启动文件
│       ├── setup_assistant.launch.py   # Setup Assistant 启动文件
│       ├── spawn_controllers.launch.py # 控制器启动文件
│       ├── static_virtual_joint_tfs.launch.py # 静态虚拟关节 TF 发布
│       └── warehouse_db.launch.py      # warehouse 数据库启动文件
├── tl_tcb605f_config/                  # tcb605f 配置（文件解释参考 tcb605）
├── tl_tcb605l_config/                  # tcb605l 配置（文件解释参考 tcb605）
├── tl_tcb605lv_config/                 # tcb605lv 配置（文件解释参考 tcb605）
├── tl_tcb605v_config/                  # tcb605v 配置（文件解释参考 tcb605）
├── tl_tcb610_config/                   # tcb610 配置（文件解释参考 tcb605）
├── tl_tcb610v_config/                  # tcb610v 配置（文件解释参考 tcb605）
├── tl_tcb705_config/                   # tcb705 配置（文件解释参考 tcb605）
├── tl_tcb705f_config/                  # tcb705f 配置（文件解释参考 tcb605）
├── tl_tcb705l_config/                  # tcb705l 配置（文件解释参考 tcb605）
├── tl_tcb705lv_config/                 # tcb705lv 配置（文件解释参考 tcb605）
├── tl_tcb705v_config/                  # tcb705v 配置（文件解释参考 tcb605）
├── tl_tcb710_config/                   # tcb710 配置（文件解释参考 tcb605，7关节）
├── tl_tcb710v_config/                  # tcb710v 配置（文件解释参考 tcb605，7关节）
└── doc/                                # 文档图片文件夹
    └── ...
```

各子功能包文件作用说明：

| 文件 | 作用 |
|------|------|
| `CMakeLists.txt` | 编译规则 |
| `package.xml` | 包描述文件 |
| `config/initial_positions.yaml` | 初始化位姿 |
| `config/joint_limits.yaml` | 关节限制 |
| `config/kinematics.yaml` | 运动学参数 |
| `config/moveit_controllers.yaml` | MoveIt2 控制器 |
| `config/moveit.rviz` | RViz2 显示配置 |
| `config/pilz_cartesian_limits.yaml` | Pilz 笛卡尔限制 |
| `config/ros2_controllers.yaml` | ros2_control 控制器 |
| `config/<arm>.ros2_control.xacro` | Xacro 描述文件 |
| `config/<arm>.srdf` | MoveIt2 控制配置文件 |
| `config/<arm>.urdf.xacro` | URDF Xacro 描述文件 |
| `launch/demo.launch.py` | 虚拟机械臂 MoveIt2 启动文件 |
| `launch/gazebo_moveit_demo_<arm_type>.launch.py` | Gazebo 仿真 MoveIt2 启动文件，`arm_type` 如 `tcb605`（不含 tl_ 前缀） |
| `launch/real_hardware_demo.launch.py` | 真实机械臂 MoveIt2 启动文件，传入 `use_real_hardware:=true` |
| `launch/move_group.launch.py` | move_group 启动文件 |
| `launch/moveit_rviz.launch.py` | RViz2 可视化启动文件 |
| `launch/rsp.launch.py` | robot_state_publisher 启动文件 |
| `launch/setup_assistant.launch.py` | Setup Assistant 启动文件 |
| `launch/spawn_controllers.launch.py` | 控制器启动文件 |
| `launch/static_virtual_joint_tfs.launch.py` | 静态虚拟关节 TF 发布 |
| `launch/warehouse_db.launch.py` | warehouse 数据库启动文件 |

---

## 4. tl_moveit2_config-话题说明

为清晰展示 MoveIt2 控制真实机械臂时各节点间的话题通信关系，在启动 `real_hardware_demo.launch.py`（同时 `tl_driver` 已运行并连接机械臂）后，可通过如下指令查看实时 rqt_graph：

```bash
ros2 run rqt_graph rqt_graph
```

运行成功后界面将显示如下画面。

![image](doc/image4.png)

该图反映了当前运行的节点与节点之间的话题通信关系，首先查看 `/joint_states` 话题。

由图可知，`/joint_states` 话题由 `tl_driver` 发布，`joint_state_broadcaster` 获取 ros2_control 状态接口中的反馈数据后也向该话题转发。`/joint_states` 被 `/robot_state_publisher` 节点和 `/tl_hardware` 节点订阅。`/robot_state_publisher` 接收 `/joint_states` 是为了持续发布关节间的 TF 变换；`/tl_hardware` 接收 `/joint_states` 是为了获取当前机械臂的关节状态信息，作为 ros2_control 控制闭环的状态反馈输入。

`tl_hardware` 同时发布了 `/tl_driver/set_servoj_pos` 话题，该话题是机械臂透传功能的话题，通过该话题 `tl_hardware` 将规划的关节位置指令发布给 `tl_driver` 节点，`tl_driver` 接收后控制机械臂进行运动。

`tl_hardware` 为 `tl_driver` 与 MoveIt2 之间通信的桥梁，其通过 `/tcb_group_controller/follow_joint_trajectory` 动作与 `/moveit_simple_controller_manager` 进行通信，获取规划点，并进行插值运算，将插值之后的数据通过透传的方式给到 `tl_driver`。

MoveIt2 本身涉及的节点有 `move_group`、`move_group_private`、`moveit_simple_controller_manager`，它们的主要作用为实现机械臂的运动规划，并将规划信息等数据显示在 RViz2 中，另一方面还需要将规划数据传递到 `tl_hardware` 端，进行进一步细分。

---

## 常见问题

**Q：如何为现有臂型重新生成 MoveIt2 配置？**

A：使用 MoveIt2 Setup Assistant 打开对应的 SRDF 文件：
```bash
ros2 launch tl_<arm_type>_config setup_assistant.launch.py
```

**Q：如何修改关节速度/加速度限制？**

A：编辑对应臂型的 `config/joint_limits.yaml` 中的 `default_velocity_scaling_factor` 和 `default_acceleration_scaling_factor`，值域 (0, 1.0]。

**Q：启动后 RViz2 中不显示模型？**

A：确保已正确编译工作空间（`colcon build`）并 source `install/setup.bash`。检查 `tl_description` 功能包已正确安装。

**Q：控制真实机械臂前需要确保什么？**

A：需要确保：
1. `tl_driver` 可正常连接机械臂（验证：`ros2 launch tl_driver tl_<arm_type>_driver.launch.py`，观察 `/joint_states` 话题是否有数据）
2. `tl_hardware` 包已正确编译（`colcon build --packages-select tl_hardware`）
3. 机械臂已上电且网络可达

**Q：控制真实机械臂时，`real_hardware_demo.launch.py` 和 `demo.launch.py` 有什么区别？**

A：`demo.launch.py` 使用 `mock_components/GenericSystem` 模拟硬件，无需连接真实机械臂即可在 RViz2 中规划；`real_hardware_demo.launch.py` 加载 `tl_hardware/TLHardwareInterface` 插件，将规划结果下发到真实机械臂执行。后者需要 `tl_driver` 已连接到真实机械臂。

**Q：7 轴型号（tcb710/tcb710v）的配置与 6 轴有何不同？**

A：7 轴型号的 ros2_control.xacro 中定义了 7 个关节（`joint1`~`joint7`），其余配置结构完全相同。MoveIt2 会自动根据关节数适配规划维度。

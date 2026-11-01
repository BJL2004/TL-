# 天链机器人 ROS2 机械臂工作空间

TL 系列机械臂 ROS2 接口与功能包集 — 四川天链机器人股份有限公司

## 环境要求

- Ubuntu 22.04
- ROS2 Humble
- 控制器版本 2403

## 支持机械臂型号

- **6 轴系列**：TCB605、TCB605F、TCB605L、TCB605LV、TCB605V、TCB610、TCB610V（末端负载 5~10 kg）
- **7 轴系列**：TCB705、TCB705F、TCB705L、TCB705LV、TCB705V、TCB710、TCB710V（末端负载 5~10 kg）
- 命名规则：`TCB` + 轴数（6/7）+ 负载能力（05=5kg, 10=10kg）+ 后缀（F=力控, L=加长, V=视觉, LV=加长+视觉）

## 搭建环境

### 安装 ROS2

参考 ROS2 官方安装指南：

> [ROS2 Humble 安装教程](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### 编译

```bash
# 克隆仓库
git clone https://github.com/tlibot-cn/tl_robot_ros2_py.git

# 切到工作区目录
cd ~/tl_robot_ros2_py

# 初始化 Git 子模块
git submodule update --init

# 编译
colcon build
source install/setup.bash
```

编译完成后即可进行各功能包的运行操作。各功能包已在 `package.xml` 中声明了相互依赖，colcon 会自动处理构建顺序。

### 代码格式化

```bash
pip install black
```

`git commit` 时会自动通过 `.githooks/pre-commit` 运行 `black` 格式化 Python 代码。首次克隆后需执行：

```bash
git config core.hooksPath .githooks
```

> **关于 tl_teleop 遥操作功能包**：`tl_teleop` 的 `setup.py` 已集成 `xrobotoolkit_sdk`（VR 手柄 Python 绑定库）的自动构建与安装。编译前建议先阅读 [`src/tl_teleop/README.md`](src/tl_teleop/README.md) 中的「xrobotoolkit_sdk 安装」小节，了解原生库依赖和两种安装方式。

## 运行

### 真实机械臂

```bash
source ~/tl_robot_ros2_py/install/setup.bash
ros2 launch tl_bringup tl_<arm_type>_bringup.launch.py
```

`<arm_type>` 可选：`tcb605`、`tcb605f`、`tcb605l`、`tcb605lv`、`tcb605v`、`tcb610`、`tcb610v`、`tcb705`、`tcb705f`、`tcb705l`、`tcb705lv`、`tcb705v`、`tcb710`、`tcb710v`

**修改通信参数**：编辑 `src/tl_driver/config/` 下对应型号的 YAML，修改 `arm_ip`、`arm_port` 等参数。

### MoveIt2 + RViz 虚拟控制

```bash
source ~/tl_robot_ros2_py/install/setup.bash
ros2 launch tl_<arm_type>_config demo.launch.py
```

替换 `<arm_type>` 为对应型号名称。

### Gazebo 仿真

```bash
source ~/tl_robot_ros2_py/install/setup.bash
ros2 launch tl_gazebo gazebo_<arm_type>_demo.launch.py
```

### MoveIt2 + 真实机械臂

通过 `tl_hardware`（ros2_control 硬件接口插件）连接 MoveIt2 与真实机械臂：

```bash
source ~/tl_robot_ros2_py/install/setup.bash
ros2 launch tl_<arm_type>_config real_hardware_demo.launch.py
```

数据链路：MoveIt2 → ros2_control（joint_trajectory_controller）→ `tl_hardware`（话题/服务）→ `tl_driver`（TCP）→ 机械臂

## 工作空间结构

```
tl_robot_ros2_py/
├── src/               # ROS2 功能包源码（10 个包）
│   ├── tl_bringup/        # 启动聚合
│   ├── tl_description/    # URDF 模型描述
│   ├── tl_driver/         # 机械臂驱动（Python）
│   ├── tl_example/        # 使用示例
│   ├── tl_gazebo/         # Gazebo 仿真
│   ├── tl_hardware/       # ros2_control 硬件接口插件（C++）
│   ├── tl_moveit2_config/ # MoveIt2 配置（14 个子包）
│   ├── tl_ros2_interface/ # 自定义 msg/srv
│   ├── tl_teleop/         # VR 手柄遥操作
│   └── tl_teleop_f710/    # F710 手柄遥操作
├── build/             # 编译中间产物（已 gitignore）
├── install/           # 编译输出（已 gitignore）
└── log/               # 编译日志（已 gitignore）
```

各功能包的详细用途请参见 [`src/README.md`](src/README.md)。

## 安全注意事项

- 每次使用前检查机械臂安装情况，包括固定螺丝是否松动
- 机械臂运行过程中，人员不可处于机械臂工作范围内
- 不使用时将机械臂置于安全位置并断开电源

## 许可与联系

**许可证**： Apache License 2.0

**公司官网**：[https://www.tlibot.com/](https://www.tlibot.com/)

**四川天链机器人股份有限公司**

如有问题，请在仓库 issue 提出或联系包维护者。

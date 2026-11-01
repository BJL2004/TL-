<div align="center">

# 天链机器人 tl_description 使用说明书

</div>

## 目录
- 1.[tl_description 功能包说明](#tl_description-功能包说明)
- 2.[tl_description 功能包使用](#tl_description-功能包使用)
- 3.[tl_description 功能包架构说明](#tl_description-功能包架构说明)
- 4.[tl_description 功能包话题说明](#tl_description-功能包话题说明)

## tl_description 功能包说明

tl_description 功能包为显示机器人模型和 TF 变换的功能包，通过该功能包可以实现电脑中的虚拟机械臂与现实中的实际机械臂联动的效果。
* 1.功能包使用
* 2.功能包架构说明
* 3.功能包话题说明

通过这三部分内容的介绍可以帮助大家：
* 1.了解该功能包的使用
* 2.熟悉功能包中的文件构成及作用
* 3.熟悉功能包相关的话题，方便开发和使用

## tl_description 功能包使用

配置环境后通过以下命令直接启动节点，运行 tl_description 功能包。使用时需将 `<arm_type>` 更换为实际的机械臂型号，`<use_sim>` 选择是否进行仿真控制。

可选择的机械臂型号（共 14 种）：

| 型号 | 轴数 | 是否带视觉 |
|------|------|-----------|
| `tcb605` | 6轴 | 否 |
| `tcb605f` | 6轴 | 否 |
| `tcb605l` | 6轴 | 否 |
| `tcb605lv` | 6轴 | 是（camera_link） |
| `tcb605v` | 6轴 | 是（camera_link） |
| `tcb610` | 6轴 | 否 |
| `tcb610v` | 6轴 | 是（camera_link） |
| `tcb705` | 7轴 | 否 |
| `tcb705f` | 7轴 | 否 |
| `tcb705l` | 7轴 | 否 |
| `tcb705lv` | 7轴 | 是（camera_link） |
| `tcb705v` | 7轴 | 是（camera_link） |
| `tcb710` | 7轴 | 否 |
| `tcb710v` | 7轴 | 是（camera_link） |

```bash
ros2 launch tl_description tl_description.launch.py arm_type:=<arm_type> use_sim:=<use_sim>
```

### 仿真控制模式

`use_sim:=true` 时，`joint_state_publisher_gui` 节点会发布关节状态话题 `/joint_states` 并生成 GUI 滑动条，通过拖动滑动条可以手动控制每个关节的角度。例如启动 tcb605 机械臂：

```bash
ros2 launch tl_description tl_description.launch.py arm_type:=tcb605 use_sim:=true
```

节点启动成功后，将弹出以下画面，通过拖动滑动条可以控制每个关节的角度：

<div align="center">

![image](doc/tl_description.png)

</div>

### 真实机械臂控制模式

`use_sim:=false` 时，节点需要订阅 `/joint_states` 话题以计算机械臂各连杆的 TF 变换，否则 RViz2 中无法正常显示模型运动。因此需要先启动 `tl_driver` 功能包提供 `/joint_states` 话题输入：

```bash
# 先启动驱动
ros2 launch tl_driver tl_driver.launch.py arm_type:=tcb605

# 再启动 description（另一终端）
ros2 launch tl_description tl_description.launch.py arm_type:=tcb605 use_sim:=false
```

节点启动成功后，RViz2 中的机械臂模型会跟随真实机械臂进行对应角度的运动。

## tl_description 功能包架构说明

### 功能包文件总览

```
├── config                                   # 关节名称配置文件（共 14 个）
│   ├── joint_names_tcb605.yaml
│   ├── joint_names_tcb605f.yaml
│   ├── joint_names_tcb605l.yaml
│   ├── joint_names_tcb605lv.yaml
│   ├── joint_names_tcb605v.yaml
│   ├── joint_names_tcb610.yaml
│   ├── joint_names_tcb610v.yaml
│   ├── joint_names_tcb705.yaml
│   ├── joint_names_tcb705f.yaml
│   ├── joint_names_tcb705l.yaml
│   ├── joint_names_tcb705lv.yaml
│   ├── joint_names_tcb705v.yaml
│   ├── joint_names_tcb710.yaml
│   └── joint_names_tcb710v.yaml
├── doc                                      # 辅助文档、图片文件
│   └── tl_description.png
├── launch                                   # 启动文件
│   └── tl_description.launch.py
├── meshes                                   # 各型号 STL 网格模型文件
│   ├── tcb605                               # 6轴，无相机
│   │   ├── link0.STL ~ link6.STL
│   ├── tcb605f                              # 6轴，无相机
│   │   ├── link0.STL ~ link6.STL
│   ├── tcb605l                              # 6轴，无相机
│   │   ├── link0.STL ~ link6.STL
│   ├── tcb605lv                             # 6轴，带相机
│   │   ├── camera_link.STL
│   │   ├── link0.STL ~ link6.STL
│   ├── tcb605v                              # 6轴，带相机
│   │   ├── camera_link.STL
│   │   ├── link0.STL ~ link6.STL
│   ├── tcb610                               # 6轴，无相机
│   │   ├── link0.STL ~ link6.STL
│   ├── tcb610v                              # 6轴，带相机
│   │   ├── camera_link.STL
│   │   ├── link0.STL ~ link6.STL
│   ├── tcb705                               # 7轴，无相机
│   │   ├── link0.STL ~ link7.STL
│   ├── tcb705f                              # 7轴，无相机
│   │   ├── link0.STL ~ link7.STL
│   ├── tcb705l                              # 7轴，无相机
│   │   ├── link0.STL ~ link7.STL
│   ├── tcb705lv                             # 7轴，带相机
│   │   ├── camera_link.STL
│   │   ├── link0.STL ~ link7.STL
│   ├── tcb705v                              # 7轴，带相机
│   │   ├── camera_link.STL
│   │   ├── link0.STL ~ link7.STL
│   ├── tcb710                               # 7轴，无相机
│   │   ├── link0.STL ~ link7.STL
│   └── tcb710v                              # 7轴，带相机
│       ├── camera_link.STL
│       ├── link0.STL ~ link7.STL
├── package.xml                              # 依赖说明文件
├── README.md                                # 说明文档
├── resource                                 # 资源标记文件
│   └── tl_description
├── rviz                                     # RViz2 配置文件（共 14 个）
│   ├── tcb605.rviz
│   ├── tcb605f.rviz
│   ├── tcb605l.rviz
│   ├── tcb605lv.rviz
│   ├── tcb605v.rviz
│   ├── tcb610.rviz
│   ├── tcb610v.rviz
│   ├── tcb705.rviz
│   ├── tcb705f.rviz
│   ├── tcb705l.rviz
│   ├── tcb705lv.rviz
│   ├── tcb705v.rviz
│   ├── tcb710.rviz
│   └── tcb710v.rviz
├── setup.cfg                                # Python 包配置文件
├── setup.py                                 # Python 编译规则文件
└── urdf                                     # URDF 描述文件 + CSV 参数表
    ├── tcb605.csv
    ├── tcb605.urdf
    ├── tcb605f.csv
    ├── tcb605f.urdf
    ├── tcb605l.csv
    ├── tcb605l.urdf
    ├── tcb605lv.csv
    ├── tcb605lv.urdf
    ├── tcb605v.csv
    ├── tcb605v.urdf
    ├── tcb610.csv
    ├── tcb610.urdf
    ├── tcb610v.csv
    ├── tcb610v.urdf
    ├── tcb705.csv
    ├── tcb705.urdf
    ├── tcb705f.csv
    ├── tcb705f.urdf
    ├── tcb705l.csv
    ├── tcb705l.urdf
    ├── tcb705lv.csv
    ├── tcb705lv.urdf
    ├── tcb705v.csv
    ├── tcb705v.urdf
    ├── tcb710.csv
    ├── tcb710.urdf
    ├── tcb710v.csv
    └── tcb710v.urdf
```

### 功能包节点说明

| 节点 | 功能 | 条件 |
|------|------|------|
| `robot_state_publisher` | 加载 URDF 并发布 `/tf` / `/tf_static` | 始终启动 |
| `joint_state_publisher_gui` | 生成关节滑块 GUI，发布 `/joint_states` | `use_sim:=true` 时启动 |
| `rviz2` | 可视化显示机器人模型 | 始终启动 |

### 文件生成说明

URDF 及 STL 文件由 **SolidWorks URDF 导出器**（sw_urdf_exporter）生成，CSV 文件为导出时的参数表副产品（仅作参考，运行时不被使用）。

## tl_description 功能包话题说明

如下为该功能包运行时的话题说明：

```
Subscribers:
  /joint_states:          sensor_msgs/msg/JointState
  /parameter_events:      rcl_interfaces/msg/ParameterEvent

Publishers:
  /parameter_events:      rcl_interfaces/msg/ParameterEvent
  /robot_description:     std_msgs/msg/String
  /rosout:                rcl_interfaces/msg/Log
  /tf:                    tf2_msgs/msg/TFMessage
  /tf_static:             tf2_msgs/msg/TFMessage

Service Servers:
  /robot_state_publisher/describe_parameters:         rcl_interfaces/srv/DescribeParameters
  /robot_state_publisher/get_parameter_types:         rcl_interfaces/srv/GetParameterTypes
  /robot_state_publisher/get_parameters:              rcl_interfaces/srv/GetParameters
  /robot_state_publisher/list_parameters:             rcl_interfaces/srv/ListParameters
  /robot_state_publisher/set_parameters:              rcl_interfaces/srv/SetParameters
  /robot_state_publisher/set_parameters_atomically:   rcl_interfaces/srv/SetParametersAtomically
```

主要关注以下几个话题：

- **Subscribers — `/joint_states`**：代表机械臂当前关节状态。仿真模式下 `joint_state_publisher_gui` 发布该话题，通过 GUI 滑动条控制机械臂；真实机械臂模式下 `tl_driver` 发布该话题，使 RViz2 模型跟随真实机械臂运动。

- **Publishers — `/tf` 和 `/tf_static`**：描述机械臂关节与关节之间的坐标变换关系（TF 变换），`robot_state_publisher` 根据 URDF 模型和关节状态自动计算并发布。

其余话题和服务使用场景较少，可按需了解。

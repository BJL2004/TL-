<div align="center">

# tl_driver服务与话题说明书

文件修订记录：

|版本号 | 时间 | 备注 |
| :---: | :---- | :---: |
|V1.0 | 2026-4-29 | 拟制 |
|V1.1 | 2026-5-9  | 修订（添加[查询机械臂状态](#查询机械臂状态)、[查询库版本信息](#查询库版本信息)、[查询关节参数](#查询关节参数)、[设置关节参数](#设置关节参数)、[查询关节温度](#查询关节温度)、[查询关节电压](#查询关节电压)、[查询电机电流](#查询电机电流)、[查询关节软件版本号](#查询关节软件版本号)、<br>[查询算法库版本](#查询算法库版本)、[设置机械臂默认DH参数](#设置机械臂默认DH参数)、[设置机械臂默认笛卡尔参数](#设置机械臂默认笛卡尔参数)、[日志下载](#日志下载)、[查询运行速度](#查询运行速度)、[设置坐标系编号](#设置坐标系编号)、[设置机械臂DH参数](#设置机械臂DH参数)、<br>[四元数转欧拉角](#四元数转欧拉角)、[欧拉角转四元数](#欧拉角转四元数)、[欧拉角转旋转矩阵](#欧拉角转旋转矩阵)、[位姿转旋转矩阵](#位姿转旋转矩阵)、[旋转矩阵转位姿](#旋转矩阵转位姿)、[旋转矩阵转位姿](#旋转矩阵转位姿)、[设置控制器有线网口IP](#设置控制器有线网口IP)、<br>[查询控制器序列号ID](#查询控制器序列号ID)、[查询当前坐标系](#查询当前坐标系)等接口|
|V1.2 | 2026-5-28 | 修订（job_insert_* 接口从话题改为服务、移除 tool_hand_calib、新增 [删除作业文件](#删除作业文件) 服务、新增 [查询机械臂运行状态](#查询机械臂运行状态) 话题、新增 [查询当前运行模式](#查询当前运行模式) 服务，补漏 `get_current_mode`） |
|V1.3 | 2026-5-29 | 新增 [查询电机扭矩](#查询电机扭矩)、[查询当前线速度与关节速度](#查询当前线速度与关节速度) |
|V1.4 | 2026-6-9 | 统一修订：所有接口改用结构化参数/返回值表格格式（服务名/话题名、服务类型/消息类型、输入参数表、输出/返回值表） |
|V1.5 | 2026-6-16 | 新增[发送跟踪笛卡尔位姿](#发送跟踪笛卡尔位姿) |

</div>

## 目录
* 1[通用约定](#通用约定)
* 1.1[坐标系编号](#坐标系编号)
* 1.2[运行模式编号](#运行模式编号)
* 1.3[返回值约定](#返回值约定)
* 1.4[单位制约定](#单位制约定)
* 2[连接管理接口](#连接管理接口)
* 2.1[机械臂连接](#机械臂连接)
* 2.2[机械臂断开连接](#机械臂断开连接)
* 2.3[机械臂上电](#机械臂上电)
* 2.4[机械臂下电](#机械臂下电)
* 3[日志管理接口](#日志管理接口)
* 3.1[日志下载](#日志下载)
* 4[信息查询接口](#信息查询接口)
* 4.1[查询关节角度](#查询关节角度)
* 4.2[查询末端位姿](#查询末端位姿)
* 4.3[查询运行速度](#查询运行速度)
* 4.4[查询控制器序列号ID](#查询控制器序列号ID)
* 4.5[查询机械臂状态](#查询机械臂状态)
* 4.6[查询库版本信息](#查询库版本信息)
* 4.7[查询关节参数](#查询关节参数)
* 4.8[查询关节温度](#查询关节温度)
* 4.9[查询关节电压](#查询关节电压)
* 4.10[查询电机电流](#查询电机电流)
* 4.11[查询关节软件版本号](#查询关节软件版本号)
* 4.12[查询算法库版本](#查询算法库版本)
* 4.13[查询当前坐标系](#查询当前坐标系)
* 4.14[查询坐标系编号](#查询坐标系编号)
* 4.15[查询机械臂DH参数](#查询机械臂DH参数)
* 4.16[查询所有作业文件名称](#查询所有作业文件名称)
* 4.17[查询目标位姿可达状态](#查询目标位姿可达状态)
* 4.18[查询机械臂运行状态](#查询机械臂运行状态)
* 4.19[查询电机扭矩](#查询电机扭矩)
* 4.20[查询当前线速度与关节速度](#查询当前线速度与关节速度)
* 5[机械臂基础功能设置接口](#机械臂基础功能设置接口)
* 5.1[设置运行速度](#设置运行速度)
* 5.2[设置控制器有线网口IP](#设置控制器有线网口IP)
* 5.3[设置关节参数](#设置关节参数)
* 5.4[设置机械臂默认DH参数](#设置机械臂默认DH参数)
* 5.5[设置机械臂默认笛卡尔参数](#设置机械臂默认笛卡尔参数)
* 5.6[坐标转换](#坐标转换)
* 5.7[设置机械臂DH参数](#设置机械臂DH参数)
* 6[作业运动控制接口](#作业运动控制接口)
* 6.1[向作业文件插入一条moveJ关节运动](#向作业文件插入一条moveJ关节运动)
* 6.2[向作业文件插入一条moveL关节运动](#向作业文件插入一条moveL关节运动)
* 6.3[向作业文件插入一条增量指令IMove](#向作业文件插入一条增量指令IMove)
* 6.4[向作业文件插入一条moveC关节运动](#向作业文件插入一条moveC关节运动)
* 6.5[运行作业文件](#运行作业文件)
* 6.6[删除作业文件](#删除作业文件)
* 7[实时运动控制接口](#实时运动控制接口)
* 7.1[MoveJ运动控制](#MoveJ运动控制)
* 7.2[MoveL运动控制](#MoveL运动控制)
* 8[坐标系与工具管理接口](#坐标系与工具管理接口)
* 8.1[设置工具手参数](#设置工具手参数)
* 8.2[工具手参数标定](#工具手参数标定)
* 8.3[设置用户坐标系](#设置用户坐标系)
* 8.4[设置坐标系编号](#设置坐标系编号)
* 8.5[设置当前坐标系](#设置当前坐标系)
* 9[示教操作接口](#示教操作接口)
* 9.1[开始点动](#开始点动)
* 9.2[停止点动](#停止点动)
* 9.3[设置拖拽模式](#设置拖拽模式)
* 9.4[查看拖拽状态](#查看拖拽状态)
* 9.5[拖拽轨迹保存](#拖拽轨迹保存)
* 9.6[拖拽轨迹回放](#拖拽轨迹回放)
* 10[错误处理与零位标定接口](#错误处理与零位标定接口)
* 10.1[清除错误](#清除错误)
* 10.2[设置关节零点](#设置关节零点)
* 11[全局路点管理接口](#全局路点管理接口)
* 11.1[查询全局位点](#查询全局位点)
* 11.2[设置全局位点](#设置全局位点)
* 12[模式与IO控制接口](#模式与IO控制接口)
* 12.1[设置当前运行模式](#设置当前运行模式)
* 12.2[查询当前运行模式](#查询当前运行模式)
* 12.3[设置数字输出](#设置数字输出)
* 12.4[查询数字输入输出状态](#查询数字输入输出状态)
* 13[modbus通信接口](#modbus通信接口)
* 13.1[写Modbus](#写Modbus)
* 13.2[读Modbus](#读Modbus)
* 14[队列运动接口](#队列运动接口)
* 14.1[设置MoveJ队列运动模式](#设置MoveJ队列运动模式)
* 14.2[MoveJ队列运动](#MoveJ队列运动)
* 14.3[停止MoveJ队列运动模式](#停止MoveJ队列运动模式)
* 15[7000端口](#7000端口)
* 15.1[打开关节跟踪模式](#打开关节跟踪模式)
* 15.2[关闭关节跟踪模式](#关闭关节跟踪模式)
* 15.3[发送跟踪关节位置](#发送跟踪关节位置)
* 15.4[发送跟踪笛卡尔位姿](#发送跟踪笛卡尔位姿)
* 16[位姿转换工具接口](#位姿转换工具接口)
* 16.1[四元数转欧拉角](#四元数转欧拉角)
* 16.2[欧拉角转四元数](#欧拉角转四元数)
* 16.3[欧拉角转旋转矩阵](#欧拉角转旋转矩阵)
* 16.4[位姿转旋转矩阵](#位姿转旋转矩阵)
* 16.5[旋转矩阵转位姿](#旋转矩阵转位姿)

## 通用约定
### 坐标系编号
| 编号 | 含义 | 说明 |
| :---: | :--- | :--- |
| 1 | 关节坐标系（Joint） | 以各关节角度表示位置，单位：度（°）或弧度（rad） |
| 2 | 直角坐标系（Cartesian/Base） | 以基座为原点，位置(X,Y,Z)单位mm，姿态(RX,RY,RZ)单位rad |
| 3 | 工具坐标系（Tool） | 以工具末端为参考的坐标系 |
| 4 | 用户坐标系（User） | 用户自定义的坐标系 |

### 运行模式编号
| 编号 | 含义 |
| :---: | :--- |
| 1 | 示教模式（手动操作） |
| 2 | 远程模式 |
| 3 | 运行模式（自动运行作业） |

### 返回值约定
- 所有 ROS2 服务的响应均包含 `bool success` 和 `string message` 字段
- `success = true`：操作成功（对应底层SDK返回值0）
- `success = false`：操作失败，`message` 中包含错误描述信息
- 对于 `std_srvs::srv::Trigger` 类型的服务，`success` 字段直接承载上述含义

### 单位制约定
- `target_pos_type` 字段中携带单位制信息：`1` = 角度制（度），`2` = 弧度制（rad）
- ROS标准消息（如 `sensor_msgs/JointState`、`CartesianPose`）中的角度/位置遵循ROS标准单位制
- SDK底层API中 `get_current_position` 等接口返回的角度单位为度（°）

---
## 连接管理接口
### 机械臂连接
| 功能描述 | 建立机械臂网络通信连接 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/connect_arm` |
| 服务类型 | `std_srvs::srv::Trigger` |

**输入参数**
无请求参数（空 `Trigger`）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 连接成功，`false` = 连接失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/connect_arm std_srvs/srv/Trigger "{}"
```
### 机械臂断开连接
| 功能描述 | 断开机械臂网络通信连接 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/disconnect_arm` |
| 服务类型 | `std_srvs::srv::Trigger` |

**输入参数**
无请求参数（空 `Trigger`）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 断开连接成功，`false` = 断开连接失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/disconnect_arm std_srvs/srv/Trigger "{}" 
```
### 机械臂上电
| 功能描述 | 机械臂使能上电 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/power_on` |
| 服务类型 | `std_srvs::srv::Trigger` |

**输入参数**
无请求参数（空 `Trigger`）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 上电成功，`false` = 上电失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/power_on std_srvs/srv/Trigger "{}"
```
### 机械臂下电
| 功能描述 | 机械臂使能下电 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/power_off` |
| 服务类型 | `std_srvs::srv::Trigger` |

**输入参数**
无请求参数（空 `Trigger`）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 下电成功，`false` = 下电失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/power_off std_srvs/srv/Trigger "{}"
```
## 日志管理接口
### 日志下载
| 功能描述 | 下载指定数量的日志到指定文件夹 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/log_download` |
| 服务类型 | `tl_ros2_interface/srv/LogDownload` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| count | int32 | 日志数量 |
| directory_path | string | 保存目录路径 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 下载成功，`false` = 下载失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/log_download tl_ros2_interface/srv/LogDownload "{count: 1, directory_path: '/home/ubuntu/桌面'}"
```
## 信息查询接口
### 查询关节角度
| 功能描述 | 查询关节角度 |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 话题名 | `/joint_states` |
| 消息类型 | `sensor_msgs::msg::JointState` |

**输入参数**
无参数（话题订阅）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| position[] | float64[] | 各关节角度（rad） |
| velocity[] | float64[] | 各关节速度 |
| effort[] | float64[] | 各关节力矩 |
#### 命令示例
```
ros2 topic echo /joint_states
```
### 查询末端位姿
| 功能描述 | 查询机械臂末端位置 |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 话题名 | `/tcp_pose` |
| 消息类型 | `tl_ros2_interface/msg/CartesianPose` |

**输入参数**
无参数（话题订阅）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| position | geometry_msgs/Point | 末端位置 (X, Y, Z) mm |
| rpy | geometry_msgs/Vector3 | 姿态欧拉角 (RX, RY, RZ) rad |
| arm_angle | float64 | 臂角 |
#### 命令示例
```
ros2 topic echo /tcp_pose
```
### 查询运行速度
| 功能描述 | 查询机械臂运行速度 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_speed` |
| 服务类型 | `tl_ros2_interface/srv/GetSpeed` |

**输入参数**
无请求参数。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 失败时包含错误描述 |
| speed | float64 | 运行速度 |
#### 命令示例
```
ros2 service call /tl_driver/get_speed tl_ros2_interface/srv/GetSpeed "{}"
```
### 查询控制器序列号ID
| 功能描述 | 查询控制器序列号ID |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_controller_id` |
| 服务类型 | `std_srvs::srv::Trigger` |

**输入参数**
无请求参数（空 `Trigger`）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 查询成功时返回控制器序列号ID，失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/get_controller_id std_srvs/srv/Trigger "{}" 
```
### 查询机械臂状态
| 功能描述 | 查询机械臂详细状态 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_robot_state` |
| 服务类型 | `tl_ros2_interface/srv/GetRobotState` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| channel | int32 | 查询通道 |
| stop | bool | 是否停止发送 |
| mode | int32 | 查询模式：0-只回复一次 1-持续回复 |
| interval | int32 | 仅mode=1时有效，回复时间范围 [10,60000] ms |
| io_state | bool | 查询IO |
| position | int32 | 0-关节坐标 1-直角坐标 |
| detail_motion_pos | bool | 机械臂的运动点位 |
| pos_sum | int32 | 当查询机械臂运动点位时，每帧数据回复的点位数目 |
| io_port | string[] | IO端口，可查询的最大数量不可大于IO实际个数 例：["DI1", "DI16", "DO1", "DO3", "DO17"] |
| optional | string[] | 查询运动点位返回的坐标类型："ACS"-关节参数 "MCS"-直角参数 "time"-时间戳 "reset"-重置点位记录 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 查询成功时返回对应的查询信息，失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/get_robot_state tl_ros2_interface/srv/GetRobotState \
"{
    channel: 1, 
    stop: false, 
    mode: 0, 
    interval: 10, 
    io_state: false, 
    position: 0, 
    detail_motion_pos: false, 
    pos_sum: 1, 
    io_port: ["DO1"], 
    optional: ["ACS"]
}"
```
### 查询库版本信息
| 功能描述 | 查询API库版本相关信息 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_library_version` |
| 服务类型 | `std_srvs::srv::Trigger` |

**输入参数**
无请求参数（空 `Trigger`）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 查询成功时返回库版本信息，失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/get_library_version std_srvs/srv/Trigger "{}"
```
### 查询关节参数
| 功能描述 | 查询机械臂关节参数 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_robot_joint_param` |
| 服务类型 | `tl_ros2_interface/srv/GetRobotJointParam` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| id | int32 | 关节序号 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 失败时包含错误描述 |
| param.reduction_ratio | float64 | 关节减速比 |
| param.encoder_resolution | int32 | 编码器位数 |
| param.pos_sw_limit | float64 | 轴正限位 |
| param.neg_sw_limit | float64 | 轴反限位 |
| param.rated_rot_speed | float64 | 电机额定正转速 |
| param.rated_derot_speed | float64 | 电机额定反转速 |
| param.max_rot_speed | float64 | 电机最大正转速 |
| param.max_derot_speed | float64 | 电机最大反转速 |
| param.rated_vel | float64 | 额定正速度 |
| param.rated_devel | float64 | 额定反速度 |
| param.max_acc | float64 | 最大加速度 |
| param.max_deacc | float64 | 最大减速度 |
| param.direction | int32 | 模型方向 |
#### 命令示例
```
ros2 service call /tl_driver/get_robot_joint_param tl_ros2_interface/srv/GetRobotJointParam "{id: 1}"
```
### 查询关节温度
| 功能描述 | 查询关节温度 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_joint_temperature` |
| 服务类型 | `tl_ros2_interface/srv/GetJointTemperature` |

**输入参数**
无请求参数。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 失败时包含错误描述 |
| temperatures | float64[] | 各个关节温度 |
#### 命令示例
```
ros2 service call /tl_driver/get_joint_temperature tl_ros2_interface/srv/GetJointTemperature "{}"
```
### 查询关节电压
| 功能描述 | 查询关节电压 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_joint_voltage` |
| 服务类型 | `tl_ros2_interface/srv/GetJointVoltage` |

**输入参数**
无请求参数。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 失败时包含错误描述 |
| joint_voltage | float64[] | 机械臂本体各关节电压 |
| positioner_voltage | float64[] | 外部轴各关节电压 |
#### 命令示例
```
ros2 service call /tl_driver/get_joint_voltage tl_ros2_interface/srv/GetJointVoltage "{}"
```
### 查询电机电流
| 功能描述 | 查询独立轴当前电机电流 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_motor_current` |
| 服务类型 | `tl_ros2_interface/srv/GetMotorCurrent` |

**输入参数**
无请求参数。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 失败时包含错误描述 |
| motor_current | float64[] | 机械臂独立轴电机电流 |
#### 命令示例
```
ros2 service call /tl_driver/get_motor_current tl_ros2_interface/srv/GetMotorCurrent "{}"
```
### 查询关节软件版本号
| 功能描述 | 查询指定关节（轴）软件版本号 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_joint_software_version` |
| 服务类型 | `tl_ros2_interface/srv/GetJointSoftwareVersion` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| axis_num | int32 | 关节（轴）号 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 查询成功时返回指定关节软件版本号，失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/get_joint_software_version tl_ros2_interface/srv/GetJointSoftwareVersion "{axis_num: 1}"
```
### 查询算法库版本
| 功能描述 | 查询算法库版本信息 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_nexmotion_lib_version` |
| 服务类型 | `std_srvs::srv::Trigger` |

**输入参数**
无请求参数（空 `Trigger`）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 查询成功时返回算法库版本信息，失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/get_nexmotion_lib_version std_srvs/srv/Trigger "{}"
```
### 查询当前坐标系
| 功能描述 | 查询当前坐标系 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_current_coord` |
| 服务类型 | `tl_ros2_interface/srv/GetCurrentCoord` |

**输入参数**
无请求参数。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 失败时包含错误描述 |
| coord | int32 | 坐标系序号：0-关节 1-直角 2-工具 3-用户 |
#### 命令示例
```
ros2 service call /tl_driver/get_current_coord tl_ros2_interface/srv/GetCurrentCoord "{}"
```
### 查询坐标系编号
| 功能描述 | 查询工具坐标系和用户坐标系编号 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_coord_num` |
| 服务类型 | `tl_ros2_interface/srv/GetCoordNum` |

**输入参数**
无请求参数。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 失败时包含错误描述 |
| tool_num | int32 | 工具坐标系序号 |
| user_num | int32 | 用户坐标系序号 |
#### 命令示例
```
ros2 service call /tl_driver/get_coord_num tl_ros2_interface/srv/GetCoordNum "{}"
```
### 查询机械臂DH参数
| 功能描述 | 查询机械臂DH参数 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_dh_param` |
| 服务类型 | `tl_ros2_interface/srv/GetDHParam` |

**输入参数**
无请求参数。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 失败时包含错误描述 |
| param | RobotDHParam | 机械臂DH参数（包含 l1~l6 等连杆参数） |
#### 命令示例
```
ros2 service call /tl_driver/get_dh_param tl_ros2_interface/srv/GetDHParam
```
### 查询所有作业文件名称
| 功能描述 | 查询所有作业文件名称 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_all_job_filename` |
| 服务类型 | `tl_ros2_interface/srv/GetAllJobFileName` |

**输入参数**
无请求参数。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 失败时包含错误描述 |
| file_name | string[] | 作业文件名称列表 |
#### 命令示例
```
ros2 service call /tl_driver/get_all_job_filename tl_ros2_interface/srv/GetAllJobFileName "{}"
```
### 查询目标位姿可达状态
| 功能描述 | 查询目标位姿可达状态 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_pos_reachable` |
| 服务类型 | `tl_ros2_interface/srv/GetPosReachable` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| pos | float64[] | 查询位姿 |
| move_type | string | 运动方式 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 目标位姿可达，`false` = 目标位姿不可达 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/get_pos_reachable tl_ros2_interface/srv/GetPosReachable \
'{
    pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -57.14, -32.93, 19.74, -89.89, -19.77, 0.0], move_type: "MOVJ"
}'
```
### 查询机械臂运行状态
| 功能描述 | 查询机械臂当前运行状态 |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 话题名 | `/arm_status` |
| 消息类型 | `tl_ros2_interface/msg/ArmStatus` |

**输入参数**
无参数（话题订阅）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| run_state | string | 运行状态：`STOP`-停止 `PAUSE`-暂停 `RUNNING`-运行中 |
#### 命令示例
```
ros2 topic echo /arm_status
```
### 查询电机扭矩
| 功能描述 | 获取当前电机扭矩 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_current_motor_torque` |
| 服务类型 | `tl_ros2_interface/srv/GetCurrentMotorTorque` |

**输入参数**
无请求参数。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 失败时包含错误描述 |
| motor_torque | int32[] | 机器人扭矩（长度7，单位%） |
| motor_torque_sync | int32[] | 外部轴扭矩（长度5，单位%） |
#### 命令示例
```
ros2 service call /tl_driver/get_current_motor_torque tl_ros2_interface/srv/GetCurrentMotorTorque "{}"
```
### 查询当前线速度与关节速度
| 功能描述 | 获取当前末端线速度和轴速度 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_current_line_joint_speed` |
| 服务类型 | `tl_ros2_interface/srv/GetCurrentLineJointSpeed` |

**输入参数**
无请求参数。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 失败时包含错误描述 |
| line_speed | float32 | 末端线速度（mm/s） |
| joint_speed | float32[] | 关节速度（长度5，单位°/s） |
| joint_speed_sync | float32[] | 外部轴关节速度（长度5，单位°/s） |
#### 命令示例
```
ros2 service call /tl_driver/get_current_line_joint_speed tl_ros2_interface/srv/GetCurrentLineJointSpeed "{}"
```
## 机械臂基础功能设置接口
### 设置运行速度
| 功能描述 | 设置机械臂运行速度 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_speed` |
| 服务类型 | `tl_ros2_interface/srv/SetSpeed` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| speed | float64 | 运行速度 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/set_speed tl_ros2_interface/srv/SetSpeed "{speed: 20.0}"
```
### 设置控制器有线网口IP
| 功能描述 | 设置控制器有线网口IP |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_controller_ip` |
| 服务类型 | `tl_ros2_interface/srv/SetControllerIP` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| name | string | 配置名称 |
| addr | string | IP地址 |
| gateway | string | 网关 |
| dns | string | DNS域名 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/set_controller_ip tl_ros2_interface/srv/SetControllerIP \
"{
    name: 'eth0',
    addr: '192.168.1.13',
    gateway: '',
    dns: ''
}"
```
### 设置关节参数
| 功能描述 | 设置机械臂关节参数 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_robot_joint_param` |
| 服务类型 | `tl_ros2_interface/srv/SetRobotJointParam` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| id | int32 | 关节序号 |
| param | RobotJointParam | 关节参数，相关参数查阅[查询关节参数](#查询关节参数) |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/set_robot_joint_param tl_ros2_interface/srv/SetRobotJointParam \
"{
    id: 1,
    param:
    {
        reduction_ratio: 1.0,
        encoder_resolution: 19,
        pos_sw_limit: 179.0,
        neg_sw_limit: -179.0,
        rated_rot_speed: 30.0,
        rated_derot_speed: -30.0,
        max_rot_speed: 1.0,
        max_derot_speed: -1.0,
        rated_vel: 180.0,
        rated_devel: -180.0,
        max_acc: 1.5,
        max_deacc: -1.5,
        direction: -1
    }
}"
```
### 设置机械臂默认DH参数
| 功能描述 | 设置机械臂默认DH参数 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/restore_default_dh_param` |
| 服务类型 | `tl_ros2_interface/srv/RestoreDefaultDHParam` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| robot_num | int32 | 机器人编号 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 恢复成功，`false` = 恢复失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/restore_default_dh_param tl_ros2_interface/srv/RestoreDefaultDHParam "{robot_num: 1}"
```
### 设置机械臂默认笛卡尔参数
| 功能描述 | 设置机械臂默认笛卡尔参数 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_default_cartesian_param` |
| 服务类型 | `std_srvs::srv::Trigger` |

**输入参数**
无请求参数（空 `Trigger`）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/set_default_cartesian_param std_srvs/srv/Trigger "{}" 
```
### 坐标转换
| 功能描述 | 坐标系数据转换 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/coord_transform` |
| 服务类型 | `tl_ros2_interface/srv/CoordTransform` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| origin_coord | int32 | 原始坐标系序号 |
| target_coord | int32 | 目标坐标系序号 |
| form | int32 | 形态 |
| origin_pos | float64[] | 原始坐标系位姿 |
| reference_pos | float64[] | 参考位姿 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 转换成功，`false` = 转换失败 |
| message | string | 失败时包含错误描述 |
| target_pos | float64[] | 目标坐标系位姿 |
#### 命令示例
```
ros2 service call /tl_driver/coord_transform tl_ros2_interface/srv/CoordTransform \
'{
    origin_coord: 0, 
    target_coord: 1, 
    form: 0, 
    origin_pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
    reference_pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}'
```
### 设置机械臂DH参数
| 功能描述 | 设置机械臂DH参数 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_dh_param` |
| 服务类型 | `tl_ros2_interface/srv/SetDHParam` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| param | RobotDHParam | 机械臂DH参数 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/set_dh_param tl_ros2_interface/srv/SetDHParam \
"{
    param:
    {
        l1: 127.5
    }
}"
```
## 作业运动控制接口
### 向作业文件插入一条moveJ关节运动
| 功能描述 | 向作业文件插入一条moveJ关节运动 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/job_insert_moveJ` |
| 服务类型 | `tl_ros2_interface/srv/JobInsertMove` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| line | int32 | 插入行序号 |
| cmd | MoveCommand | 运动指令（详见MoveCommand.msg） |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 插入成功，`false` = 插入失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
targetPosType=0为自定义数组 posInfo[14] [0]坐标系 0：关节 1：直角 2：工具 3：用户 [1]:0 角度制 1弧度制 [2]形态 [3]工具手坐标序号 [4]用户坐标序号 [5][6] 备用 [7-13] 点位信息
```
ros2 service call /tl_driver/job_insert_moveJ tl_ros2_interface/srv/JobInsertMove "{
  line: 1,
  cmd: {
    target_pos_value: [
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    ],
    target_pos_name: '',
    target_pos_type: 0,
    coord: 0,
    velocity: 20.0,
    velocity_sync: 0.0,
    acc: 20.0,
    dec: 20.0,
    pl: 0,
    time: 0,
    tool_num: 0,
    user_num: 0,
    posidtype: 0,
    configuration: 0,
    spin: 0,
    para_sync: false
  }
}"
```
targetPosType=1,需要设置targetPosName为"P0001",默认中间三个0
此时target_pos_value根据实际机械臂轴数来设定参数数量，输入关节角度
```
ros2 service call /tl_driver/job_insert_moveJ tl_ros2_interface/srv/JobInsertMove "{
  line: 1,
  cmd: {
    target_pos_value: [
      10.0, 20.0, 0.0, 0.0, 0.0, 0.0
    ],
    target_pos_name: 'P0001',
    target_pos_type: 1,
    coord: 0,
    velocity: 20.0,
    velocity_sync: 0.0,
    acc: 20.0,
    dec: 20.0,
    pl: 0,
    time: 0,
    tool_num: 0,
    user_num: 0,
    posidtype: 0,
    configuration: 0,
    spin: 0,
    para_sync: false
  }
}"

```
### 向作业文件插入一条moveL关节运动
| 功能描述 | 向作业文件插入一条moveL关节运动 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/job_insert_moveL` |
| 服务类型 | `tl_ros2_interface/srv/JobInsertMove` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| line | int32 | 插入行序号 |
| cmd | MoveCommand | 运动指令（详见MoveCommand.msg） |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 插入成功，`false` = 插入失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/job_insert_moveL tl_ros2_interface/srv/JobInsertMove "{
  line: 1,
  cmd: {
    target_pos_value: [
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    ],
    target_pos_name: '',
    target_pos_type: 0,
    coord: 0,
    velocity: 20.0,
    velocity_sync: 0.0,
    acc: 20.0,
    dec: 20.0,
    pl: 0,
    time: 0,
    tool_num: 0,
    user_num: 0,
    posidtype: 0,
    configuration: 0,
    spin: 0,
    para_sync: false
  }
}"
```
### 向作业文件插入一条增量指令IMove
| 功能描述 | 向作业文件插入一条增量指令IMove |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/job_insert_imove` |
| 服务类型 | `tl_ros2_interface/srv/JobInsertMove` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| line | int32 | 插入行序号 |
| cmd | MoveCommand | 运动指令（详见MoveCommand.msg） |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 插入成功，`false` = 插入失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/job_insert_imove tl_ros2_interface/srv/JobInsertMove "{
  line: 1,
  cmd: {
    target_pos_value: [
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    ],
    target_pos_name: '',
    target_pos_type: 0,
    coord: 0,
    velocity: 20.0,
    velocity_sync: 0.0,
    acc: 20.0,
    dec: 20.0,
    pl: 0,
    time: 0,
    tool_num: 0,
    user_num: 0,
    posidtype: 0,
    configuration: 0,
    spin: 0,
    para_sync: false
  }
}"
```
### 向作业文件插入一条moveC关节运动
| 功能描述 | 向作业文件插入一条moveC关节运动 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/job_insert_moveC` |
| 服务类型 | `tl_ros2_interface/srv/JobInsertMove` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| line | int32 | 插入行序号 |
| cmd | MoveCommand | 运动指令（详见MoveCommand.msg） |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 插入成功，`false` = 插入失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/job_insert_moveC tl_ros2_interface/srv/JobInsertMove "{
  line: 1,
  cmd: {
    target_pos_value: [
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    ],
    target_pos_name: '',
    target_pos_type: 0,
    coord: 0,
    velocity: 20.0,
    velocity_sync: 0.0,
    acc: 20.0,
    dec: 20.0,
    pl: 0,
    time: 0,
    tool_num: 0,
    user_num: 0,
    posidtype: 0,
    configuration: 0,
    spin: 0,
    para_sync: false
  }
}"
```
### 运行作业文件
| 功能描述 | 运行作业文件 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/job_run` |
| 服务类型 | `tl_ros2_interface/srv/JobRun` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| job_name | string | 作业名称 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 运行成功，`false` = 运行失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/job_run tl_ros2_interface/srv/JobRun "{job_name: '回零点'}"
```
### 删除作业文件
| 功能描述 | 删除作业文件 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/job_delete` |
| 服务类型 | `tl_ros2_interface/srv/JobRun` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| job_name | string | 作业名称 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 删除成功，`false` = 删除失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/job_delete tl_ros2_interface/srv/JobRun "{job_name: '回零点'}"
```
## 实时运动控制接口
### MoveJ运动控制
| 功能描述 | 机械臂MoveJ运动控制 |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 话题名 | `/tl_driver/moveJ` |
| 消息类型 | `tl_ros2_interface/msg/MoveCommand` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| target_pos_value | float64[] | 目标位姿（14维数组） |
| target_pos_name | string | 目标位姿名称 |
| target_pos_type | int32 | 目标位姿类型 |
| coord | int32 | 坐标系序号 |
| velocity | float64 | 运行速度 |
| velocity_sync | float64 | 速度同步 |
| acc | float64 | 加速度 |
| dec | float64 | 减速度 |
| pl | int32 | 平滑度 |
| time | int32 | 提前执行时间 |
| tool_num | int32 | 工具坐标系编号 |
| user_num | int32 | 用户坐标系编号 |
| posidtype | int32 | 变量类型 |
| configuration | int32 | 形态 |
| spin | int32 | MOVCA指令使用：0-姿态不变 1-六轴不转 2-六轴旋转 |
| para_sync | bool | 外部轴是否同步 |

**输出/返回值**
无返回值（话题发布单向通信）。
#### 命令示例
```
ros2 topic pub --once /tl_driver/moveJ tl_ros2_interface/msg/MoveCommand \
"{
    target_pos_value: [90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    target_pos_name: '',
    target_pos_type: 0,
    coord: 0,
    velocity: 20.0,
    velocity_sync: 0.0,
    acc: 20.0,
    dec: 20.0,
    pl: 0,
    time: 0,
    tool_num: 0,
    user_num: 0,
    posidtype: 0,
    configuration: 0,
    spin: 0,
    para_sync: false
}"
```
### MoveL运动控制
| 功能描述 | 机械臂MoveL运动控制 |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 话题名 | `/tl_driver/moveL` |
| 消息类型 | `tl_ros2_interface/msg/MoveCommand` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| target_pos_value | float64[] | 目标位姿（14维数组） |
| target_pos_name | string | 目标位姿名称 |
| target_pos_type | int32 | 目标位姿类型 |
| coord | int32 | 坐标系序号 |
| velocity | float64 | 运行速度 |
| velocity_sync | float64 | 速度同步 |
| acc | float64 | 加速度 |
| dec | float64 | 减速度 |
| pl | int32 | 平滑度 |
| time | int32 | 提前执行时间 |
| tool_num | int32 | 工具坐标系编号 |
| user_num | int32 | 用户坐标系编号 |
| posidtype | int32 | 变量类型 |
| configuration | int32 | 形态 |
| spin | int32 | MOVCA指令使用：0-姿态不变 1-六轴不转 2-六轴旋转 |
| para_sync | bool | 外部轴是否同步 |

**输出/返回值**
无返回值（话题发布单向通信）。
#### 命令示例
```
ros2 topic pub --once /tl_driver/moveL tl_ros2_interface/msg/MoveCommand \
"{
    target_pos_value: [10.0, 230.0, 245.0, -3.14, 0.0, -1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    target_pos_name: '',
    target_pos_type: 0,
    coord: 1,
    velocity: 20.0,
    velocity_sync: 0.0,
    acc: 20.0,
    dec: 20.0,
    pl: 0,
    time: 0,
    tool_num: 0,
    user_num: 0,
    posidtype: 0,
    configuration: 0,
    spin: 0,
    para_sync: false
}"
```
## 坐标系与工具管理接口
### 设置工具手参数
| 功能描述 | 设置工具手相关参数 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_tool_param` |
| 服务类型 | `tl_ros2_interface/srv/SetToolParam` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| tool_num | int32 | 工具手编号 |
| param | ToolParam | 工具手参数（包含 x/y/z 偏移、姿态、负载等） |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/set_tool_param tl_ros2_interface/srv/SetToolParam \
'{
    tool_num: 1, 
    param: 
    {
        x: 100.0, 
        y: 20.0, 
        z: 50.0, 
        a: 0.1, 
        b: 0.2, 
        c: 0.3, 
        payload_mass: 1.5, 
        payload_inertia: 0.01, 
        payload_mass_center_x: 5.0, 
        payload_mass_center_y: 5.0, 
        payload_mass_center_z: 10.0
    }
}'
```
### 工具手参数标定
| 功能描述 | 工具手参数标定 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/tool_hand_calib` |
| 服务类型 | `tl_ros2_interface/srv/ToolHandCalib` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| tool_num | int32 | 工具坐标系序号 |
| point_num | int32 | 标定点数 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 标定成功，`false` = 标定失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/tool_hand_calib tl_ros2_interface/srv/ToolHandCalib "{tool_num: 3}"
```
### 设置用户坐标系
| 功能描述 | 设置用户坐标系相关参数 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_user_coord` |
| 服务类型 | `tl_ros2_interface/srv/SetUserCoord` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| user_num | int32 | 用户坐标系编号 |
| pos | CartesianPose | 直角坐标系参数（包含位置、欧拉角、臂角） |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/set_user_coord tl_ros2_interface/srv/SetUserCoord \
"{
    user_num: 1, 
    pos: 
    {
        header: 
        {
            frame_id: 'base_link'
        }, 
        position: 
        {
            x: 200.0, 
            y: 100.0, 
            z: 50.0
        }, 
        rpy: 
        {
            x: 0.1, 
            y: 0.2, 
            z: 0.3
        }, 
        arm_angle: 0.0
    }
}"
```
### 设置坐标系编号
| 功能描述 | 设置工具坐标系和用户坐标系编号 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_coord_num` |
| 服务类型 | `tl_ros2_interface/srv/SetCoordNum` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| tool_num | int32 | 工具坐标系序号 |
| user_num | int32 | 用户坐标系序号 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/set_coord_num tl_ros2_interface/srv/SetCoordNum "{tool_num: 1, user_num: 2}"
```
### 设置当前坐标系
| 功能描述 | 设置当前坐标系 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_current_coord` |
| 服务类型 | `tl_ros2_interface/srv/SetCurrentCoord` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| coord | int32 | 坐标系序号：0-关节 1-直角 2-工具 3-用户 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/set_current_coord tl_ros2_interface/srv/SetCurrentCoord "{coord: 1}"
```
## 示教操作接口
### 开始点动
| 功能描述 | 开始点动机械臂 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/start_jogging` |
| 服务类型 | `tl_ros2_interface/srv/Jogging` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| axis | int32 | 关节轴号 |
| direction | bool | 关节运动方向：`true`-正方向 `false`-反方向（注意：direction == 1 和 direction == 0 两种取值） |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 点动成功，`false` = 点动失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/start_jogging tl_ros2_interface/srv/Jogging "{axis: 3, direction: 1}"
```
### 停止点动
| 功能描述 | 停止点动机械臂 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/stop_jogging` |
| 服务类型 | `tl_ros2_interface/srv/Jogging` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| axis | int32 | 关节轴号 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 停止点动成功，`false` = 停止点动失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/stop_jogging tl_ros2_interface/srv/Jogging "{axis: 3}"
```
### 设置拖拽模式
| 功能描述 | 设置机械臂拖拽模式 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_drag_mode` |
| 服务类型 | `tl_ros2_interface/srv/SetDragMode` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| mode | int32 | 拖拽模式：0-无 1-3D鼠标 2-力矩模式 3-位置 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/set_drag_mode tl_ros2_interface/srv/SetDragMode "{mode: 3}"
```
### 查看拖拽状态
| 功能描述 | 查询拖拽是否结束 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_drag_status` |
| 服务类型 | `std_srvs::srv::Trigger` |

**输入参数**
无请求参数（空 `Trigger`）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 拖拽结束，`false` = 拖拽未结束 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/get_drag_status std_srvs/srv/Trigger "{}"
```
### 拖拽轨迹保存
| 功能描述 | 拖拽轨迹保存 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/track_save` |
| 服务类型 | `tl_ros2_interface/srv/TrackSave` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| traj_name | string | 保存轨迹名称 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 保存成功，`false` = 保存失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/track_save tl_ros2_interface/srv/TrackSave "{traj_name: 'traj_test'}"
```
### 拖拽轨迹回放
| 功能描述 | 拖拽轨迹回放 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/track_playback` |
| 服务类型 | `tl_ros2_interface/srv/TrackPlayback` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| vel | int32 | 轨迹回放速度 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 轨迹回放成功，`false` = 轨迹回放失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/track_playback tl_ros2_interface/srv/TrackPlayback "{vel: 20}"
```
## 错误处理与零位标定接口
### 清除错误
| 功能描述 | 清除错误 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/clear_error` |
| 服务类型 | `std_srvs::srv::Trigger` |

**输入参数**
无请求参数（空 `Trigger`）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 清除错误成功，`false` = 清除错误失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/clear_error std_srvs/srv/Trigger "{}"
```
### 设置关节零点
| 功能描述 | 设置关节零点 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_axis_zero_pos` |
| 服务类型 | `tl_ros2_interface/srv/SetAxisZeroPos` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| axis | int32 | 关节轴号 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |

> **注意**：设置关节零点需要先进行`power_off`，完成设置后再`power_on`。
#### 命令示例
```
ros2 service call /tl_driver/set_axis_zero_pos tl_ros2_interface/srv/SetAxisZeroPos "{axis: 1}"
```
## 全局路点管理接口
### 查询全局位点
| 功能描述 | 查询全局位点 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_global_pos` |
| 服务类型 | `tl_ros2_interface/srv/GetGlobalPos` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| pos_name | string | 位点名称 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 失败时包含错误描述 |
| pos_info | float64[] | 位点信息 |
#### 命令示例
```
ros2 service call /tl_driver/get_global_pos tl_ros2_interface/srv/GetGlobalPos "{pos_name: 'GP0002'}"
```
### 设置全局位点
| 功能描述 | 设置全局位点 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_global_pos` |
| 服务类型 | `tl_ros2_interface/srv/SetGlobalPos` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| pos_name | string | 位点名称 |
| pos_info | float64[] | 位点信息 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/set_global_pos tl_ros2_interface/srv/SetGlobalPos \
'{
    pos_name: "GP0002", 
    pos_info: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 57.14, -32.93, 19.74, -89.89, -19.77, 0.0]
}'
```
## 模式与IO控制接口
### 设置当前运行模式
| 功能描述 | 设置当前运行模式 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_current_mode` |
| 服务类型 | `tl_ros2_interface/srv/SetCurrentMode` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| mode | int32 | 模式序号：0-示教 1-远程 2-运行 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/set_current_mode tl_ros2_interface/srv/SetCurrentMode "{mode: 2}"
```
### 查询当前运行模式
| 功能描述 | 查询当前运行模式 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_current_mode` |
| 服务类型 | `tl_ros2_interface/srv/GetCurrentMode` |

**输入参数**
无请求参数。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 失败时包含错误描述 |
| mode | int32 | 当前模式序号：0-示教 1-远程 2-运行 |
#### 命令示例
```
ros2 service call /tl_driver/get_current_mode tl_ros2_interface/srv/GetCurrentMode "{}"
```
### 设置数字输出
| 功能描述 | 设置数字输出 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/set_digital_output` |
| 服务类型 | `tl_ros2_interface/srv/SetDigitalOutput` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| port | int32 | 端口号 |
| value | int32 | 端口输出状态：0或1 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/set_digital_output tl_ros2_interface/srv/SetDigitalOutput "{port: 1, value: 1}"
```
### 查询数字输入输出状态
| 功能描述 | 查看数字输入输出状态 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_digital_input_output` |
| 服务类型 | `tl_ros2_interface/srv/GetDigitalInputOutput` |

**输入参数**
无请求参数。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 查询成功，`false` = 查询失败 |
| message | string | 失败时包含错误描述 |
| input | int32[] | 数字输入状态 |
| output | int32[] | 数字输出状态 |
#### 命令示例
```
ros2 service call /tl_driver/get_digital_input_output tl_ros2_interface/srv/GetDigitalInputOutput "{}"
```
## modbus通信接口
### 写Modbus
| 功能描述 | Modbus数据写入 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/modbus_write` |
| 服务类型 | `tl_ros2_interface/srv/ModbusWrite` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| master_id | int32 | 主站ID |
| addr | int32 | 主站地址 |
| data | int32[] | 写入数据 |
| master_param | ModbusMasterParam | Modbus主站参数（含 type/start_addr，以及RTU或TCP子参数） |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 写入成功，`false` = 写入失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/modbus_write tl_ros2_interface/srv/ModbusWrite \
"{
    master_id: 1,
    addr: 1135,
    data: [0, 0, 0, 0, 0],
    master_param: 
    {
        type: 'RTU',
        start_addr: true,
        rtu: 
        {
            slave_id: 2,
            port: 2,
            baudrate: 115200,
            data_bit: 8,
            stop_bit: 1,
            check_bit: 'N'
        }
    }
}"
```
### 读Modbus
| 功能描述 | Modbus数据读取 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/modbus_read` |
| 服务类型 | `tl_ros2_interface/srv/ModbusRead` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| master_id | int32 | 主站ID |
| addr | int32 | 主站地址 |
| quantity | int32 | 读取数量 |
| master_param | ModbusMasterParam | Modbus主站参数（其它参数可参考写Modbus部分） |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 读取成功，`false` = 读取失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/modbus_read tl_ros2_interface/srv/ModbusRead \
"{
    master_id: 1,
    addr: 1135,
    quantity: 5,
    master_param: 
    {
        type: 'RTU',
        start_addr: true,
        rtu: 
        {
            slave_id: 2,
            port: 2,
            baudrate: 115200,
            data_bit: 8,
            stop_bit: 1,
            check_bit: 'N'
        }
    }
}"
```
## 队列运动接口
### 设置MoveJ队列运动模式
| 功能描述 | 设置MoveJ队列运动模式 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/queue_motion_set_status` |
| 服务类型 | `tl_ros2_interface/srv/QueueMotionSetStatus` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| status | bool | 队列运动模型状态开关：`true`-打开 `false`-关闭 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 设置成功，`false` = 设置失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/queue_motion_set_status tl_ros2_interface/srv/QueueMotionSetStatus "{status: true}"
```
### MoveJ队列运动
| 功能描述 | MoveJ队列运动 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/queue_motion_movej` |
| 服务类型 | `tl_ros2_interface/srv/QueueMotionMoveJ` |

**输入参数**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| is_continue | bool | 是否连续运动 |
| cmd | MoveCommand | 运动控制命令参数（详见MoveCommand.msg） |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 运动成功，`false` = 运动失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/queue_motion_movej tl_ros2_interface/srv/QueueMotionMoveJ \
"{
    is_continue: false,
    cmd: 
    {
        target_pos_value: [0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0], 
        velocity: 20.0, 
        acc: 20.0, 
        dec: 20.0
    }
}"
```
### 停止MoveJ队列运动模式
| 功能描述 | 停止MoveJ队列运动模式 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/queue_motion_stop` |
| 服务类型 | `std_srvs::srv::Trigger` |

**输入参数**
无请求参数（空 `Trigger`）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 停止成功，`false` = 停止失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/queue_motion_stop std_srvs/srv/Trigger "{}"
```
## 7000端口
### 打开关节跟踪模式
| 功能描述 | 打开关节跟踪模式 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/open_servoj` |
| 服务类型 | `tl_ros2_interface/srv/OpenServoJ` |

**输入参数**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| vmax | float64[] | °/s | 各轴最大速度，数组长度 ≥ 关节数 |
| amax | float64[] | °/s² | 各轴最大加速度 |
| jmax | float64[] | °/s³ | 各轴最大加加速度 |

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 打开成功，`false` = 打开失败 |
| message | string | 失败时包含错误描述 |

> **注意**：该模式只能在连接端口7000后使用。使用前建议先通过 [设置运行速度](#设置运行速度) 增大机械臂运行速度，如果运行速度太小会出现关节不动或运行缓慢的情况。
#### 命令示例
```
ros2 service call /tl_driver/open_servoj tl_ros2_interface/srv/OpenServoJ \
"{
    vmax: [300, 300, 300, 300, 300, 300, 300],
    amax: [3000, 3000, 3000, 3000, 3000, 3000, 3000],
    jmax: [50000, 50000, 50000, 50000, 50000, 50000, 50000]
}"
```
* 注意: 该模式只能在连接端口7000后使用。在使用该功能前最好先增加机械臂运行速度，如果运行速度太小的话会出现关节不动或者关节运行较慢的情况。
### 关闭关节跟踪模式
| 功能描述 | 关闭关节跟踪模式 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/close_servoj` |
| 服务类型 | `std_srvs::srv::Trigger` |

**输入参数**
无请求参数（空 `Trigger`）。

**输出/返回值**
| 参数名 | 类型 | 说明 |
|--------|------|------|
| success | bool | `true` = 关闭成功，`false` = 关闭失败 |
| message | string | 失败时包含错误描述 |
#### 命令示例
```
ros2 service call /tl_driver/close_servoj std_srvs/srv/Trigger "{}"
```
### 发送跟踪关节位置
| 功能描述 | 发送跟踪关节位置 |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 话题名 | `/tl_driver/set_servoj_pos` |
| 消息类型 | `std_msgs::msg::Float64MultiArray` |

**输入参数**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| data | float64[] | ° | 目标关节角度数组（7维，前6轴角度，第7轴补0） |

**输出/返回值**
无返回值（话题发布单向通信）。

> **注意**：该话题仅在 [打开关节跟踪模式](#打开关节跟踪模式) 后有效。需以固定周期持续调用以维持跟踪。
#### 命令示例
```
ros2 topic pub /tl_driver/set_servoj_pos std_msgs/msg/Float64MultiArray \ 
"{
    layout: 
    {
        dim: [], 
        data_offset: 0
    }, 
    data: [0.0, 0.0, 0.0, 0.0, 0.0, 20.0, 0.0]
}"
```
### 发送跟踪笛卡尔位姿
| 功能描述 | 笛卡尔空间直线伺服运动：在当前位姿与目标位姿之间插入多个中间点，通过 IK 转为关节角后经 servoj 通道高频发送，实现笛卡尔直线轨迹的实时伺服控制（单向话题，无返回值） |
| :---: | :---- |
| 通信机制 | ROS2话题 |
| 话题名 | `/tl_driver/set_servol_pos` |
| 消息类型 | `tl_ros2_interface/msg/ServolMove` |

**输入参数**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| target_pose | float64[] | mm / rad | 目标笛卡尔位姿 [x, y, z, rx, ry, rz]（6维） |
| step_size | float64 | mm | 插值步长，相邻插值点之间的最大笛卡尔距离，默认为 5.0 |
| coord | int32 | — | 坐标系：1-基座标系(Base) 2-工具坐标系(Tool) 3-用户坐标系(User) |

> **注意**：该话题需在 [打开关节跟踪模式](#打开关节跟踪模式) 后使用。内部使用四元数 Slerp 进行姿态插值，100Hz 频率发送关节角。收到话题消息后节点自动执行插值运动，执行过程中节点日志会显示插值点数。

**算法流程说明**
```
1. 获取当前笛卡尔位姿
2. 根据 step_size 计算插值点数 N = ceil(距离 / step_size)
3. 循环 N 个插值点：
   a. 位置分量线性插值 (x, y, z)
   b. 姿态分量 rpy → 四元数 → Slerp → rpy
   c. 插值位姿经坐标转换接口做 IK（笛卡尔 → 关节）
   d. 通过 set_servoJ_pos 发送关节角到 7000 端口
   e. accumulative timing 维持 100Hz 发送频率
4. 运动完成，记录日志
```
#### 命令示例
```
ros2 topic pub --once /tl_driver/set_servol_pos tl_ros2_interface/msg/ServolMove \
"{
    target_pose: [327.47, 0.0, 395.44, 3.14, 0.0, 0.0],
    step_size: 5.0,
    coord: 1
}"
```
## 位姿转换工具接口
### 四元数转欧拉角
| 功能描述 | 四元数转欧拉角 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_quat2rpy` |
| 服务类型 | `tl_ros2_interface/srv/GetPosTransform` |

**输入参数**
| 参数名 | 类型 | 长度 | 说明 |
|--------|------|------|------|
| input | float64[] | 4 | 四元数 [w, x, y, z]（无单位） |

**输出/返回值**
| 参数名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| success | bool | — | `true` = 转换成功，`false` = 转换失败 |
| message | string | — | 失败时包含错误描述 |
| output | float64[] | rad | 欧拉角 [rx, ry, rz] |
#### 命令示例
```
ros2 service call /tl_driver/get_quat2rpy tl_ros2_interface/srv/GetPosTransform \
"{
    input: [-0.080, 0.919, 0.365, 0.122]
}"
```
### 欧拉角转四元数
| 功能描述 | 欧拉角转四元数 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_rpy2quat` |
| 服务类型 | `tl_ros2_interface/srv/GetPosTransform` |

**输入参数**
| 参数名 | 类型 | 单位 | 长度 | 说明 |
|--------|------|------|------|------|
| input | float64[] | rad | 3 | 欧拉角 [rx, ry, rz] |

**输出/返回值**
| 参数名 | 类型 | 长度 | 说明 |
|--------|------|------|------|
| success | bool | — | `true` = 转换成功，`false` = 转换失败 |
| message | string | — | 失败时包含错误描述 |
| output | float64[] | 4 | 四元数 [w, x, y, z]（无单位） |
#### 命令示例
```
ros2 service call /tl_driver/get_rpy2quat tl_ros2_interface/srv/GetPosTransform \
"{
    input: [-2.9, 0.167, -0.777]
}"
```
### 欧拉角转旋转矩阵
| 功能描述 | 欧拉角转旋转矩阵 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_rpy2r` |
| 服务类型 | `tl_ros2_interface/srv/GetPosTransform` |

**输入参数**
| 参数名 | 类型 | 单位 | 长度 | 说明 |
|--------|------|------|------|------|
| input | float64[] | rad | 3 | 欧拉角 [rx, ry, rz] |

**输出/返回值**
| 参数名 | 类型 | 长度 | 说明 |
|--------|------|------|------|
| success | bool | — | `true` = 转换成功，`false` = 转换失败 |
| message | string | — | 失败时包含错误描述 |
| output | float64[] | 9 | 3×3旋转矩阵（行主序，无单位） |
#### 命令示例
```
ros2 service call /tl_driver/get_rpy2r tl_ros2_interface/srv/GetPosTransform \
"{
    input: [-2.9, 0.167, -0.777]
}"
```
### 位姿转旋转矩阵
| 功能描述 | 位姿转旋转矩阵 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_tr2r` |
| 服务类型 | `tl_ros2_interface/srv/GetPosTransform` |

**输入参数**
| 参数名 | 类型 | 长度 | 说明 |
|--------|------|------|------|
| input | float64[] | 16 | 4×4齐次变换矩阵（行主序，无单位） |

**输出/返回值**
| 参数名 | 类型 | 长度 | 说明 |
|--------|------|------|------|
| success | bool | — | `true` = 转换成功，`false` = 转换失败 |
| message | string | — | 失败时包含错误描述 |
| output | float64[] | 9 | 3×3旋转矩阵（行主序，无单位） |
#### 命令示例
```
ros2 service call /tl_driver/get_tr2r tl_ros2_interface/srv/GetPosTransform \
"{
    input: [0.703, 0.691, 0.166, 0.000, 0.652, -0.720, 0.236, 0.000, 0.283, -0.057, -0.957, 0.000, 0.000, 0.000, 0.000, 1.000]
}"
```
### 旋转矩阵转位姿
| 功能描述 | 旋转矩阵转位姿 |
| :---: | :---- |
| 通信机制 | ROS2服务 |
| 服务名 | `/tl_driver/get_r2tr` |
| 服务类型 | `tl_ros2_interface/srv/GetPosTransform` |

**输入参数**
| 参数名 | 类型 | 长度 | 说明 |
|--------|------|------|------|
| input | float64[] | 9 | 3×3旋转矩阵（行主序，无单位） |

**输出/返回值**
| 参数名 | 类型 | 长度 | 说明 |
|--------|------|------|------|
| success | bool | — | `true` = 转换成功，`false` = 转换失败 |
| message | string | — | 失败时包含错误描述 |
| output | float64[] | 16 | 4×4齐次变换矩阵（行主序，无单位） |
#### 命令示例
```
ros2 service call /tl_driver/get_r2tr tl_ros2_interface/srv/GetPosTransform \
"{
    input: [0.703, 0.691, 0.166, 0.652, -0.720, 0.236, 0.283, -0.057, -0.957]
}"
```

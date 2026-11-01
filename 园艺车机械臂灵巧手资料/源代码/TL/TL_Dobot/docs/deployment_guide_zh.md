# TCB605 + ROH-LiteS001 独立部署指南

本文对应 `TL_Dobot` 工程。目标是先建立可靠的底层闭环，再接入 D435 检测和抓取逻辑。当前工程不包含 VLA 推理，也不应在底层保护尚未通过时直接部署 VLA。

## 1. 手眼标定时是否安装灵巧手

可以先不安装灵巧手。

只要 D435 支架已经刚性固定在机械臂法兰上，采集时记录的是 `tool0/flange` 位姿，那么不安装灵巧手不会影响 `T_tool0_camera`。安装灵巧手后必须：

- 测量灵巧手实际抓取中心相对法兰的 TCP 位姿。
- 设置工具总质量、惯量和重心，包含灵巧手、D435、支架和线缆。
- 将 `T_tool0_camera` 转换成抓取 TCP 对应的 `T_tool_camera`，或者安装后重新标定。
- D435 支架只要拆装、碰撞或被线缆拉动，就重新标定。

无手标定结果的语义为：

```text
p_tool0 = T_tool0_camera * p_camera
T_base_camera = T_base_tool0 * T_tool0_camera
```

## 2. 工程结构与数据流

```text
TCB605 厂家 ROS2 驱动
        │  /joint_states, /tcp_pose, /tl_driver/modbus_*
        ├── tl_safe_arm       机械臂运动安全代理
        ├── tl_hand_guard     灵巧手保护控制
        └── tl_handeye_collect / tl_handeye_calibrate
                         │
                    D435 RGB-D
```

自动运动必须发布到 `/tl_safe/moveJ` 或 `/tl_safe/moveL`。不要让自动程序直接发布 `/tl_driver/moveJ`、`/tl_driver/moveL`，否则会绕过代理检查。

主要文件：

- `config/arm_safety.yaml`：关节、工作空间、单步运动和速度限制。
- `config/hand.yaml`：ROH Modbus、位置、速度、电流、力和堵转保护。
- `config/handeye.yaml`：D435 分辨率、棋盘格和采集稳定条件。
- `safe_arm_node.py`：运动安全代理。
- `roh_hand_node.py`：灵巧手保护状态机。
- `handeye_collect_node.py`：无主动运动的数据采集器。
- `handeye_calibrate.py`：手眼求解和质量门槛。

## 3. Jetson 文件和依赖准备

把整个 Windows `TL_Dobot` 目录复制到 Jetson：

```text
~/Desktop/TL_Dobot
```

同时保留唯一一份新版天链驱动：

```text
~/Desktop/tl_robot_ros2_py-arm
```

工程不依赖 GitHub，可通过 U 盘、移动硬盘或局域网 SCP 传输。安装基础包：

```bash
sudo apt update
sudo apt install python3-opencv python3-yaml python3-numpy python3-pytest
```

D435 已经能通过 `pyrealsense2` 采集 640x480 RGB 时，不要重复安装相机包，也不要替换 Jetson 的 CUDA、PyTorch 或 NumPy 环境。

## 4. 编译

先编译天链厂家驱动：

```bash
source /opt/ros/humble/setup.bash
cd ~/Desktop/tl_robot_ros2_py-arm
colcon build --packages-select tl_ros2_interface
colcon build --symlink-install
source install/setup.bash
```

编译本工程：

```bash
cd ~/Desktop/TL_Dobot
export TL_DRIVER_DIR=~/Desktop/tl_robot_ros2_py-arm
source /opt/ros/humble/setup.bash
source "$TL_DRIVER_DIR/install/setup.bash"
colcon build --symlink-install
source install/setup.bash
chmod +x scripts/*.sh
./scripts/run_static_checks.sh
```

## 5. 第一阶段：只接机械臂和 D435

### 5.1 硬件条件

- D435 支架刚性连接在法兰或法兰转接板上。
- 棋盘格固定在工作台，整个采集过程中不能移动。
- 当前配置是 10x7 内角点、15 mm 格子。必须用卡尺确认实际打印尺寸，打印机不能“适合页面”。
- 不安装灵巧手，不让相机线缆拖动支架。
- 清空机械臂工作区，实体急停放在手边。

### 5.2 启动厂家驱动

终端 1：

```bash
cd ~/Desktop/tl_robot_ros2_py-arm
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tl_bringup tl_tcb605_bringup.launch.py
```

资料默认机械臂地址为 `192.168.1.13:6001`，以 `src/tl_driver/config/tl_tcb605_config.yaml` 和真机为准。

终端 2：

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
cd ~/Desktop/TL_Dobot
source install/setup.bash
ros2 run tl_hand_deploy tl_system_check --skip-camera
```

必须看到 `/joint_states` 和 `/tcp_pose` 为 `OK`。当前天链驱动的 `/joint_states` 是角度制，`/tcp_pose` 位置是毫米、RPY 是弧度；本工程已经按该单位处理。

### 5.3 确认 tool0

当前 ARM64 驱动没有注册 `/tl_driver/set_coord_num`，不能调用该服务。手眼采集前先查询当前工具和用户坐标编号：

```bash
timeout 5s ros2 service call /tl_driver/get_coord_num \
  tl_ros2_interface/srv/GetCoordNum "{}"
```

只有返回 `tool_num: 0`、`user_num: 0` 才继续。否则停止采集，通过控制器配置或补充驱动接口切换到 `tool0/user0`。随后设置并查询当前坐标类型：

```bash
timeout 5s ros2 service call /tl_driver/set_current_coord \
  tl_ros2_interface/srv/SetCurrentCoord "{coord: 1}"
timeout 5s ros2 service call /tl_driver/get_current_coord \
  tl_ros2_interface/srv/GetCurrentCoord "{}"
```

采集程序不会发送 ServoJ。拖动时按住末端实体 `P` 键，移动到位后松开。

### 5.4 采集

终端 3：

```bash
cd ~/Desktop/TL_Dobot
source /opt/ros/humble/setup.bash
source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
source install/setup.bash
ros2 launch tl_hand_deploy handeye_collect.launch.py
```

每组数据按以下顺序采集：

1. 棋盘格完整出现在 D435 彩色图像中。
2. 按住 `P` 拖到新姿态。
3. 松开 `P` 并静止至少 1 秒。
4. 按 `s` 保存；程序检查角点、位姿新鲜度、关节稳定性和姿态重复性。
5. 重复 25 到 35 组，至少不得少于 20 组。
6. 按 `q` 退出。

必须覆盖图像中心和四角、多个观察距离、绕至少两个轴的明显旋转。不能只做平移，也不能所有图片都正视棋盘格。避开机械臂奇异位形、关节极限和电缆拉紧位置。

### 5.5 求解与验收

```bash
DATASET=$(find ~/tl_handeye_data -maxdepth 1 -type d -name 'session_*' | sort | tail -n 1)
ros2 run tl_hand_deploy tl_handeye_calibrate "$DATASET"
```

求解器会尝试 `intrinsic_xyz`、`extrinsic_xyz` 和 Tsai、Park、Horaud、Andreff、Daniilidis 方法。它根据固定标定板在基座坐标中的一致性选择结果。

通过后生成：

```text
calibration/handeye_accepted.yaml
calibration/report.json
```

质量条件：

- `accepted: true`
- `camera_reprojection_rms_px <= 1.0`
- `translation_rms_mm <= 8.0`
- `rotation_rms_deg <= 2.0`
- `motion_translation_span_mm >= 80`
- `motion_rotation_span_deg >= 20`

如果只生成 `handeye_candidate_rejected.yaml`，禁止用于自动抓取。重新采集时优先增加旋转方向和图像边缘样本。

## 6. 第二阶段：安装灵巧手

### 6.1 TCP、质量和重心

安装后先不自动抓取。下面只展示服务结构，所有数值必须换成实测值：

```bash
ros2 service call /tl_driver/set_tool_param tl_ros2_interface/srv/SetToolParam \
"{tool_num: 1, param: {
  x: 0.0, y: 0.0, z: 0.0,
  a: 0.0, b: 0.0, c: 0.0,
  payload_mass: 0.486,
  payload_inertia: 0.0,
  payload_mass_center_x: 0.0,
  payload_mass_center_y: 0.0,
  payload_mass_center_z: 0.0
}}"
```

`0.486 kg` 只是 ROH-LiteS001 手册参考重量，不能代替灵巧手、D435、支架、螺栓和运动线缆的总重量。惯量和重心应由 CAD 或称重测量得到。

### 6.2 把法兰手眼矩阵转换到抓取 TCP

测得法兰到抓取 TCP 的 `[x,y,z,rx,ry,rz]` 后：

```bash
ros2 run tl_hand_deploy tl_convert_handeye_frame \
  ~/tl_handeye_data/session_xxx/calibration/handeye_accepted.yaml \
  --flange-to-tool X_MM Y_MM Z_MM RX_RAD RY_RAD RZ_RAD \
  --output ~/Desktop/TL_Dobot/config/handeye_tool.yaml
```

输出语义为 `p_tool = T_tool_camera * p_camera`。如果 TCP 或支架测量不可靠，安装灵巧手后重新做一次完整标定更稳妥。

### 6.3 灵巧手保护控制

使用的关键寄存器：

- 状态 `1085-1090`
- 电流限制和实际电流 `1095-1110`
- 力限制和实际力 `1115-1124`
- 速度 `1125-1130`
- 目标与实际位置 `1135-1150`
- 堵转保护 `1175-1210`

状态为 `0=展开中，1=抓取中，2=位置到位，3=过流停止，4=达到力限制，5=堵转停止`。本工程只把 `4` 作为接触候选，把 `3` 和 `5` 都按故障处理：冻结当前位置、向张开方向释放一小段、锁定后续命令。

先只读启动：

```bash
ros2 launch tl_hand_deploy hand_guard.launch.py
ros2 topic echo /tl_hand/status
```

初始 `limits_verified: false`，因此任何动作请求都会被拒绝。完成空载、小幅度、软物体和异常停止测试后，才在 `src/tl_hand_deploy/config/hand.yaml` 中改为：

```yaml
limits_verified: true
```

重新启动并执行：

```bash
ros2 service call /tl_hand/set_enabled std_srvs/srv/SetBool "{data: true}"
ros2 service call /tl_hand/open std_srvs/srv/Trigger "{}"
```

第一次抓取必须使用泡沫或海绵。手指目标从小值逐步增加，不直接使用 `FIST=60000`，也不把手册最大负载当成安全抓取力。

## 7. 机械臂安全代理

启动：

```bash
ros2 launch tl_hand_deploy safety_only.launch.py
ros2 topic echo /tl_safe/status
```

清空工作区、确认实体急停后解锁：

```bash
ros2 service call /tl_safe/set_enabled std_srvs/srv/SetBool "{data: true}"
```

命令工具默认只打印目标，必须显式加 `--execute` 才发布：

```bash
ros2 run tl_hand_deploy tl_safe_move joint 0 0 0 0 0 0
ros2 run tl_hand_deploy tl_safe_move joint 0 0 0 0 0 0 --execute
```

代理检查关节限位、单步关节变化、笛卡尔工作区、单步平移、姿态变化、速度和加速度。当前限位是工程保守配置，不是厂家完整碰撞模型；真机调试前必须按实际安装环境缩小工作区。

```bash
ros2 service call /tl_safe/stop std_srvs/srv/Trigger "{}"
```

`/tl_safe/stop` 只能阻止后续命令，不能保证中断厂家驱动中已经执行的 MoveJ/MoveL。危险运动必须按实体急停。

## 8. 验收顺序

1. 机械臂空载，实体急停测试通过。
2. `/joint_states`、`/tcp_pose` 连续更新。
3. D435 RGB 和深度稳定，无 USB 掉线。
4. 无手完成手眼标定并通过残差门槛。
5. 安装灵巧手，设置 TCP、质量、惯量和重心。
6. 灵巧手只读通信正常，空载张开和小幅动作正常。
7. 软物体验证力限制、过流和堵转释放。
8. 安全代理完成小步 MoveJ、MoveL 测试。
9. 最后实现 `检测 -> 预抓取 -> 低速接近 -> 闭合 -> 抬升验证 -> 转运 -> 放置 -> 撤离` 状态机。

## 9. 明确禁止项

- 不在标定采集时同时启动自动抓取。
- 不同时 source 多份旧版 `tl_robot_ros2_py-arm`。
- 不把无手的 `T_tool0_camera` 直接当作灵巧手抓取 TCP 的矩阵。
- 不绕过安全代理向厂家运动话题发送自动命令。
- 不把 `STATUS_STUCK=5` 直接判为抓取成功。
- 不修改天链预编译 `_tl_host.so`。
- 不刷写资料中的 ROH-A001/A002 固件到 ROH-LiteS001。

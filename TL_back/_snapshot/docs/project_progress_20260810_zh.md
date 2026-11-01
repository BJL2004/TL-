# TL_Dobot 项目进度说明

更新时间：2026-08-10

本文档记录当前“天链机械臂 + D435 + 假手/后续傲意灵巧手”部署工程已经完成的工作、当前状态、已知问题和下一步操作。

## 1. 当前目标

当前阶段不是直接上真实傲意灵巧手抓取，而是先跑通以下链路：

```text
D435 识别/点击目标
-> 手眼矩阵转换到 base 坐标
-> 加入 7 点均值 XY 粗补偿
-> 根据假手抓取中心反算 tool0 位姿
-> 使用天链 CoordTransform 求 IK
-> 使用 500/1000 点 ServoJ 平滑运动
-> 回到检测位
```

真实灵巧手暂时不安装，先用假手验证“抓取中心能否对准目标中心”。

## 2. 已完成工作

### 2.1 基础驱动与系统检查

已确认天链驱动工作正常：

- `/joint_states` 可读
- `/tcp_pose` 可读
- `/tl_driver/coord_transform` 可用
- `/tl_driver/open_servoj` 可用
- `/tl_driver/set_servoj_pos` 可用

已确认 D435 相机可被 `pyrealsense2` 打开。

### 2.2 手眼标定链路

已完成一套 D435 眼在手上的手眼采集和求解流程：

- 采集程序：`tl_handeye_collect`
- 标定程序：`tl_handeye_calibrate`
- 当前使用标定文件：

```bash
~/tl_handeye_data/session_20260808_022900/calibration/handeye_candidate_rejected.yaml
```

注意：该标定文件状态是 `accepted=False`，质量门槛未通过。因此当前只能用于粗略测试，不能作为最终真实抓取标定。

### 2.3 无手俯视到点测试

无手俯视模式已经跑通，程序能够完成：

```text
点击目标 -> 计算目标 -> ServoJ 到目标附近 -> 回检测位
```

当前使用 7 点实测均值作为临时 XY 补偿：

```text
base XY bias = [-20.7, +5.3] mm
```

在命令中使用：

```bash
TL_BASE_XY_BIAS_MM="-20.7 5.3"
```

该补偿只适合当前俯视测试姿态和当前相机安装状态。

### 2.4 假手虚拟 TCP 模式

已实现假手虚拟 TCP 模式：

```bash
--mock-grasp-offset-mm X Y Z
--mock-grasp-clearance-mm C
--grasp-rpy-rad RX RY RZ
```

含义：

- `mock-grasp-offset-mm`：tool0 原点到假手抓取中心的偏移，单位 mm，坐标系为 tool0。
- `mock-grasp-clearance-mm`：假手抓取中心距离视觉目标点上方的安全高度。
- `grasp-rpy-rad`：抓取时 tool0 的固定姿态。

### 2.5 平抓自动抬高

最新增加了“平抓自动抬高到可达高度”的逻辑：

- 如果平抓目标的当前 Z 不可达，程序会优先在同 XY 上向上探测。
- 找到第一个可达高度后，会直接使用该高度继续生成 IK。
- 这只会把目标抬高，不会让机械臂向物体更近。

## 3. 关键问题解释

### 3.1 为什么“平抓”不是只调 J5

平抓不是单独把 J5 转过去。原因是：

- 假手抓取中心不在 tool0 原点上。
- J5 一转，假手抓取中心会绕腕部转动。
- 要让“假手掌心/抓取中心”落到物体中心，必须同时求解 J1-J6。

所以正确做法是：

```text
目标物体中心
-> 加安全高度
-> 加假手抓取中心偏移
-> 固定平抓 RPY
-> 求完整 6 轴 IK
```

### 3.2 为什么之前显示无法移动

最近一次输出中，IK 已经返回了关节解：

```text
J5 = -177.7 deg 或 -169.7 deg
```

但程序原本软件限位写的是：

```text
J5: -130 deg .. +130 deg
```

而用户手动拖到平抓位时截图显示：

```text
J5 = -188.116 deg
```

因此该位置机械臂物理上可以到达，程序之前是被过于保守的软件 J5 限位误拦。

### 3.3 本次针对平抓做的修改

已将测试程序中的 J5 软件检查范围调整为：

```text
J5: -210 deg .. +210 deg
```

已让启动脚本支持：

```bash
TL_MAX_JOINT_DELTA_DEG
```

假手平抓脚本默认：

```bash
TL_MAX_JOINT_DELTA_DEG=220
```

这是为了允许从检测位 `J5≈0 deg` 运动到平抓位 `J5≈-170 deg`。

## 4. 当前假手平抓参数

根据手动拖动到目标附近的平抓截图，当前使用的平抓姿态为：

```text
RPY = [-2.043, 1.471, -1.025] rad
```

当前假手抓取中心初始偏移为：

```text
TL_MOCK_GRASP_OFFSET_MM="-6.3 34.0 42.5"
```

当前建议先用较高安全高度：

```text
TL_MOCK_GRASP_CLEARANCE_MM=130
```

等方向和可达性稳定后，再逐步降低：

```bash
TL_MOCK_GRASP_CLEARANCE_MM=110 ./scripts/run_mock_hand_side_grasp_test.sh
TL_MOCK_GRASP_CLEARANCE_MM=90  ./scripts/run_mock_hand_side_grasp_test.sh
TL_MOCK_GRASP_CLEARANCE_MM=70  ./scripts/run_mock_hand_side_grasp_test.sh
```

不要一开始直接降到 30 mm。

最新脚本还支持到达高位后的 Z 方向微调：

```text
j：tool0 目标向下 5 mm
u：tool0 目标向上 5 mm
```

每按一次都会先重新做 IK 检查；如果当前高度不可达，程序会打印 `Command rejected/stopped`，不会发送运动。需要更细时可以这样启动：

```bash
TL_FINE_Z_STEP_MM=2 ./scripts/run_mock_hand_side_grasp_test.sh
```

当前阶段建议先让假手在 130 mm 高安全距离到位，再用 `j` 一次下降 5 mm，观察假手方向和离物体距离，确认无碰撞后再继续下降。

## 5. 需要复制到 Nano 的文件

最少复制以下三个文件：

```text
C:\Users\DragonAfterAll\Desktop\TL_Dobot\src\tl_hand_deploy\tl_hand_deploy\overhead_test.py
C:\Users\DragonAfterAll\Desktop\TL_Dobot\scripts\run_no_gripper_hover_test.sh
C:\Users\DragonAfterAll\Desktop\TL_Dobot\scripts\run_mock_hand_side_grasp_test.sh
```

如果担心文件没同步，直接复制整个：

```text
C:\Users\DragonAfterAll\Desktop\TL_Dobot
```

到 Nano：

```text
/home/nvidia/Desktop/TL_Dobot
```

## 6. Nano 上下一步操作

复制完成后执行：

```bash
cd ~/Desktop/TL_Dobot
source /opt/ros/humble/setup.bash
source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash

colcon build --symlink-install --packages-select tl_hand_deploy

chmod +x scripts/run_no_gripper_hover_test.sh
chmod +x scripts/run_mock_hand_side_grasp_test.sh

./scripts/run_mock_hand_side_grasp_test.sh
```

窗口出现后：

```text
点击目标中心
按 c 计算
确认出现 IK joints
确认没有 Command rejected/stopped
确认路径安全
按 m 运动
如果需要更低，按 j 单步下降；如果太低，按 u 单步升高
按 r 返回检测位
按 q 退出
```

## 7. 运行时重点看什么

必须确认以下输出：

```text
Mock-hand mode
fixed grasp RPY[rad]=[-2.043, 1.471, -1.025]
base XY bias[mm]=[-20.7, 5.3]
mock grasp base[mm]=...
tool0 target TCP=[..., ..., ..., -2.043, 1.471, -1.025]
IK joints[deg]=...
```

如果仍出现：

```text
IK joints outside configured limits
```

说明 Nano 上没有复制最新版 `overhead_test.py`，或者没有重新 build/source。

如果出现：

```text
CoordTransform has no IK
```

说明当前目标点、平抓姿态和高度组合在驱动 IK 分支下不可达。优先提高：

```bash
TL_MOCK_GRASP_CLEARANCE_MM
```

或者把目标物体放到更靠近机械臂工作区中心的位置。

## 8. 当前项目进度判断

当前状态：

```text
驱动通信：已跑通
D435 取图：已跑通
手眼采集：已跑通
手眼精度：未最终通过，只能粗用
无手俯视到点：已跑通
7 点 XY 粗补偿：已接入并实测有效
假手虚拟 TCP：已接入
假手平抓姿态：已接入
假手平抓运动：正在验证，最新问题是软件限位误拦，已修正
真实傲意灵巧手：尚未进入真实闭合测试
```

下一阶段目标：

```text
1. 复制最新版文件到 Nano
2. 使用平抓脚本确认假手能按平抓姿态运动
3. 用高 clearance 测方向
4. 逐步降低 clearance
5. 做假手九点误差记录
6. 修正假手 TCP 偏移
7. 稳定后再安装真实傲意灵巧手
8. 给灵巧手加限力、限速、接触保护，再做真实抓取
```

## 9. 重要安全注意事项

- 当前程序直接使用 ServoJ，不经过安全代理。
- 运动前必须保证急停可触达。
- 假手测试阶段不要让假手接触硬物。
- 初次平抓测试使用 `130 mm` 高安全距离。
- 只有确认方向正确后，才逐步降低高度。
- 真实傲意灵巧手安装前，不要把假手参数直接当作真实手参数使用。
- 真实灵巧手需要重新测量抓取中心相对 tool0 的偏移。

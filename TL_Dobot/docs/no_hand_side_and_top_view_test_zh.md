# 无手平视/俯视检测到点测试

本阶段不安装灵巧手，也不使用假手。目标是验证同一套 D435 手眼矩阵在两种机械臂观察姿态下能否完成：

```text
相机检测目标 -> 目标点转 base -> tool0 移动到目标上方/附近 -> 回检测位
```

## 1. 关键结论

如果 D435 支架相对 `tool0` 没动，只是机械臂从俯视姿态切换到平视姿态，原来的 `T_tool0_camera` 理论上仍然可用。

但程序必须区分两件事：

```text
观察姿态：为了让相机看到目标，可以俯视，也可以平视
到点姿态：为了让 tool0 到目标上方/附近，必须单独指定
```

俯视时，观察姿态通常就可以作为到点姿态。平视时，观察姿态通常不能直接作为到点姿态，否则容易出现 `CoordTransform has no IK`。

## 2. 俯视检测模式

让相机垂直向下看目标。该模式默认使用当前检测位 RPY 作为到点 RPY。

```bash
cd ~/Desktop/TL_Dobot
source /opt/ros/humble/setup.bash
source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
colcon build --symlink-install --packages-select tl_hand_deploy
chmod +x scripts/*.sh

TL_HOVER_MM=150 \
./scripts/run_no_gripper_hover_test.sh
```

窗口操作：

```text
点击目标中心
按 c 计算
确认 camera/base/hover TCP/IK joints
按 m 移动
按 r 回检测位
按 q 退出
```

## 3. 平视检测模式

让相机平视目标，用平视姿态观察；但是到点姿态用一套固定 RPY。

先把机械臂拖到一个“无手到点姿态”，也就是 tool0 朝向你希望最终靠近目标的姿态，然后记录 RPY：

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
ros2 topic echo /tcp_pose --once
```

记录：

```text
rpy.x
rpy.y
rpy.z
```

然后回到平视检测位，运行：

```bash
cd ~/Desktop/TL_Dobot

TL_HOVER_MM=150 \
TL_GRASP_RPY_RAD="3.087 -0.014 1.602" \
./scripts/run_no_gripper_hover_test.sh
```

把 `3.087 -0.014 1.602` 换成你实际记录的无手到点姿态 RPY。

也可以用角度：

```bash
TL_HOVER_MM=150 \
TL_GRASP_RPY_DEG="176.9 -0.8 91.8" \
./scripts/run_no_gripper_hover_test.sh
```

## 4. 平视检测的注意事项

平视相机点击到的是物体可见表面的 3D 点，不一定是物体几何中心，也不一定是顶面中心。所以这一步只用于无手粗到点验证。

如果出现：

```text
Nearest reachable probe found without moving
```

说明视觉算出的目标点在当前固定到点姿态下靠近 IK 边界。先移动目标区域或换一个更靠近工作区中心的检测位，不要盲目调速度。

如果出现明显整体偏差，先记录多个点的偏差。等无手平视/俯视都能稳定移动后，再进入假手九点校准。

## 5. 必须重做手眼标定的情况

只要 D435 相对 `tool0` 的安装位置或角度发生变化，就必须重新采集手眼标定。只有机械臂姿态变化、相机支架没有动时，才可以继续使用旧的 `T_tool0_camera`。

# DO3 / DO4 夹爪测试

## 能确认什么

手册只能确认控制器第3、4路数字输出是 `DO3`、`DO4`，不能确认哪一路是夹爪张开或闭合。张开/闭合由电磁阀线圈和现场接线决定，必须在低风险状态下实测。

测试程序不会发送机械臂运动命令。DO3、DO4 始终按互补状态使用：一路为高电平，另一路为低电平。`0`、正常结束和异常退出都会恢复初始互补状态，不会主动设置为 DO3=0、DO4=0。

默认初始状态为 `DO3=1, DO4=0`。如果你的现场要求相反，启动前设置：

```bash
TL_GRIPPER_INITIAL_HIGH=4 bash scripts/run_gripper_do_test.sh
```

## 测试前

1. 移走夹爪内和夹爪附近的物体。
2. 气动夹爪先降低气压；电动夹爪断开机械负载。
3. 确认电磁阀或继电器有独立负载电源和正确公共端。
4. 感性负载应有续流保护，不要在不了解额定电流时直接由DO驱动大功率线圈。
5. 保持急停可用，机械臂本体不需要移动。

## 启动

第一次使用前，先给天链驱动增加队列数字输出适配：

```bash
cd ~/Desktop/TL_Dobot
python3 scripts/patch_tl_driver_dout_api.py \
  --driver-root ~/Desktop/tl_robot_ros2_py-arm

cd ~/Desktop/tl_robot_ros2_py-arm
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select tl_ros2_interface tl_driver
source install/setup.bash
```

补丁把原来不存在的 `set_digital_output()` 调用改成单指令队列：开启队列、加入一个 DOUT、发送一条指令、关闭底层队列并等待控制器完成状态切换。它不会调用会执行 `power_off()` 的 ROS 队列状态服务，也不会加入任何机械臂运动指令。重复执行补丁脚本会自动升级已应用的旧版本。

重启天链驱动，然后在新终端执行：

```bash
cd ~/Desktop/TL_Dobot
source /opt/ros/humble/setup.bash
source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
colcon build --symlink-install --packages-select tl_hand_deploy
chmod +x scripts/run_gripper_do_test.sh
bash scripts/run_gripper_do_test.sh
```

看到确认提示后，确认夹爪为空、压力已降低，再输入大写 `TEST`。

## 按键顺序

1. 输入 `3`：切换到 `DO3=1, DO4=0`，保持0.5秒。观察夹爪后输入 `o`（张开）、`c`（闭合）或 `n`（没有动作），然后恢复初始互补状态。
2. 输入 `4`：切换到 `DO3=0, DO4=1`，保持0.5秒并记录动作，然后恢复初始互补状态。
3. 输入 `0`：恢复初始互补状态。
4. 输入 `s`：只有一个张开通道和一个闭合通道时才保存。
5. 输入 `q`：恢复初始互补状态并退出。

保存结果：

```text
~/tl_gripper_io/gripper_do_map.yaml
```

检查：

```bash
cat ~/tl_gripper_io/gripper_do_map.yaml
```

结果会明确显示 `open_port` 和 `close_port`。不要在确定映射前把夹爪动作接入自动抓取流程。

注意：如果硬件要求两个 DO 在切换的每一个瞬间都必须保持一高一低，两个独立数字输出无法保证原子切换，需要控制器提供双路原子写入或在电气侧增加互锁。当前程序保证最终状态和恢复状态是一高一低，并按低电平先切换以减少双线圈同时通电时间。

## 没有动作时

依次检查外部负载电源、公共端、DO3/DO4端子、继电器/电磁阀接线、气源压力以及驱动终端中 `Set digital output successfully` 返回。不要把两个 DO 长时间设置成相同电平。

如果提示 `tl_interface has no attribute set_digital_output`，说明驱动补丁尚未应用，或应用后没有重新编译并重启驱动。检查：

```bash
grep -n "queue_dout" \
  ~/Desktop/tl_robot_ros2_py-arm/src/tl_driver/lib/arm/tl_interface.py
grep -n "tl_interface.queue_dout" \
  ~/Desktop/tl_robot_ros2_py-arm/src/tl_driver/tl_driver/tl_driver_node.py
```

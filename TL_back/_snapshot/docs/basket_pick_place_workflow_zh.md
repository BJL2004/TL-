# 假手抓取并放入花篮流程

更新时间：2026-08-11

当前目标不是马上用真实傲意灵巧手抓取，而是先用假手验证完整路径：

```text
俯视检测位
-> 点击物体中心
-> 平抓接近物体
-> 模拟抓取
-> 抬升
-> 移动到花篮上方
-> 下降到花篮释放位
-> 模拟释放
-> 回检测位
```

## 1. 当前采用的方案

放置花篮不先做视觉识别，而是采用手动示教花篮位置。原因：

- 黑色花篮深度反射不稳定，D435 容易漏深度。
- 花篮在架子上，边缘和孔洞会让点击中心不稳定。
- 放置目标通常固定，示教两个位置比每次视觉估计更可靠。
- 后续真实灵巧手更脆弱，示教安全位可以减少碰撞风险。

因此需要示教两个位置：

```text
basket_approach：花篮上方安全位
basket_release ：花篮内部或入口上方释放位
```

假手阶段不会真正闭合；真实灵巧手阶段再接入 `/tl_hand/grasp` 和 `/tl_hand/open`。

## 2. 文件和程序

新增程序：

```text
tl_hand_deploy.mock_place_teach
tl_hand_deploy.mock_pick_place
```

新增脚本：

```text
~/Desktop/TL_Dobot/scripts/teach_mock_basket_pose.sh
~/Desktop/TL_Dobot/scripts/run_mock_pick_place_basket_test.sh
```

花篮示教文件默认保存到：

```text
~/tl_mock_pick_place/place_pose.yaml
```

## 3. 构建

复制最新工程后执行：

```bash
cd ~/Desktop/TL_Dobot
source /opt/ros/humble/setup.bash
source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash

colcon build --symlink-install --packages-select tl_hand_deploy

chmod +x scripts/teach_mock_basket_pose.sh
chmod +x scripts/run_mock_pick_place_basket_test.sh
chmod +x scripts/run_mock_hand_side_grasp_test.sh
```

## 4. 示教花篮位置

### 4.1 示教花篮上方安全位

先手动拖动机械臂，让假手位于花篮正上方，并且高度足够安全。建议离花篮口至少 100-150 mm。

然后执行：

```bash
cd ~/Desktop/TL_Dobot
./scripts/teach_mock_basket_pose.sh basket_approach
```

### 4.2 示教花篮释放位

再手动拖动机械臂，让假手到达你希望释放物体的位置。假手阶段不要碰到花篮壁；真实手阶段释放位也要预留手指张开的空间。

然后执行：

```bash
cd ~/Desktop/TL_Dobot
./scripts/teach_mock_basket_pose.sh basket_release
```

检查示教文件：

```bash
cat ~/tl_mock_pick_place/place_pose.yaml
```

必须同时看到：

```text
basket_approach
basket_release
```

## 5. 假手完整流程测试

先用较高抓取安全距离测试：

```bash
cd ~/Desktop/TL_Dobot

TL_PICK_GRASP_CLEARANCE_MM=70 \
./scripts/run_mock_pick_place_basket_test.sh
```

窗口出现后：

```text
1. 点击物体中心
2. 按 c 计算抓取位
3. 确认出现 pick approach IK 和 pick grasp IK
4. 确认没有 Command rejected/stopped
5. 确认路径没有碰撞风险
6. 按 g 执行完整抓取放置流程
7. 按 q 退出
```

流程执行顺序：

```text
move above pick target
move to pick grasp pose
fake hand simulated grasp
lift from pick target
move above basket
move to basket release pose
fake hand simulated release
return above basket
return observation pose
```

如果高位方向正确，再逐步降低：

```bash
TL_PICK_GRASP_CLEARANCE_MM=60 ./scripts/run_mock_pick_place_basket_test.sh
TL_PICK_GRASP_CLEARANCE_MM=50 ./scripts/run_mock_pick_place_basket_test.sh
TL_PICK_GRASP_CLEARANCE_MM=40 ./scripts/run_mock_pick_place_basket_test.sh
```

不要一开始就低高度测试。

## 6. 当前默认俯视平抓参数

脚本默认使用当前 5 点更新后的俯视补偿：

```text
TL_BASE_XY_BIAS_MM="-85.25 -16.44"
```

平抓姿态：

```text
TL_GRASP_RPY_RAD="-2.043 1.471 -1.025"
```

假手抓取中心相对 tool0 的初始偏移：

```text
TL_MOCK_GRASP_OFFSET_MM="-6.3 34.0 42.5"
```

## 7. 后续接入真实灵巧手

真实灵巧手必须先单独完成以下验证：

```text
1. 空载打开/闭合
2. 限流参数验证
3. 限力参数验证
4. 堵转保护验证
5. /tl_hand/stop 能冻结并轻微释放
6. 手指不会撞到花篮壁
```

确认后才允许启用：

```bash
TL_USE_HAND=1 ./scripts/run_mock_pick_place_basket_test.sh
```

启用后程序会调用：

```text
/tl_hand/set_enabled
/tl_hand/open
/tl_hand/grasp
/tl_hand/open
```

灵巧手保护节点仍然会限制：

```text
current_limit_ma
force_limit_mn
stall_speed
stall_current_ma
stop_after_period_ms
retry_after_period_ms
```

## 8. 安全注意事项

- 假手测试先只验证路径，不要追求低高度。
- 花篮释放位要给真实手指张开留空间。
- 真手测试前必须降低速度、限流、限力。
- 第一次真手抓取不要让手指夹硬物。
- 真实灵巧手如果状态异常，立即调用：

```bash
ros2 service call /tl_hand/stop std_srvs/srv/Trigger "{}"
```

- 机械臂运动仍是直接 ServoJ，必须保证物理急停可触达。

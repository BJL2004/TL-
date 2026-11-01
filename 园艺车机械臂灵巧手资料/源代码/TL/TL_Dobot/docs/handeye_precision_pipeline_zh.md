# TCB605 + D435 高精度手眼标定流程

## 1. 先说结论

当前 `session_20260808_022900` 的旧结果不能用于自动抓取：平移 RMS 为 9.87 mm，最大残差为 22.27 mm，并且文件本身是 `handeye_candidate_rejected.yaml`。

现有坐标链 `T_base_tool0 * T_tool0_camera * p_camera` 是正确的。旧结果不准不等于坐标公式错误，但也不能仅凭 0.91 度旋转 RMS 就断定唯一根因是姿态不足。必须结合逐样本残差、双轴旋转覆盖和留一法稳定性判断。

软件能保证的是“不合格结果不会被标记为可用”，不能脱离机械刚性、机器人重复定位精度和独立实测，承诺绝对抓取误差。

## 2. 新版质量门

新版求解器只允许 `intrinsic_xyz` 参与生产结果选择。`extrinsic_xyz` 仍写入 `report.json`，但仅供诊断，不会因为分数较低而被误选。

默认验收条件全部满足才生成 `handeye_accepted.yaml`：

| 项目 | 默认门槛 |
|---|---:|
| 有效样本 | 至少 20 组，建议 25-35 组 |
| 全局重投影 RMS | 不大于 0.8 px |
| 单帧最大重投影 | 不大于 1.0 px |
| 基座标定板平移 RMS | 不大于 5 mm |
| 基座标定板平移最大残差 | 不大于 10 mm |
| 基座标定板旋转 RMS | 不大于 0.5 度 |
| 基座标定板旋转最大残差 | 不大于 1.0 度 |
| TCP 平移总跨度 | 至少 120 mm |
| TCP 旋转总跨度 | 至少 40 度 |
| 旋转分量跨度 | 至少 2 个分量各覆盖 25 度 |
| 独立旋转主方向 | 至少 2 个主方向 RMS 达到 8 度 |
| 棋盘格图像覆盖 | 九宫格至少覆盖 5 格 |
| 留一法矩阵变化 | 平移最大 5 mm，旋转最大 0.5 度 |

离群剔除不是固定删除 10%。程序只逐个删除同时超过稳健统计阈值和绝对下限的明确异常帧，最多删除质量样本的 15%，并把样本编号、实测值和判定阈值写入 `report.json`。

## 3. 物理准备

1. 暂不安装灵巧手，D435 支架、法兰转接件和线缆必须固定，不能在运动时拉扯相机。
2. 棋盘格必须是 `9 x 6` 个内角点（外角 10 x 7 格），实测方格边长 `25 mm`；整张纸贴在刚性平板上，禁止卷曲。
3. 棋盘格固定在工作台，整个采集过程不能移动。
4. 使用 D435 彩色流 `640 x 480 @ 30 FPS`，不要同时启动其他 RealSense 程序。
5. 机械臂使用 `tool_num=0`、基座坐标；`/tcp_pose` 的 RPY 约定固定为 `intrinsic_xyz`。

任何一次相机支架拆装、碰撞、法兰松动或线缆明显拉动，都必须重新采集。

## 4. 重新采集

在 Jetson 上构建并启动：

```bash
cd ~/Desktop/TL_Dobot
source /opt/ros/humble/setup.bash
source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
colcon build --symlink-install --packages-select tl_hand_deploy
source install/setup.bash
ros2 launch tl_hand_deploy handeye_collect.launch.py
```

每组操作：按住末端 P 键拖动，松开后静止至少 2-3 秒，确认画面清晰后按 `s`。程序不主动发送机械臂运动指令。

采集 25-35 组，满足以下分布：

1. 棋盘格中心依次覆盖图像中央、左上、右上、左下、右下，最好覆盖 7-9 个九宫格区域。
2. 相机到棋盘格距离分布在约 0.30、0.40、0.50、0.60 m，不要全部在同一距离。
3. 绕两个正交方向分别覆盖正负倾角，目标每个方向总跨度 30-60 度。
4. 既要改变方向，也要改变位置；不要只绕法兰轴旋转，也不要只做平移。
5. 棋盘格必须完整入镜，面积和清晰度不合格时程序会拒绝保存。

预览中的 `coverage=N/9` 是棋盘格中心已经覆盖的九宫格数。每次保存还会更新：

```text
~/tl_handeye_data/session_*/collection_summary.json
```

结束前建议确认：样本数至少 25，九宫格覆盖至少 5，两个旋转分量跨度均不小于 25 度，并且 `rotation_principal_spread_deg` 至少有两个值不小于 8 度。后一个条件用于防止沿单一斜轴旋转被误判为双轴覆盖。

## 5. 求解与读报告

```bash
DATASET=~/tl_handeye_data/session_实际时间
ros2 run tl_hand_deploy tl_handeye_calibrate "$DATASET"
echo "exit_code=$?"
```

退出码 `0` 才表示内部质量门全部通过；退出码 `2` 表示只生成候选文件，禁止自动抓取。

重点查看：

```bash
cat "$DATASET/calibration/handeye_accepted.yaml"
cat "$DATASET/calibration/report.json"
```

`quality_checks` 必须全部为 `true`。`outlier_rejection.rejected_samples` 列出被剔除帧；若接近 15% 上限，不应继续相信该数据集，建议重新采集。`diagnostic_candidates_not_eligible_for_selection` 只是 RPY 约定对照，不能用于运行。

## 6. 九点绝对精度验证

内部一致性通过不等于抓取精度已经通过。先不加任何经验补偿，在工作平面布置 3 x 3 十字点，间距约 60 mm，使用短、轻、与 tool0 轴线同轴的指针验证。

每一点记录视觉目标与指针实际落点的基座坐标误差。建议验收：

| 项目 | 建议目标 |
|---|---:|
| 九点 XY RMS | 不大于 5 mm |
| 九点 XY 最大误差 | 不大于 8 mm |
| Z 高度最大误差 | 不大于 5 mm |

结果解释：

- 九点方向和大小接近：检查 tool0、指针或 TCP 的固定偏移，确认后才能使用常量补偿。
- 误差随 X/Y 位置系统变化：手眼旋转、机器人坐标或支架刚性仍有问题，必须重标定，不能用均值掩盖。
- 误差随机分散：检查点击、深度、纸面反光、机械臂静止与重复定位。

通过九点验证前，禁止启用灵巧手自动闭合与接触抓取。

完成 9 点保存后，使用严格验收命令。采集时 `--base-xy-bias-mm` 和 Z 补偿必须为零，也不能启用假手偏移：

```bash
LOG=~/tl_handeye_validation/本次九点记录.jsonl
ros2 run tl_hand_deploy tl_handeye_validate "$LOG" \
  --calibration "$DATASET/calibration/handeye_accepted.yaml"
echo "exit_code=$?"
```

退出码 `0` 且生成的 `.acceptance.json` 中所有 `quality_checks` 为 `true`，才表示 3 x 3 工作区绝对精度通过。该命令检查原始误差，不会用均值补偿隐藏大固定偏移。

旧版九点日志没有记录 Z 补偿来源，严格验收会拒绝这些旧记录；更新程序后需重新采集一套未补偿九点数据。

## 7. 安装灵巧手后的处理

无手标定得到的是 `T_tool0_camera`，不能直接当成掌心 TCP 的矩阵。已准确测得 `T_flange_tool` 时可转换：

```bash
ros2 run tl_hand_deploy tl_convert_handeye_frame \
  "$DATASET/calibration/handeye_accepted.yaml" \
  --flange-to-tool X_MM Y_MM Z_MM RX_RAD RY_RAD RZ_RAD \
  --output ~/tl_handeye_data/grasp_tcp_camera.yaml
```

转换程序默认拒绝 `accepted: false` 的来源。转换后还必须用掌心 TCP 重新做九点验证，并单独标定掌心中心、指尖接触面和安全进近距离。若灵巧手安装导致相机支架受力或位置变化，应直接重做手眼标定。

## 8. 旧数据的正确用途

旧的 `handeye_candidate_rejected.yaml` 只能用于非接触粗悬停和诊断。不要通过放宽门槛、继续叠加大幅经验补偿或使用 `--allow-rejected` 来宣称精度已经合格。

`TL_TCB/control_node.yaml`、旧版 `vision_grasp_demo.py` 内置矩阵和本工程生成的 YAML 不能混用。运行时必须打印并核对当前加载的标定绝对路径；重新标定后应清理启动脚本中的旧补偿。曾经从 `[-20.7, 5.3] mm` 漂移到 `[-85.25, -16.44] mm` 的经验补偿不是稳定工具偏移的证据，而是旧标定或验证条件不一致的警报。

## 9. 同步到 Jetson

最稳妥的方式是复制整个 `src/tl_hand_deploy` ROS 包和本说明文档，不要只替换单个 Python 文件。复制后执行：

```bash
cd ~/Desktop/TL_Dobot
source /opt/ros/humble/setup.bash
source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
colcon build --symlink-install --packages-select tl_hand_deploy
source install/setup.bash

ros2 pkg executables tl_hand_deploy | grep -E 'handeye_(collect|calibrate|validate)'
```

应看到 `tl_handeye_collect`、`tl_handeye_calibrate`、`tl_handeye_validate`。然后再按第 4-7 节重新采集、求解和验收。

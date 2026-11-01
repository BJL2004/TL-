# 天链机械臂无灵巧手定位与悬停测试

默认测试现已与 `TL_TCB/vision_grasp_demo.py` 的实际计算及运动链保持一致：

1. D435 深度对齐到彩色图。
2. 使用 `p_base = T_base_tool0 * T_tool0_camera * p_camera`。
3. TCP 姿态采用 `intrinsic XYZ`，目标保持检测位末端姿态。
4. 调用 `/tl_driver/coord_transform` 将 Base 位姿转换为关节角。
5. 调用 `/tl_driver/open_servoj`，以 500 点、100 Hz 发送 ServoJ 轨迹。
6. 真实关节误差小于 1 度才判定到位。

ServoJ 指令与参考程序一样直接发送到 `/tl_driver/set_servoj_pos`，不启动安全代理、不需要解锁。程序仍会在发送前检查工作区、关节限位、最大关节跳变和驱动订阅状态。与参考程序相比，目标由鼠标点击而不是 YOLO 检测，且不会向真实 `/joint_states` 发布伪造反馈。

## 当前精度结论

`TL_TCB/vision_grasp_demo.py` 内置的手眼矩阵与当前 `handeye_candidate_rejected.yaml` 相同，所以参考程序本身不能独立证明该矩阵准确。现有 23 组数据的平移 RMS 约 9.87 mm、最大残差约 22.27 mm，未通过 8 mm 门槛，只能做无接触粗测，不能用于安装灵巧手后的准确抓取。

另外，`TL_TCB/control_node.yaml` 中还存在另一份矩阵；它与 `vision_grasp_demo.py` 实际使用矩阵相差约 52.95 mm 和 4.72 度。当前兼容基准只认实际运行的 `vision_grasp_demo.py`，禁止混用 `control_node.yaml` 的矩阵。

## 一、对照标定算法

```bash
cd ~/Desktop/TL_Dobot
source /opt/ros/humble/setup.bash
source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
export PYTHONPATH="$PWD/src/tl_hand_deploy:${PYTHONPATH:-}"
chmod +x scripts/*.sh

./scripts/run_handeye_accuracy_compare.sh \
  ~/tl_handeye_data/session_20260808_022900
```

程序会分别重算：

- TL_Dobot：固定 D435 出厂内参，比较全部手眼方法。
- TL_TCB：从棋盘图片重新估计内参，固定 `Tsai + intrinsic XYZ`。

报告保存到：

```text
~/tl_handeye_data/session_20260808_022900/calibration/handeye_accuracy_comparison.json
```

该报告检查数据集内部一致性，不等于末端对物体的绝对定位精度。绝对精度必须用下方实机悬停测试测量。

## 二、重新构建

```bash
cd ~/Desktop/TL_Dobot
source /opt/ros/humble/setup.bash
source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
colcon build --symlink-install --packages-select tl_hand_deploy
chmod +x scripts/*.sh
```

若之前启动过 `safe_arm_node`，先关闭旧终端，确保只保留一个实例。

## 三、先只计算，不运动

```bash
cd ~/Desktop/TL_Dobot/src/tl_hand_deploy
source /opt/ros/humble/setup.bash
source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
export PYTHONPATH="$PWD:${PYTHONPATH:-}"

python3 -m tl_hand_deploy.overhead_test \
  --calibration ~/tl_handeye_data/session_20260808_022900/calibration/handeye_candidate_rejected.yaml \
  --allow-rejected \
  --hover-mm 100
```

鼠标点击物体顶面中心后按 `c`。检查终端中的 `camera[m]`、`base[mm]`、`hover TCP` 和 `IK joints[deg]`。未添加 `--execute` 时，按 `m` 也不会发送运动命令。

## 四、无灵巧手实机悬停

确认天链驱动已运行、路径无障碍物、急停可立即触达，然后只执行：

```bash
cd ~/Desktop/TL_Dobot
./scripts/run_no_gripper_hover_test.sh
```

操作顺序：

1. 点击物体顶面中心。
2. 按 `c`，核对相机点、Base 点、悬停 TCP 和 IK 关节值。
3. 确认路径无人且目标合理后按一次 `m`，机械臂直接开始运动。
4. 测量法兰中心投影与物体中心的 X/Y 偏差。
5. 按 `r` 返回启动时记录的检测位关节。
6. 按 `q` 退出，程序自动关闭 ServoJ。

建议在工作区左、中、右至少各测 3 个点并记录 X/Y 偏差。当前无灵巧手测试保持 100 mm 悬停，不下降、不接触物体；现有标定最大残差约 22.27 mm，因此不得继续降低悬停高度。

## 五、多点绝对精度实测

仅依靠相机图像不能独立验证同一份手眼矩阵。需要在法兰 `tool0` 轴线上安装一根轻质直指针，长度必须小于 100 mm，并在桌面固定一张带 3×3 十字点的测试纸。指针不得接触纸面，也不得改变 D435 支架位置。

每个测试点执行：

1. 点击十字中心，按 `c` 计算，再按 `m` 到视觉预测位置。
2. 从机械臂 Base 坐标方向观察指针与十字的水平偏差。
3. 使用 `a/d` 调整 Base-X 减/加，使用 `s/w` 调整 Base-Y 减/加；默认每次 1 mm，Z 和姿态保持不变。
4. 指针对准十字后按 `v` 保存实测修正量。
5. 按 `r` 回检测位，移动测试纸或选择下一个分布位置，重复至少 6 点，推荐 3×3 共 9 点。

记录文件：

```text
~/tl_handeye_validation/offsets.jsonl
~/tl_handeye_validation/offsets.summary.json
```

报告含义：

- `mean_correction_base_mm`：应加到视觉目标上的固定 Base-XYZ 补偿。
- `after_fixed_bias_xy_rms_mm`：扣除固定补偿后的平面 RMS。
- `after_fixed_bias_xy_max_mm`：扣除固定补偿后的最大平面误差。
- `predominantly fixed offset`：主要是固定偏移，可先加常量补偿。
- `position-dependent systematic error`：误差随工作区位置变化，应重新手眼标定，不能只加常量。
- `scattered measurement/calibration error`：测量或标定数据离散，需要检查指针对准重复性、深度和标定采集质量。

旧 MoveL 诊断版仍可通过以下命令查看，但不再是默认测试：

```bash
python3 -m tl_hand_deploy.overhead_movel_test --help
```

## 六、测试纸

多点精度验证使用：

```text
assets/calibration/handeye_validation_3x3_cross_60mm_A4.pdf
```

这是 A4 纸、3×3 十字中心、相邻点间距 60 mm 的精度测试纸。打印时选择“实际大小”或“100%”，禁止选择“适合页面”；打印后先用直尺确认底部校尺线恰好为 100 mm。该纸用于实测空间偏差，不用于重新计算手眼矩阵。

原手眼标定棋盘格仍位于：

```text
assets/calibration/checkerboard_10x7_inner_corners_15mm.pdf
```

棋盘格是 10×7 内角点、15 mm 方格，只用于重新采集手眼标定数据，不能替代 3×3 精度测试纸。

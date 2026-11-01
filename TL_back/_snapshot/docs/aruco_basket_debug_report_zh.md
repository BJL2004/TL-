# 花篮动态定位精度问题排查报告

日期：2026-08-14
对象：TCB605 + D435 + 22cm 假手 + 6 个 ArUco 花篮（长栏一字排开布局）
软件：TL_Dobot 仓库（Jetson Orin Nano 运行，Windows 开发）

---

## 一、背景与目标

6 个独立花篮沿长栏排列，每个花篮旁刚性固定一张 ArUco 码（编号 1-6 ↔ ArUco ID 0-5）。
目标：机械臂通过相机识别码，动态定位花篮位置，假手自动到达花篮上方完成"放置"，
花篮位置变化后无需重新示教。

流程：示教（B→A→P→R）→ 动态定位（实时估计码在基座系的位置）→ 放置循环（4 步）。

## 二、动态定位原理

示教时记录的不是画面坐标，而是**码坐标系下的工具位姿**：

```text
T_marker_tool0 = 码位姿⁻¹ × 工具位姿      （示教时保存到 basket_aruco.yaml）
```

抓取/放置时：

```text
目标点 = 码现在位姿 × T_marker_tool0
       = base_marker_now @ inv(base_marker_teach) @ 示教时tool位姿
```

码现在位姿由实时链路估计：`base 系码位姿 = TCP位姿 × 手眼标定 × PnP检测`。

**关键性质（误差抵消）**：当码处于示教时的位置、且机械臂处于示教时的搜索位时，
上式中的两次测量误差高度相关并大部分互相抵消，目标点接近示教位姿——这就是
"示教位置恒可达"的数学原因。

**码一旦移动**，只有 `base_marker_now` 是实时估计的，其误差（码尺寸、倾斜、
画面位置、距离）直接变成目标点偏差，且随移动距离放大。

## 三、问题现象

1. 码在示教位置时，放置路径完全正常；
2. 码稍微改变在摄像头中的位置（移动花篮 3-5cm 以上），目标点大幅偏移，
   机械臂"完全到达不了"（对不准花篮 / 目标点不可达）；
3. 测试多个花篮均不准确。

## 四、根因分析

### 根因 1：码实际尺寸 ≠ 标称尺寸（头号根因）

- 程序默认按 **60mm** 解算 PnP，实测黑色方形外边为 **58mm**（比例误差 3.4%）；
- 后果：码的距离估计偏大 3.4%（0.4m 处约 14mm），码横向移动时估计的移动量
  同样按比例放大，码离示教位置越远偏差越大；
- 诊断印证：B 定位反推的码宽度为 61.8–63.8mm（标称 60mm），明显偏大。

### 根因 2：示教数据新旧混杂（直接导致"测 3 组都不准"）

- 首次用 60mm 示教了全部 6 个花篮；
- 修正为 58mm 后只重示教了 6、2、4 号，**1、3、5 号仍为 60mm 旧数据**；
- yaml 顶层 `marker_length_m` 已被更新为 0.058 → 旧数据的相对变换按 58mm
  解算，混合误差，定位必然不准。

### 根因 3：码的视角（倾斜 / 画面位置 / 距离）影响 PnP 精度

- 码平面与相机视线夹角越大，PnP 越差；倾角 >50° 时两个数学解接近，
  存在解翻转风险（目标点飞掉）；
- 码在画面边缘时畸变残余误差大（实测 170px 边缘处定位明显偏差）；
- 观测距离推荐 0.25–0.7m，过远时角点像素小、误差占比大；
- 注：示教与测试在同一搜索位（ServoJ 精确回位，误差 0.005°）时，
  手眼标定的固定分量会在两次测量中部分抵消；但手眼旋转误差仍会把码的实时位移
  映射到错误方向，因此它不是零影响。必须使用通过验收的同一份手眼标定完成示教与运行。

### 根因 4：程序缺陷——`/joint_states is stale` 中断放置循环

- 长时间 ServoJ 流式运动（14s）期间主线程不 spin，话题回调积压，
  `_validate_joint` 误判 joint_states 过期，Step 4/4 返回动作被拒。

## 五、修复措施（提交记录）

| 提交 | 内容 |
|---|---|
| 70def7f | 花篮脚本默认标定改为 best/handeye_best.yaml，bias 改为 -53.2 -34.2 |
| e79faa1 | 模块 parse_args 默认标定/bias 同步 |
| da0885b | 花篮脚本自动 source ROS2 与驱动工作空间（新终端可直接运行） |
| 394030a | 固定观测位改为示教观测位（关节角 -89.940 -15.911 56.571 2.025 18.634 -1.448） |
| 8e4085c | 新增 place-only 模式（跳过抓取，只放置，无需点击物体） |
| 6910445 | 定位诊断打印：码实测宽度反推 + 码基座系位置 |
| 19c7c2e | 示教默认码尺寸改为实测 58mm（--marker-length-m 0.058） |
| cf054da | _validate_joint 等待 0.6s 刷新 joint_states，修复 stale 误判 |
| 本次修复 | 深度辅助码中心、平移模式、倾角诊断、旧示教数据一致性门槛 |

## 六、诊断打印说明（B 定位时输出）

```text
marker number=4 estimated base position[mm]=[23.3, -565.4, 53.3]   ← 码在基座系的位置
marker measured black-square width=60.4 mm (nominal 58.0, deviation 4.1%)  ← 尺寸反推
marker plane tilt from camera line of sight=XX.X deg               ← 码平面倾角
dynamic basket_approach TCP=[...]                                   ← 算出的目标点
```

判定标准：

| 指标 | 合格 | 异常 |
|---|---|---|
| 宽度偏差 | <2% | >5% 时检查码尺寸/打印变形 |
| 倾角 | <40° | >50° 时转动码正对相机 |
| 目标点 vs 示教位（码未动时） | 差异 <5mm | 差异大 → 码被碰动或示教位置不准 |

## 七、操作规范（必读）

1. **码的尺寸**：示教前用尺实测每张码的黑色方形四边，全部一致才用该值配置
   `--marker-length-m`（本套码实测 58mm）；打印变形/非正方形会导致定位偏差；
2. **码的安装**：与花篮刚性固定、平整无反光；码平面尽量正对相机（倾角 <40°）；
3. **示教规范**：6 个花篮必须**全部**重示教（B→A→P→R），按 B 后该花篮不能移动；
   A/P 位置必须真实对准篮口（悬停释放则 A=P 同高度，但要确认 XY 对准）；
4. **测试规范**：移动花篮后让码留在画面中心附近、距离 0.3–0.6m、朝向尽量不变；
5. **校准一致性**：示教与测试使用同一份标定（best/handeye_best.yaml）；
   重新标定后必须重新示教全部花篮。

## 八、当前状态与后续建议

- 已具备：6 花篮动态定位（自动扫描）、深度辅助平移模式、place-only 放置循环、
  诊断打印、观测位统一；
- 待完成：复制本次修改到 Jetson，用 58mm **重示教全部 6 个花篮**，再移动码验证；
- 后续：真实灵巧手（ROH-LiteS001）安装后需验证握爪间隙与 TCP 换算
  （convert_handeye_frame.py）；必要时做正式 9 点验证与位置相关残差建模。

## 九、本次问题的最终解释

“示教码的位置可以到达，花篮稍微移动后就偏差很大”并不表示机械臂只能到达
示教坐标，也不首先说明 IK 失败。它说明示教位置处的误差被两次相同的码测量
部分抵消了：

```text
旧算法目标 = base_marker_now @ inv(base_marker_teach) @ taught_tool_target
```

这个刚体变换的乘法顺序是正确的，但 `base_marker_now` 和
`base_marker_teach` 都来自单张平面 ArUco 的 PnP。花篮不动时，两次 PnP 的
系统误差接近，目标看起来准确；花篮移动到图像的另一位置或另一距离后，码尺寸
比例误差、边缘畸变、倾斜造成的 PnP 旋转误差就不再相同，误差直接进入目标点。
因此“只能回到示教码位置”是定位误差被抵消后的典型表现。

还要特别注意：旧诊断中的“实测黑方宽度”曾使用 PnP 自己算出的 `z` 反推，
这不能独立验证码的真实尺寸，因为 PnP 的尺寸假设已经参与了 `z` 的计算。
现在改为用 D435 对齐彩色图像的深度在码中心取值，再反推宽度，诊断才是独立的。

## 十、本次代码修复

当前动态花篮模式默认改为 `translation`：

1. 示教 B 时保存每个码在基座系的 `T_base_marker`、码尺寸和深度来源；
2. 运行 B 时用 D435 深度对齐后的码中心计算实时三维平移，PnP 只提供角点和显示姿态；
3. 计算放置目标时只使用“实时码中心 - 示教码中心”的基座系平移增量；
4. 目标工具的 RPY 保持示教时的 RPY，不再让一次噪声 PnP 旋转带动 22 cm 假手的末端位置；
5. 默认限制花篮相对示教位置的最大移动量为 200 mm，超过则停止并提示定位可能错误；
6. 旧版缺少动态参考的示教记录不再默默使用，程序会要求用同一尺寸重新示教。

这套模式适合你的架子：花篮允许平移，ArUco 与花篮的安装方向基本不变。只有当花篮
确实发生明显旋转时，才使用 `TL_BASKET_MOTION_MODE=rigid`；刚体模式对单个平面码
的 PnP 姿态质量要求更高。

## 十一、重新示教与验证

旧的 60 mm/58 mm 混合记录必须重新示教，不能只改 YAML 顶层数字：

```bash
mv ~/tl_mock_pick_place/basket_aruco.yaml \
  ~/tl_mock_pick_place/basket_aruco.legacy.$(date +%Y%m%d_%H%M%S).yaml
cd ~/Desktop/TL_Dobot
source /opt/ros/humble/setup.bash
source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
./scripts/teach_aruco_basket.sh
```

对每个花篮严格执行 `B -> A -> P -> R`，再拖到下一个花篮。完成后确认：

```bash
python3 - <<'PY'
import yaml
p = '/home/nvidia/tl_mock_pick_place/basket_aruco.yaml'
d = yaml.safe_load(open(p, encoding='utf-8'))
for marker_id in range(6):
    r = d['markers'][str(marker_id)]
    print(marker_id + 1, 'T_base_marker=', 'T_base_marker' in r['search_pose'],
          'length=', r['search_pose'].get('marker_length_m'),
          'approach=', 'basket_approach' in r,
          'release=', 'basket_release' in r)
PY
```

测试时让花篮只做 20--50 mm 平移，码保持平整并尽量位于画面中心，运行：

```bash
TL_USE_HAND=0 \
TL_BASKET_MOTION_MODE=translation \
TL_MAX_BASKET_SHIFT_MM=200 \
./scripts/run_dynamic_aruco_basket_test.sh
```

窗口操作仍按程序提示：`B` 定位花篮，`C` 计算物体，`G` 执行完整流程；只验证放置时
设置 `TL_PLACE_ONLY=1`。重点观察输出中的：

```text
marker measured black-square width=...
marker plane tilt from camera line of sight=...
basket translation from teach[mm]=...
dynamic basket_approach TCP=...
dynamic basket_release TCP=...
```

如果花篮实际平移 50 mm，而输出平移约为 `[50, 0, 0]`（允许几毫米噪声），说明
动态定位链路正常；如果码中心平移 50 mm、输出却明显缩放或方向错误，应先检查码实际
黑方边长、D435 深度对齐、码是否翘曲，再继续调手眼补偿。不要用一个新的固定 XY bias
去掩盖这种位置相关误差。

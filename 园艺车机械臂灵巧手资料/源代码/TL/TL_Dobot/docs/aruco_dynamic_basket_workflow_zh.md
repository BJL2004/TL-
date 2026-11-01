# ArUco 动态花篮定位与假手抓放流程（最终版）

更新时间：2026-08-14

适用于：TCB605 + D435 + 22cm 假手 + 6 个独立花篮（长栏一字排开，每篮旁一张 ArUco 码）。

整体流程：

```text
示教（B→A→P→R × 6）→ 抓取（C）→ 去花篮（R）→ 放置（g）→ 全程自动回默认位
```

## 一、使用前检查

1. 用尺实测每张码的黑色方形**外边长**，并确认**四边一致**；本套码实测 58 mm，
   示教脚本默认 `--marker-length-m 0.058`。白边和数字不计入尺寸。
2. 码必须平整、无反光、与花篮**刚性固定**，示教后码相对花篮不能移动。
3. 码平面尽量**正对相机**（与视线夹角 <40°；>50° 时 PnP 有解翻转风险）。
4. 观测距离 0.3–0.6 m 最佳；码保持完整可见。
5. 假手阶段保持 `TL_USE_HAND=0`。

## 二、构建与环境

```bash
cd ~/Desktop/TL_Dobot
chmod +x scripts/*.sh
colcon build --symlink-install --packages-select tl_hand_deploy
```

驱动必须已运行，且 `/tcp_pose`、`/joint_states`、`/tl_driver/coord_transform`、
`/tl_driver/set_servoj_pos` 正常。ServoJ 控制程序同一时刻只能运行一个
（不能与 safety_only.launch.py / 另一个示教或抓取程序并存）。

## 三、示教（v2 格式，必须全部 6 个）

```bash
./scripts/teach_aruco_basket.sh
```

对每个花篮执行一轮（共 6 轮）：

```text
1. 拖到能看到该花篮码的位置，静止 2 秒
2. 按 B   锁定该花篮（选中离画面中心最近的码，冻结码位姿、存搜索位）
3. 拖到接近位（篮口上方 120-180mm 或释放高度），静止 1 秒，按 A
4. 拖到释放位（A/P 同高度=悬停释放；或低 30-50mm），静止 1 秒，按 P
5. 按 R   ServoJ 自动返回搜索位（保持急停可触达）
6. 平移到下一个花篮，重复 1-5
```

注意：

- 从按 B 到按 P 之间，该花篮不能移动；
- A/P 时码不需要在画面里（用 B 时冻结的位姿），但花篮本体不能动；
- 全部完成后按 Q。检查：

```bash
grep -c "T_base_marker" ~/tl_mock_pick_place/basket_aruco.yaml   # 应为 6
head -8 ~/tl_mock_pick_place/basket_aruco.yaml                   # format_version: 2
```

- 旧格式（无 `T_base_marker`）或码长不一致的数据会被程序直接拒绝，需重示教；
- 重新标定、移动 D435 支架、更换工具后必须全部重示教。

## 四、分步抓放流程（推荐，place-only 模式）

```bash
TL_PLACE_ONLY=1 TL_USE_HAND=0 ./scripts/run_dynamic_aruco_basket_test.sh
```

启动后自动进入默认位 HOME（TCP≈203.4, 3.4, 374.9）。

| 按键 | 动作 | 结束位置 |
|---|---|---|
| 点击 + `C` | 计算 + 自动抓取抬升（接近→降抓取位→模拟抓取→抬升） | 自动回 HOME |
| `a`/`d` | 微调目标 X（±1mm，自动重新抓取） | HOME |
| `s`/`w` | 微调目标 Y（±1mm） | HOME |
| `j`/`u` | 微调目标 Z（j 降 / u 升，±5mm） | HOME |
| `R` | 自动定位花篮（无需先按 B，自动扫描找码）→ 移动到花篮示教位置 | 花篮上方 |
| `g` | 放置（降释放位→模拟释放→回花篮上方）→ 自动回默认位 | HOME |
| `Q` | 退出 | — |

说明：

- `R` 会先自动定位：不指定编号时沿 6 个示教搜索位扫描，找到第一个稳定码；
  也可先按数字键 `1`-`6` 指定花篮再按 `R`，按 `0` 恢复自动；
- 花篮移动后重新按 `R` 即可（translation 模式自动补偿平移，保持示教姿态）；
- 抓取位不准时用微调键逐步逼近（位置打印会显示调整量）。

## 五、完整循环模式（可选）

不设 `TL_PLACE_ONLY`（默认 0）时，按 `g` 为全自动 8 步循环
（抓取→回 HOME→去花篮→放置→回 HOME），中途不人工确认。
日常建议用第四节的分步模式。

## 六、关键参数（脚本默认已配置）

| 环境变量 | 默认值 | 说明 |
|---|---|---|
| `TL_HOME_JOINT_DEG` | `-1.521 -1.225 35.678 3.856 31.471 -3.533` | 默认位/起始/返回位 |
| `TL_OBSERVATION_JOINTS` | `-89.940 -15.911 56.571 2.025 18.634 -1.448` | 花篮观测位（未设 HOME 时用） |
| `TL_PLACE_ONLY` | `0` | `1` = 分步流程 |
| `TL_BASKET_MOTION_MODE` | `translation` | 只补偿平移、保持示教姿态 |
| `TL_MAX_BASKET_SHIFT_MM` | `200` | 花篮位移超限拒绝（防跑飞） |
| `TL_BASE_XY_BIAS_MM` | `-53.2 -34.2` | 已烧录的抓取补偿 |
| `TL_PICK_APPROACH_CLEARANCE_MM` | `130` | 抓取上方安全高度 |
| `TL_PICK_GRASP_CLEARANCE_MM` | `50` | 抓取位（点击点上方高度） |
| `TL_FINE_STEP_MM` / `TL_FINE_Z_STEP_MM` | `1` / `5` | 微调步长 |

## 七、定位诊断打印（按 R/B 时）

```text
marker number=N estimated base position[mm]=[...]      ← 码在基座系的位置
marker measured black-square width=58.X mm (nominal 58) ← 尺寸反推（倾角<20° 才可信）
marker plane tilt from camera line of sight=XX.X deg   ← 倾角（>50° 警告）
basket translation from teach[mm]=[...]                ← 花篮相对示教的位移
dynamic basket_approach TCP=[...]                       ← 算出的目标点
```

码未移动时 `translation` 应 ≈ [0,0,0]；移动 3-5cm 后应等于实际移动量。

## 八、分阶段验证与注意事项

建议按顺序验证：

1. 花篮原位：`translation ≈ [0,0,0]`，目标点等于示教位；
2. 花篮平移 30 mm → 80 mm：目标跟随，误差数 mm 内；
3. 多次重复后再换真实灵巧手。

注意：

- 物体必须放在机械臂正前方 30 cm 内、侧抓姿态可达区（目标 base 约
  X 300-380、Y +20~+60、Z -60 以上）；目标不可达时程序会报 IK 错误或
  auto-lift 悬空（打印 `Auto-lifted target`），此时应挪动物体而非盲目执行；
- 每次移动花篮后必须重新按 `R` 定位；定位后到按 `g` 前禁止移动花篮；
- 若报 `basket shift exceeds 200 mm`，说明定位异常或花篮移出区域；
- 若报 `version-2` / `re-teach` 错误，说明该花篮示教数据过期，需重示教。

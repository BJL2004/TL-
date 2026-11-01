# 灵巧手抓取-花篮放置完整操作手册

更新时间：2026-08-15
适用：TCB605 机械臂 + D435 相机 + ROH-LiteS001 灵巧手 + 6 个 ArUco 花篮（长栏布局）

---

## 一、系统组成与程序分工

| 程序 | 命令 | 用途 |
|---|---|---|
| 灵巧手测试 | `bash scripts/run_roh_hand_test.sh` | 灵巧手独立测试（初始化/手势/力度分级/位置电流显示） |
| 示教 | `bash scripts/teach_aruco_basket.sh` | 记录 6 个花篮位置（B→A→P→R，启动自动到示教起始位） |
| 花篮抓放流程 | `TL_USE_HAND=1 TL_PLACE_ONLY=1 bash scripts/run_dynamic_aruco_basket_test.sh` | 完整抓取→放置流程（本手册核心） |
| 抓取高度标定 | `TL_USE_HAND=0 bash scripts/run_mock_hand_side_grasp_test.sh` | 标定抓取真实高度（g 下降/j 微调/Z 记录日志） |

通信：机械臂 ModBus 透传端口 2（RS485）控制灵巧手（从站 2、115200）；
花篮定位：ArUco 码（DICT_4X4_50，实测 58mm）+ D435 深度。

## 二、关键位置（已固化）

| 位置 | 关节角 (°) | TCP (mm) | 用途 |
|---|---|---|---|
| HOME | -1.521, -1.225, 35.678, 3.856, 31.471, -3.533 | 203.4, 3.4, 374.9 | 起始/待机/抓取位 |
| 检测位（观测位） | -83.668, 67.725, 3.956, -2.101, -5.484, -1.431 | -8.7, 27.7, 414.1 | 看码识别花篮、R/g 中转 |
| 抓取高度 | TL_FINAL_GRASP_Z_MM=69.62（base 系 tool0 Z） | — | 假手/灵巧手末端贴物体表面 |

## 三、灵巧手测试（换装后必做）

```bash
bash scripts/run_roh_hand_test.sh
```

| 键 | 功能 | 键 | 功能 |
|---|---|---|---|
| z | 初始化（自检） | 1-6 | 抓取力度分级（极轻→握拳） |
| r | 软复位 | v/m/p/k/g/s/i | 手势（比耶/中指/捏合/OK/赞/摇滚/指） |
| o | 张开 | q | 退出 |

每次命令后显示 6 指位置与电流。确认：初始化成功、各指正常运动、无堵转/空转。

## 四、示教（关键：灵巧手抓握状态）

**重要**：放置时灵巧手是抓握状态（握着物体），所以示教 A/P 时**手必须保持抓握状态**（握着典型物体），否则几何不符、放置偏移。

```text
1. 先让手进入抓握状态（保持不松）：
   bash scripts/run_roh_hand_test.sh → z 初始化 → o 张开 → 放典型物体 → 5 抓握 → q 退出
2. 示教（自动运动到示教起始位 -83.668...）：
   bash scripts/teach_aruco_basket.sh
3. 每个花篮一轮，共 6 轮（全程手抓握）：
   拖到码可见 → 静止2s → B → 拖到接近位 → A → 拖到释放位 → P → R 返回 → 下一花篮
4. 验证：grep -c "T_base_marker" ~/tl_mock_pick_place/basket_aruco.yaml  # 应为 6
```

清理重示教：`mv ~/tl_mock_pick_place/basket_aruco.yaml ~/tl_mock_pick_place/basket_aruco.yaml.bak` 后全部重示教。

## 五、完整抓放流程（日常操作）

```bash
cd ~/Desktop/TL_Dobot
TL_USE_HAND=1 TL_PLACE_ONLY=1 bash scripts/run_dynamic_aruco_basket_test.sh
```

| 按键 | 动作 | 结束位置 |
|---|---|---|
| 点击 + C | 抓取：到位 69.62 → 灵巧手力度5抓握 → 抬升 | HOME |
| R | 自动定位花篮（扫描找码）→ 移动到检测位 | 检测位 |
| g | 去花篮上方 → 释放位 → 灵巧手松开（物体落花篮）→ 回花篮上方 | 检测位 |
| R | 回 HOME | HOME |
| Q | 退出 | — |

灵巧手只在两处动作：**抓取位（力度5抓握）、花篮释放位（松开）**，中间全程保持抓握。

## 六、关键参数（脚本默认已固化）

| 环境变量 | 默认值 | 说明 |
|---|---|---|
| TL_HOME_JOINT_DEG | -1.521 -1.225 35.678 3.856 31.471 -3.533 | HOME 位 |
| TL_OBSERVATION_JOINTS | -83.668 67.725 3.956 -2.101 -5.484 -1.431 | 检测位 |
| TL_FINAL_GRASP_Z_MM | 69.62 | 抓取真实高度（tool0 base Z） |
| TL_HAND_GRASP_POSITIONS | 35000×5 + 25000 | 抓取力度 5（6 指位置） |
| TL_HAND_OPEN_POSITIONS | 0×6 | 松开（全开） |
| TL_BASKET_MOTION_MODE | translation | 只补偿花篮平移，保持示教姿态 |
| TL_MAX_BASKET_SHIFT_MM | 200 | 花篮位移超限拒绝（防跑飞） |
| TL_BASE_XY_BIAS_MM | -53.2 -34.2 | 抓取补偿（已烧录） |
| TL_PICK_APPROACH/GRASP_CLEARANCE_MM | 130 / 50 | 抓取安全高度 |

## 七、动态补偿能力与限制

- **平移/上下移动**：✅ 补偿（shift 为完整 3D 平移差），范围 200mm 内
- **限制**：码必须在搜索位视野内；目标位必须在 IK 工作空间；Z 方向精度约 ±1cm 内（深度测量）；花篮**旋转**不补偿（需 rigid 模式，不推荐）；码相对花篮移动必须重示教
- 每次移动花篮后按 R 重新定位；定位后到按 g 前禁止移动花篮

## 八、故障排查

| 现象 | 处理 |
|---|---|
| 驱动没启动 | `ros2 launch tl_driver tl_tcb605_driver.launch.py`，确认 /tcp_pose 话题 |
| 灵巧手无响应 | 检查 RS485 接线、供电、从站 2；先跑 roh_hand_test 的 z 初始化 |
| 报 version-2 / re-teach | 该花篮示教数据过期，重示教 |
| basket shift exceeds 200mm | 定位异常或花篮移太远，检查码 |
| 报 Auto-lifted / IK 错误 | 物体位置不可达，挪物体（正前方 30cm、Y≈+20~+60） |
| 放置偏移 | 检查是否用灵巧手抓握状态重新示教；固定偏移可报告补偿 |
| 找不到码 | 码贴平无反光、完整可见、距离 0.3-0.6m、倾角 <40° |

## 九、文件清单

```text
src/tl_hand_deploy/tl_hand_deploy/roh_hand_test.py      灵巧手测试
src/tl_hand_deploy/tl_hand_deploy/aruco_basket_teach.py 示教
src/tl_hand_deploy/tl_hand_deploy/mock_pick_place.py    花篮抓放流程
src/tl_hand_deploy/tl_hand_deploy/overhead_test.py      抓取标定（M/R 行为）
scripts/run_roh_hand_test.sh
scripts/teach_aruco_basket.sh
scripts/run_dynamic_aruco_basket_test.sh
scripts/run_mock_pick_place_basket_test.sh
scripts/run_mock_hand_side_grasp_test.sh
docs/aruco_dynamic_basket_workflow_zh.md                动态花篮流程文档
docs/aruco_basket_debug_report_zh.md                    排查报告
```

## 十、换装注意事项

假手→灵巧手后必须：① 重示教 6 个花篮（抓握状态）；② 重新标定抓取高度（TL_FINAL_GRASP_Z_MM）；③ 用 roh_hand_test 确认通信与手势。全部通过后再进完整流程。

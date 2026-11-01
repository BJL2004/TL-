# TL_Handback：两个完全独立的程序

**TL_Handback 是一个 ROS2 工作区，里面有两个互不相关的程序包：**

| | 程序 1：灵巧手抓种子 | 程序 2：U 型叉子叉花篮 |
|---|---|---|
| 包 | `src/tl_hand` | `src/tl_fork_teach` |
| 干什么 | 抓取小种子（如 A1-01）**放入**花篮 | 用 U 型叉子把**整个花篮**叉起、转运、放回 |
| 抓取工具 | ROH-LiteS001 灵巧手 | U 型叉子（无工具控制） |
| 通信 | MQTT 命令 | 纯键盘（无 MQTT） |
| 花篮定位 | ArUco 视觉自动定位 | ArUco 视觉 + 示教关节直驱 |
| 数据文件 | `~/tl_mock_pick_place/basket_aruco.yaml` | `~/tl_fork_teach/fork_basket_teach.yaml` |
| 启动脚本 | `scripts/run_fixed_pick_mqtt.sh` | `scripts/run_fork_basket_teach.sh`（示教） |

**两个程序互不调用、互不干扰，不要混用。** 灵巧手程序不会动叉子，叉子程序不会碰灵巧手。不能同时运行。

---

# 程序 1：灵巧手抓种子（tl_hand 包）

## 四排花篮架构（24 个花篮）

架子分左右各两排，每排 6 个花篮，共 **24 个**：

```
        左 侧（A）          右 侧（B，J1 镜像）
上排   A1-01 ... A1-06     B1-01 ... B1-06
下排   A2-01 ... A2-06     B2-01 ... B2-06
```

- 编号规则：`A1-01` = 左上第 1 个，`B2-06` = 右下第 6 个
- 内部编号：A1-01..06 → 1..6，A2-01..06 → 7..12，B1-01..06 → 13..18，B2-01..06 → 19..24
- ArUco ID：内部编号 - 1 = 0..23（DICT_4X4_50，码长 58mm）
- **B 排与 A 排的示教旋转角度相反（仅基座 J1 取反）**，其余关节姿态相同

## 功能

通过 MQTT 收到种子编号命令（如 `A1-03`）后：

```
观测位 → 固定点抓取种子（灵巧手力度5抓握）→ 抬升 → 回位
→ ArUco 定位目标花篮 → 飞入花篮 → 灵巧手张开释放
→ 回到力度1保护状态 → 回观测位
```

没有 YOLO、没有点击选点——抓取点是**固化在 launch 里的固定 base 坐标**。

## 灵巧手保护（力度 1）

- 程序启动后：灵巧手写入力度 1 保护位置 `[8000×5, 5000]`，不悬空不张手
- 抓取时：力度 5 抓握 `[35000×5, 25000]`
- 放置释放后：立即回到力度 1 保护状态
- 流程失败时：同样回到力度 1

## 启动

```bash
# 终端 1：TL 驱动（必须先启动）
ros2 launch tl_driver tl_tcb605_driver.launch.py

# 终端 2：控制节点（保持运行，不要 Ctrl+C）
bash ~/Desktop/TL_Handback/scripts/run_fixed_pick_mqtt.sh
```

## 发送命令

```bash
# 用 TL_Dobot 的 Python 控制器
python3 ~/Desktop/TL_Dobot/scripts/mqtt_seed_controller.py --target A1-03
```

反馈在 `tl/robot/feedback`：`retCode` 0=成功 / 1=失败（带 message 原因）/ 2=占用。

## 花篮怎么选（targetId 规则）

**targetId 按排架规则自动解析，不需要配置映射表：**

| targetId | 花篮编号 | 目标 |
|---|---|---|
| `A1-01` | 1 | 左上第 1 个 |
| `A1-06` | 6 | 左上第 6 个 |
| `A2-01` | 7 | 左下第 1 个 |
| `B1-01` | 13 | 右上第 1 个 |
| `B2-06` | 24 | 右下第 6 个 |

解析优先级：launch 里的显式映射表（`--mqtt-target-map`，默认空）→ 排架规则解析（`A1-01` 格式）→ 都没有才自动扫描。手动选择（键盘）：`1..9` 直接选，`10..24` 按 `.` 输入。

## 关键配置（launch/tl_hand_pick_place.launch.py）

| 参数 | 当前值 | 含义 |
|---|---|---|
| `fixed_pick_base_mm` | `500.0 -250.94 58.1` | 抓取点（base 坐标，由 tool0 示教值换算） |
| `final_grasp_z_mm` | `68.937` | 最终抓取 tool0 绝对 Z |
| `--grasp-rpy-rad` | `1.839 1.239 1.343` | 抓取姿态（新臂示教值） |
| `observation_joints` | `-23.246 -14.220 53.738 4.389 26.746 -3.532` | 观测位关节角（新臂示教值） |
| `home_joint_deg` | `-23.246 -14.220 53.738 4.389 26.746 -3.532` | home 关节角（= 新观测位） |
| `--use-hand` | 启用 | 灵巧手抓取（力度 5 抓取 / 张开释放 / 力度 1 保持） |
| `--mqtt-target-map` | 空 | 显式 targetId→花篮编号映射（可选，规则解析已覆盖） |

## 花篮示教（24 个，TL_Dobot 的 teach_aruco_basket.sh）

```bash
# 左排示教（A1 上排 / A2 下排，J1 用正常方向）
TL_TEACH_START_JOINT_DEG="-23.246 -14.220 53.738 4.389 26.746 -3.532" \
bash ~/Desktop/TL_Dobot/scripts/teach_aruco_basket.sh

# 右排示教（B1 上排 / B2 下排，J1 与左排相反——把机械臂转到另一侧示教）
TL_TEACH_START_JOINT_DEG="23.246 -14.220 53.738 4.389 26.746 -3.532" \
bash ~/Desktop/TL_Dobot/scripts/teach_aruco_basket.sh
```

按键（示教程序自动选最靠近画面中心的 marker，无需按数字键）：

| 键 | 记录内容 |
|---|---|
| `B` | 记录搜索位（marker 居中时按，自动选中最近 marker） |
| `A` | 记录 approach（接近位） |
| `P` | 记录 release（放置释放位） |
| `R` | 返回搜索位，进入下一个花篮 |
| `Q` | 退出 |

> B 排示教：把机械臂旋转到右侧（J1 与 A 排相反），其余操作完全相同。
> 输出写 `~/tl_mock_pick_place/basket_aruco.yaml`，与灵巧手程序共用，示教完直接生效。

---

# 程序 2：U 型叉子叉花篮（tl_fork_teach 包）

## 功能

用 U 型叉子完成花篮的叉取—转运—放回，全程**不碰灵巧手**：

```
搜索花篮 → 接近 → 水平插入 → 竖直抬升 → 转运到高位 carry 点
→ 转运到临时位置（T）→ [按 R 时] 逆序返回：T → W → 放回 → 抽叉 → 回搜索位
```

## 两段程序

### 2a. 示教（fork_basket_teach.py）

```bash
bash ~/Desktop/TL_Handback/scripts/run_fork_basket_teach.sh
```

按键（手动拖动机械臂到位，按对应键记录）：

| 键 | 记录内容 |
|---|---|
| `1..6` | 选择花篮编号 |
| `B` | 记录当前花篮的搜索位 |
| `F` | 叉子接近位 |
| `I` | 水平插入位 |
| `L` | 竖直抬升位 |
| `W` | 携篮转运高位 waypoint |
| `T` | 临时持篮位（转运终点） |
| `O` | 观测位（固定为灵巧手检测位） |
| `R` | 返回搜索位，清空状态 |
| `Q` | 退出 |

输出：`~/tl_fork_teach/fork_basket_teach.yaml`

### 2b. 执行（fork_basket_cycle.py）

```bash
python3 -m tl_fork_teach.fork_basket_cycle \
  --teach-file ~/tl_fork_teach/fork_basket_teach.yaml \
  --state-file ~/tl_fork_teach/active_transfer.yaml \
  --allow-rejected --execute
```

按键：
- `1..6` 选择花篮 → `P`：叉取并转运到临时位（花篮留在叉子上）
- `R`：按逆序把花篮放回原位置，再回搜索位

## 特点

- 执行时**直接驱动示教关节角**（taught joints），保证路径可达
- 无 MQTT、无灵巧手、无 YOLO、无安全节点
- 中断后状态写入 `active_transfer.yaml`，重启后按阶段恢复

---

# 两个程序共同的依赖

1. **TL 驱动**：`~/Desktop/tl_robot_ros2_py-arm`（提供 /tl_driver 关节、坐标变换、ModBus 服务）
2. **D435 相机** + 手眼标定：`~/tl_handeye_data/best/handeye_best.yaml`
3. **24 个 ArUco 花篮标记**（DICT_4X4_50，边长 58mm，ID 0..23）

# 注意

- 换机械臂后：关节零点/基座坐标系/手眼标定全部可能失效，**必须重新示教 + 重新标定**
- 无保护节点，全程手边放急停
- 不要同时运行两个程序

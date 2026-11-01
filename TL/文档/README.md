# TL 机器人程序包（NANO 板运行手册）

单文件夹部署：种子（放种子）、叉花篮（修剪）、示教、MQTT 通信全部在这里。
把整个 `TL` 文件夹复制到 Jetson Nano 后，只在这个文件夹里操作。

---

## 一、目录结构

```
TL/
├── README.md                  ← 本手册（运行手册）
├── 程序说明文档.md             ← 程序原理/协议/文件详解（想了解程序怎么工作看这个）
├── scripts/                   ← 顶层一键脚本（推荐从这里启动）
│   ├── run_driver.sh          ← 启动机械臂驱动（常驻）
│   ├── run_seed_mqtt.sh       ← 种子节点（灵巧手放种子，MQTT）
│   ├── run_fork_mqtt.sh       ← 叉子修剪节点（叉花篮，MQTT）
│   ├── teach_seed.sh          ← 种子示教（192 花盆）
│   └── teach_fork.sh          ← 叉花篮示教（192 花盆）
├── TL_Handback/               ← 运行程序仓库（种子节点/叉子节点/叉子示教）
└── TL_Dobot/                  ← 示教仓库（种子示教）
```

**不在 TL 内、需单独准备：**
| 项 | 路径 | 说明 |
|---|---|---|
| 机械臂驱动 | `~/Desktop/tl_robot_ros2_py-arm` | 厂商驱动，需先编译（`TL_DRIVER_DIR` 可改路径）|
| 手眼标定文件 | `~/tl_handeye_data/best/handeye_best.yaml` | 每次标定后更新 |
| 示教输出（自动生成）| `~/tl_fork_teach/fork_basket_teach.yaml`、`~/tl_mock_pick_place/basket_aruco.yaml` | 示教时写入 |
| 实体码 | DICT_4X4_250，ID 0..191，黑方块 54mm | 192 个，全部重新打印 |

---

## 二、板载部署（第一次）

```bash
# 1) 把 TL 文件夹整体复制到 Jetson
#    例如: scp -r TL nvidia@<jetson-ip>:~/Desktop/

# 2) 安装依赖（ROS2 Humble + 必要库）
sudo apt install -y python3-paho-mqtt python3-opencv python3-yaml
pip3 install pyrealsense2 --user   # 或用 apt 的 ros-humble-realsense2-camera 依赖

# 3) 编译 tl_hand 包（种子节点/协议库所在）
cd ~/Desktop/TL/TL_Handback
source /opt/ros/humble/setup.bash
colcon build --packages-select tl_hand --symlink-install
```

---

## 三、日常运行

### 1. 启动驱动（终端 A，常驻）
```bash
bash ~/Desktop/TL/scripts/run_driver.sh
```

### 2. 启动节点（终端 B，二选一，不能同时跑）
```bash
# 种子（灵巧手放种子）
bash ~/Desktop/TL/scripts/run_seed_mqtt.sh          # 有画面（调试用）
bash ~/Desktop/TL/scripts/run_seed_mqtt_headless.sh  # 无画面（车端/生产用）

# 叉子（叉花篮修剪）
bash ~/Desktop/TL/scripts/run_fork_mqtt.sh
```
看到 `MQTT connected and subscribed: node/arm/01/down` 即就绪。
节点启动后自动回观测位（叉上有花篮时不自动动）。

### 3. 发命令测试（终端 C）
```bash
# 订阅反馈
mosquitto_sub -h 116.205.245.172 -u shapandevice -P 'qwer1234!!' \
  -t node/arm/01/up -q 1 -v -C 2

# 放种子到 A1-03（seed / seed_place 等价，反馈 cmd 回显收到的命令名）
mosquitto_pub -h 116.205.245.172 -u shapandevice -P 'qwer1234!!' \
  -t node/arm/01/down -m '{"cmd":"seed","targetId":"A1-03"}' -q 1
mosquitto_pub -h 116.205.245.172 -u shapandevice -P 'qwer1234!!' \
  -t node/arm/01/down -m '{"cmd":"seed_place","targetId":"A1-03"}' -q 1

# 叉花篮到修剪位 → 修剪 → 放回
mosquitto_pub -h 116.205.245.172 -u shapandevice -P 'qwer1234!!' \
  -t node/arm/01/down -m '{"cmd":"trim_pick","targetId":"A1-03"}' -q 1
mosquitto_pub -h 116.205.245.172 -u shapandevice -P 'qwer1234!!' \
  -t node/arm/01/down -m '{"cmd":"trim_place","targetId":"A1-03"}' -q 1

# 紧急停止（锁存，重启节点清除）
mosquitto_pub -h 116.205.245.172 -u shapandevice -P 'qwer1234!!' \
  -t node/arm/01/down -m '{"cmd":"estop"}' -q 1
```

---

## 四、MQTT 协议

| 项 | 值 |
|---|---|
| Broker | 公网 `116.205.245.172:1883`（用户名 `shapandevice` 密码 `qwer1234!!`）|
| 命令 topic | `node/arm/01/down`（车→机械臂）|
| 反馈 topic | `node/arm/01/up`（机械臂→车）|
| 命令 | `seed` / `seed_place`（完整放种周期）/ `trim_pick` / `trim_place` / `estop` |
| targetId | `A1-01`~`A4-24`、`B1-01`~`B4-24` → 花盆 1..192（码 ID=花盆号-1）|
| retCode | `0`=成功 `1`=响应命令开始动作(先发) `2`=位置被占/忙 `3`=识别/放置失败(带 message) |
| 反馈格式 | `{"cmd":"seed_place","targetId":"A1-03","retCode":0}`（cmd 回显收到的命令名，回显 targetId）|

- `seed` 与 `seed_place` 行为完全一致（固定点取种 → 定位花盆 → 放置 → 回观测位），只接受一次命令
- 完整反馈时序（4 条，命令名按阶段固定）：
  1. `{"cmd":"seed","targetId":"A1-03","retCode":1}` 收到命令，开始取种
  2. `{"cmd":"seed","targetId":"A1-03","retCode":0}` 取种完成（种子已抓稳）
  3. `{"cmd":"seed_place","targetId":"A1-03","retCode":1}` 放置响应（识别成功，开始放置）
  4. `{"cmd":"seed_place","targetId":"A1-03","retCode":0}` 放置完成（终态）
- 失败时按失败所在阶段回 `retCode:3`（取种阶段失败回 `seed:3`，放置阶段失败回 `seed_place:3`）
- 已放过种子的花盆不拒绝，重复命令照放（`retCode 2` 用于命令忙/周期未结束时）

- 每个命令两条反馈：先 `retCode:1`（收到、开始动），完成后终态 `0/2/3`
- `estop` 无 targetId；急停后其他命令全部拒绝（retCode 3），**重启节点**清除
- 改本地 broker：`TL_MQTT_HOST=127.0.0.1 TL_MQTT_USERNAME= TL_MQTT_PASSWORD= bash .../run_seed_mqtt.sh`
- 局域网远程查看：MQTTBox 连 `192.168.1.100:1883`（匿名；若拒连，Jetson 上 mosquitto 需 `listener 1883 0.0.0.0`）

---

## 五、示教

### 种子示教（192 花盆，每盆手拖，只启动时自动回一次起始位）
```bash
bash ~/Desktop/TL/scripts/teach_seed.sh
```
启动自动回默认起始位（A 侧）→ **每个花盆手拖**：拖到码清晰入镜→输入花盆号→`B`→拖到花盆上方→`A`→拖到放种位置→`P`，逐个教；
`v` 一键转 B 侧（180°）继续；`Q` 保存。示教时 ServoJ 释放，程序不主动动（仅 `R` 返回会运动）。
- 按键：`0-9` 输入号 / `B` 选中（要求码稳定可见）/ `A` 接近位 / `P` 释放位 / `v` B侧起始位 / `c` 清空 / `Q` 退出
- 起始位可改：`TL_TEACH_START_JOINT_DEG`（A 侧）、`TL_TEACH_START_JOINT_DEG_B`（B 侧）

### 叉花篮示教（192 花盆，每盆需到花篮处）
```bash
bash ~/Desktop/TL/scripts/teach_fork.sh
```
拖到花篮处 → 输入花盆号 → `B`（检测位）→ `F` 接近 → `I` 插入 → `L` 提起；
`W` 搬运中点、`T` 修剪位、`O` 观测位（全局，教一次）；`R` 返回；`Q` 保存。

---

## 六、常见问题

| 现象 | 处理 |
|---|---|
| `D435 frame timeout` / 检测 0 samples | 拔插 D435 USB（换 USB3 口），重启节点 |
| 启动提示 `Active fork transfer state found` | 叉上真有花篮→发 `trim_place`；空叉→`rm ~/tl_fork_teach/active_transfer.yaml` 重启 |
| 码识别不到/偏移巨大 | 确认码是 DICT_4X4_250 新码、54mm、贴正；示教文件是否重新教过 |
| MQTT 连不上（reason=5）| 用户名/密码错，检查 `TL_MQTT_USERNAME/TL_MQTT_PASSWORD` |
| 机械臂不动但反馈 retCode:3 | 看节点终端的 message 字段原因 |
| 急停后无法恢复 | 重启节点（estop 锁存设计）|

**安全：物理 E-stop 始终保留在手边；程序级急停（estop）只是停止运动流，不是断电。**

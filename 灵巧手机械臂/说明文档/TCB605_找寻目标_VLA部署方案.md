# 天链 TCB605 VLA 找寻目标部署方案（无灵巧手版）

> 最后更新: 2026-08-07
> 开发板: Jetson Orin Nano 16GB (JetPack 6.2, CUDA 12.6, PyTorch 2.8 GPU)
> 训练机: A800 80GB
> 任务目标: VLA 驱动机械臂末端接近目标物体表面（30mm 内），不含抓取
> 前置: tl_driver ROS2 驱动已编译运行，D435 眼在手安装完毕

---

## 目录

1. [系统架构](#1-系统架构)
2. [硬件清单](#2-硬件清单)
3. [动作空间设计](#3-动作空间设计)
4. [数据采集](#4-数据采集)
5. [格式转换](#5-格式转换)
6. [模型训练](#6-模型训练)
7. [Jetson 推理部署](#7-jetson-推理部署)
8. [安全机制](#8-安全机制)
9. [故障排查](#9-故障排查)
10. [实施时间线](#10-实施时间线)

---

## 1. 系统架构

### 1.1 总体架构

```
┌─────────────────────────────────────────────┐
│              Jetson Orin Nano 16GB            │
│                                              │
│  ┌──────────┐  192.168.1.13:6001/7000        │
│  │ tl_driver │ ←────────────────────────────│── TCB605 控制器
│  │ (ROS2)   │   TCP 自定义协议               │
│  └────┬─────┘                                │
│       │ ROS2 Topic                           │
│  ┌────┴──────────────┐                       │
│  │ VLA 推理 + 执行    │                       │
│  │ tcb_find_policy_   │                       │
│  │   node.py          │                       │
│  └────┬──────────────┘                       │
│       │                                      │
│  ┌────┴──────┐                               │
│  │ D435 相机  │  眼在手上 (USB 3.0)           │
│  │ 640×480   │                               │
│  └───────────┘                               │
└─────────────────────────────────────────────┘
```

> 无灵巧手、无夹爪、无 ModBus 透传。架构比抓取版大幅简化。

### 1.2 通信协议

| 设备 | 接口 | 参数 |
|------|------|------|
| TCB605 | TCP 自定义 | 192.168.1.13，命令端口 6001，数据端口 7000 |
| D435 | USB 3.0 | pyrealsense2，640×480@30fps |
| 示教器 | 拖拽模式 | P 键进入/退出 |

### 1.3 VLA 推理流程

```
D435 图像 + 当前 TCP 位姿 + 任务指令("接近红色方块")
        │
        ▼
┌───────────────────┐
│ SmolVLA / ACT     │  ← Jetson 本地推理
│ (~450M / ~10M)    │
└───────┬───────────┘
        │ 输出: [dx, dy, dz]  (mm，基坐标系相对位移)
        ▼
┌───────────────────┐
│ 安全过滤器         │  ← 限幅 + Z轴保护 + 步数上限
└───────┬───────────┘
        │
        ▼
┌───────────────────┐
│ MoveL 绝对位姿     │  ← /tl_driver/moveL 话题
│ target = tcp+action│   coord=1 直角系
└───────────────────┘
```

---

## 2. 硬件清单

| 设备 | 型号/参数 | 用途 |
|------|----------|------|
| 机械臂 | 天链 TCB605，6轴，负载5kg | 运动执行 |
| 驱动 | ROS2 Humble tl_driver（ARM版） | 通信与控制 |
| 深度相机 | Intel RealSense D435 | 眼在手上视觉 |
| 开发板 | Jetson Orin Nano 16GB | 采集+推理 |
| 训练机 | NVIDIA A800 80GB | 模型训练 |

> 末端只装 D435，无灵巧手无夹爪。本方案只做"找寻接近"，不做抓取。

---

## 3. 动作空间设计

### 3.1 动作空间（3维）

```
action = [dx, dy, dz]

参数说明:
  dx, dy, dz : TCP 在基坐标系下的相对位移 (mm)
```

### 3.2 设计理由

- **找寻目标只需平移**：眼在手上、相机俯视，末端接近目标表面只需 XYZ 平移，姿态保持不变
- **3维降低学习难度**：相比 7维（含旋转+手部），3维 action 训练数据需求少、收敛快
- **执行简单**：target = current_tcp_xyz + action，姿态沿用当前值，直接 MoveL

### 3.3 成功标准

末端 TCP 的 Z 坐标距离目标表面 ≤ 30mm，且 XY 偏差 ≤ 20mm。

### 3.4 不含姿态的原因

眼在手上 + 相机俯视场景，末端姿态恒定（朝下）。若后续需要姿态调整，扩展为：
```
action = [dx, dy, dz, drx, dry, drz]  # 6维
```

---

## 4. 数据采集

### 4.1 采集原理

**拖拽示教（Kinesthetic Teaching）**：人手拖机械臂从观测位移动到目标上方，再下降接近目标表面（30mm内），脚本以 8fps 同步记录图像和位姿，事后差分生成 action 标签。

```
pose[t]   = [x, y, z]        ← /tcp_pose 的 position
pose[t+1] = [x', y', z']
action[t] = pose[t+1] - pose[t] = [dx, dy, dz]
```

> 只取 position 差分，忽略 rpy（姿态恒定）。

### 4.2 采集软件

新建文件：`tcb_find_collect.py`，放在 `~/Desktop/hand_demo_arm/`。

**按键说明：**

| 按键 | 功能 |
|------|------|
| r | 开始录制当前 episode（机械臂先到观测位） |
| f | 结束录制，保存为成功 episode |
| x | 废弃当前 episode |
| q | 退出程序 |

**采集一条 episode 流程：**

```
步骤1: 程序启动 → 机械臂自动 MovJ 回观测位
步骤2: 确认 D435 画面能看到桌面目标
步骤3: 按 r → 开始录制
步骤4: 按示教器 P 键 → 进入拖拽模式
步骤5: 手拖机械臂靠近目标上方 → 继续下降到接近目标表面（30mm内）
步骤6: 按示教器 P 键 → 退出拖拽模式
步骤7: 按 f → 保存为成功 episode
```

> 无灵巧手，采集流程比抓取版简单——只需拖动到位，无手部控制、无 ModBus。

### 4.3 每条 episode 记录的数据

```
episode_000001/
├── episode.json             # {task, frames, success, create_time}
├── metadata.jsonl           # 逐帧元数据
├── actions.jsonl            # 动作标签（build_actions 生成）
└── rgb/
    ├── rgb_000000.jpg
    ├── rgb_000001.jpg
    └── ...
```

> 不采集 depth（ACT/SmolVLA 只用 RGB，省存储）。如需多模态扩展再补。

**metadata.jsonl 每行格式：**

```json
{
  "frame_index": 0,
  "timestamp_ns": 1785990000000000000,
  "rgb": "rgb/rgb_000000.jpg",
  "tcp_pose": [x, y, z, rx, ry, rz],
  "joint_angle": [j1, j2, j3, j4, j5, j6],
  "task": "接近红色方块",
  "event": "start"
}
```

> tcp_pose 单位：mm（xyz） + 弧度（rpy），与 /tcp_pose 话题一致。

### 4.4 采集参数

```python
RECORD_FPS = 8
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
OBSERVE_JOINTS = [0.0, -28.5, -91.8, 0.0, -59.5, 0.0]  # 示例，需实测确认可达
TASK = "接近红色方块"
```

### 4.5 采集数量

| 阶段 | 数量 | 目的 |
|------|------|------|
| 调试 | 5 条 | 验证采集→action→转换链路 |
| 基线 | 50 条 | ACT policy 基线 |
| 稳定 | 100-200 条 | SmolVLA 微调 |
| 鲁棒 | 300+ 条 | 泛化 |

### 4.6 注意事项

1. 每次从观测位开始，保证初始状态一致
2. 物体位置变化（±20cm），第一批变化不要太大
3. 轨迹平滑，不犹豫不急停
4. 终止位必须是末端距目标表面 ≤30mm
5. 相机支架不能松动，松动后旧数据全部失效
6. 每天采集前录 1 条测试并回放检查

---

## 5. 格式转换

### 5.1 原始 → LeRobot Dataset

转换脚本 `convert_to_lerobot_find.py` 在 Jetson 上执行：

```bash
cd ~/Desktop/hand_demo_arm
python3 convert_to_lerobot_find.py \
  --data-dir ~/tcb_find_data/raw \
  --output-dir ~/tcb_find_data/lerobot_dataset \
  --task "接近红色方块"
```

### 5.2 字段映射

| TCB 原始字段 | LeRobot Dataset 字段 | 说明 |
|-------------|---------------------|------|
| rgb/rgb_xxxxxx.jpg | observation.images.wrist | 腕部相机 RGB |
| tcp_pose + joint_angle | observation.state | 观测状态（归一化） |
| actions.jsonl 的 [dx,dy,dz] | action | 动作标签（归一化） |
| task | task | 任务描述 |
| episode 目录名 | episode_index | 回合编号 |
| frame_index | frame_index | 帧序号 |

### 5.3 归一化（关键，不可省）

observation.state 混合了 mm（tcp xyz）、弧度（tcp rpy）、度（joint），量级差异大，必须归一化：

```python
# 每维单独标准化（mean/std 存到 info.json）
state_mean = [...]   # 12维
state_std = [...]
action_mean = [0.0, 0.0, 0.0]      # action 均值约0
action_std = [10.0, 10.0, 10.0]    # 单步位移标准差

# 归一化后存入 parquet
state_norm = (state - state_mean) / state_std
action_norm = (action - action_mean) / action_std
```

> 不归一化会导致 loss 被大量级（mm 的几百）主导，训练发散。

### 5.4 上传 HuggingFace

```bash
huggingface-cli login
huggingface-cli upload <your_username>/tcb605_find_target \
  ~/tcb_find_data/lerobot_dataset \
  --repo-type dataset
```

---

## 6. 模型训练

### 6.1 训练环境（A800）

```bash
conda create -n lerobot python=3.12 -y
conda activate lerobot
git clone https://github.com/huggingface/lerobot.git
cd lerobot
git checkout <锁定具体commit>   # 重要：锁定版本，训练机与 Jetson 一致
pip install -e ".[smolvla]"
pip install wandb
```

### 6.2 两步走

```
第一步: ACT Policy (基线验证)
  目的: 验证数据格式、动作空间、训练框架
  模型: ~10M 参数
  时间: 30000 步，约 2-4 小时
  作用: 数据没问题后再上 SmolVLA

第二步: SmolVLA 微调
  目的: 语言指令驱动的自主找寻
  模型: ~450M 参数（不是1-2B）
  时间: 50000 步，约 5-10 小时
  前提: ACT 基线跑通
```

### 6.3 ACT 训练命令

```bash
# 训练前确认参数名（LeRobot 版本可能变）
lerobot-train --help | grep -E "policy|dataset|batch"

python lerobot/scripts/train.py \
  --policy.path=act \
  --dataset.repo_id=<your_username>/tcb605_find_target \
  --output_dir=outputs/train/tcb605_find_act \
  --job_name=tcb605_find_act \
  --batch_size=64 \
  --steps=30000 \
  --device=cuda
```

### 6.4 SmolVLA 训练命令

```bash
python lerobot/scripts/train.py \
  --policy.path=lerobot/smolvla_base \
  --dataset.repo_id=<your_username>/tcb605_find_target \
  --output_dir=outputs/train/tcb605_find_smolvla \
  --job_name=tcb605_find_smolvla \
  --batch_size=8 \
  --steps=50000 \
  --device=cuda
```

> A800 80GB，SmolVLA batch=8 没问题。

### 6.5 训练监控

```
✅ loss 持续下降（前5000步快降，后平稳）
✅ 每1000步 eval: action_mse < 0.1 (ACT)
✅ 无 NaN / 梯度爆炸
✅ checkpoint 约 2GB（SmolVLA）/ 50MB（ACT）
```

### 6.6 导出 checkpoint

```bash
cd outputs/train/tcb605_find_smolvla
tar -czf tcb605_find_smolvla.tar.gz checkpoints/last/
scp tcb605_find_smolvla.tar.gz nvidia@<Jetson_IP>:~/tcb_find_models/
```

---

## 7. Jetson 推理部署

### 7.1 ⚠ 关键：不要用 conda 装 LeRobot

**这是 Jetson 部署 VLA 最大的坑**：`pip install lerobot` 会拉 CPU 版 PyTorch，覆盖系统的 jetson-ai-lab GPU wheel，导致推理慢 100 倍。

**正确做法**：用系统 Python 3.10 + jetson-ai-lab PyTorch，LeRobot 用 `--no-deps` 装：

```bash
# 1. 确认系统 PyTorch GPU 正常
python3 -c "import torch; print(torch.cuda.is_available())"  # 必须 True

# 2. LeRobot 用 --no-deps 装，避免覆盖 PyTorch
pip3 install --no-deps lerobot
pip3 install datasets pyarrow transformers accelerate einops

# 3. 再次验证 PyTorch 没被覆盖
python3 -c "import torch; print(torch.cuda.is_available())"  # 仍需 True

# 4. 解压 checkpoint
mkdir -p ~/tcb_find_models && cd ~/tcb_find_models
tar -xzf tcb605_find_smolvla.tar.gz
```

### 7.2 推理架构（两进程分离）

```
进程1: tcb_find_infer_server.py  (HTTP 推理服务，系统 Python 3.10)
  → 加载 SmolVLA/ACT checkpoint
  → 监听 http://127.0.0.1:18080/predict
  → 接收 image+state → 推理 → 返回 [dx,dy,dz]

进程2: tcb_find_policy_node.py   (ROS2 执行节点，系统 Python 3.10)
  → 订阅 D435 图像 + /tcp_pose
  → POST 到推理服务
  → 安全过滤 + 发布 /tl_driver/moveL
```

> 两进程都用系统 Python 3.10（不分裂版本），通过 HTTP 解耦避免 LeRobot 依赖污染 ROS2 环境。

### 7.3 启动推理服务

```bash
# 终端1
python3 ~/Desktop/hand_demo_arm/tcb_find_infer_server.py \
  --model-path ~/tcb_find_models/checkpoints/last \
  --host 127.0.0.1 --port 18080
```

### 7.4 启动执行节点

**先 dry-run（只打印不动）：**

```bash
# 终端2
source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
python3 ~/Desktop/hand_demo_arm/tcb_find_policy_node.py \
  --task "接近红色方块" \
  --dry-run
```

**确认动作合理后执行：**

```bash
python3 ~/Desktop/hand_demo_arm/tcb_find_policy_node.py \
  --task "接近红色方块" \
  --execute \
  --max-step-mm 10.0 \
  --loop-hz 1.0
```

> 第一轮执行时手放在急停按钮附近！

### 7.5 推理性能预估

| 指标 | ACT | SmolVLA |
|------|-----|---------|
| 单步推理 | ~50ms | ~500-1000ms |
| 安全执行频率 | 2-5 Hz | 1-2 Hz |
| 显存占用 | ~1GB | ~4GB |
| 一次找寻（10-20步） | 5-10s | 15-30s |

---

## 8. 安全机制

### 8.1 硬限制（代码内嵌）

```python
MAX_STEP_MM = 20.0      # 单步最大平移 20mm
MAX_STEP_Z_MM = 10.0    # 单步 Z 轴最大 10mm
MIN_Z_MM = 30.0         # 最低 Z 高度 30mm（防撞桌面）
MOVE_VELOCITY = 15.0    # MoveL 速度 15%
MAX_STEPS = 30          # 单次找寻最多 30 步（防无限循环）
MAX_CONSECUTIVE_ERRORS = 3  # 连续3次异常 → Stop
```

### 8.2 运行前检查

```
□ 急停按钮可用
□ 机械臂周围无人
□ 工作台无杂物
□ D435 固定牢靠
□ 先 dry-run 确认动作合理
□ 再低速 execute 实测
```

### 8.3 成功判定

末端 TCP 距目标表面 ≤30mm 且 XY 偏差 ≤20mm → 判定找寻成功，停止运动。

---

## 9. 故障排查

| 现象 | 可能原因 | 解决 |
|------|---------|------|
| 采集时 /tcp_pose 无数据 | 驱动未上电 | `ros2 service call /tl_driver/power_on std_srvs/srv/Trigger` |
| 拖拽模式进不去 | 示教器模式不对 | 示教器切到"示教"模式再按 P |
| D435 无画面 | USB 断开 | 重插 USB 3.0 |
| 推理返回零动作 | 推理服务骨架模式 | 检查 --real-policy 参数 |
| MoveL 不动 | 目标点不可达 | 减小 --max-step-mm |
| 推理极慢（<0.1Hz） | PyTorch 装成 CPU 版 | 重装 jetson-ai-lab GPU wheel |
| 训练 loss 不降 | action 单位/归一化有误 | 检查 actions.jsonl + 归一化 |
| 模型输出异常大 | 推理环境与训练不一致 | 确认 Python/PyTorch/LeRobot 版本一致 |

---

## 10. 实施时间线

```
Day 1-2 ── 数据采集 + 环境准备
  采集 5 条调试 → 验证链路
  采集 50 条成功 episode
  生成 action + 转 LeRobot + 上传

Day 3 ── 模型训练
  A800 安装 LeRobot（锁定版本）
  ACT 基线训练（2-4h）+ 离线验证
  SmolVLA 微调（5-10h，可过夜）

Day 4-5 ── 部署 + 测试
  Jetson 安装推理环境（--no-deps）
  启动推理服务 + dry-run 验证
  低速真实执行测试
  调试 + 迭代数据
```

> 首次部署建议 5 天，留足调试时间。

---

## 附录 A: 参数速查

| 参数 | 值 |
|------|-----|
| TCB605 IP | 192.168.1.13 |
| 命令端口 | 6001 |
| 数据端口 | 7000 |
| D435 分辨率 | 640×480 @ 30fps |
| 采集 FPS | 8 |
| 推理 FPS | 1-2（SmolVLA）/ 2-5（ACT） |
| 单步最大平移 | 20mm |
| 单步最大 Z 轴 | 10mm |
| 最低 Z 高度 | 30mm |
| MoveL 速度 | 15% |
| 动作维度 | 3 [dx,dy,dz] |
| 观测维度 | 12 [tcp_xyz, tcp_rpy, j1-j6] |

## 附录 B: 代码文件清单

| 文件名 | 功能 | 位置 |
|--------|------|------|
| tcb_find_collect.py | 拖拽示教采集 | ~/Desktop/hand_demo_arm/ |
| build_actions_find.py | 差分生成动作标签 | 同上 |
| convert_to_lerobot_find.py | 转 LeRobot Dataset（含归一化） | 同上 |
| tcb_find_infer_server.py | HTTP 推理服务 | 同上 |
| tcb_find_policy_node.py | ROS2 执行节点 | 同上 |

## 附录 C: 关键接口（tl_driver）

| 功能 | 机制 | 接口 |
|------|------|------|
| 获取 TCP 位姿 | 话题 | /tcp_pose（CartesianPose，xyz=mm，rpy=弧度） |
| 获取关节角 | 话题 | /joint_states（JointState） |
| 笛卡尔运动 | 话题 | /tl_driver/moveL（MoveCommand，coord=1） |
| 关节运动 | 话题 | /tl_driver/moveJ（MoveCommand，coord=0） |
| 上电 | 服务 | /tl_driver/power_on（std_srvs/Trigger） |
| 清除错误 | 服务 | /tl_driver/clear_error（std_srvs/Trigger） |

> MoveL 的 target_pos_value 前6位 = [x,y,z,rx,ry,rz]，coord=1 直角系，velocity 控制速度。
> 注意：rpy 是**弧度**不是度，执行端构造 target 时要保持单位一致。

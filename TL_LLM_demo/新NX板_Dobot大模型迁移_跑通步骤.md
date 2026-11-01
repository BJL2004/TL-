# 新 NX NANO 板 · Dobot 视觉抓取 + 大模型迁移跑通步骤

> 目标板：新 Jetson Orin NX/Nano（已装好 torch/CUDA/YOLO/RealSense/Ollama 环境）
> 示例地址：`nvidia@172.20.10.7`，工程目录 `~/Desktop/Dobot_Vision_Grasp`
> 目标：把 Windows 上验证通过的 `LLM → 安全层 → MQTT → YOLO 抓取 → VLM 参谋` 原样跑通。

---

## 0. 迁移前确认

1. **新板接的是同一台 Dobot + 同一套 D435i 装夹吗？**
   - 是：`hand_eye_calibration_result.npz`、`weights/best.pt`、`PICK_READY_POSE` 可以直接沿用。
   - 不是：标定必须重做，`PICK_READY_POSE` 必须重新示教/低速验证。
2. **物理急停在手边，工作区清空。**
3. Windows 上已确认能跑通（本次已验证抓取/放置/语音）。

---

## 1. 板端环境验收

SSH 到新板：

```bash
ssh nvidia@172.20.10.7
```

逐项检查：

```bash
cat /etc/nv_tegra_release          # R36 = JetPack 6
python3 -V                         # 3.10.x

python3 -c "import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available())"
# 期望 2.5.0a0+...nv24.08 12.6 True

python3 -c "import torchvision; print(torchvision.__version__)"
# 0.20.0

python3 -c "import cv2, numpy, ultralytics, pyrealsense2, paho.mqtt; print('deps ok')"
# 不报错即可

python3 -c "import pyrealsense2 as rs; print('devices:', len(rs.context().query_devices()))"
# 相机插好应为 1

ollama --version
systemctl is-active ollama
ollama list
ollama ps
```

Ollama 应能看到：

```text
phi3:mini
qwen2.5vl:3b
```

`ollama ps` 的 `PROCESSOR` 应有 GPU。

> 如果 Ollama 还没配好，按《老板子Ollama安装与模型部署说明》第 2-4 节补装/启动；
> 新板已经有 `~/Downloads/ollama-linux-arm64*.tar.zst` 的话直接解压即可。

---

## 2. 从 Windows 拷工程到新板

### 2.1 板端先备份旧目录（如果有）

```bash
ls -d ~/Desktop/Dobot_Vision_Grasp 2>/dev/null && \
mv ~/Desktop/Dobot_Vision_Grasp ~/Desktop/Dobot_Vision_Grasp.bak_$(date +%Y%m%d_%H%M)
```

### 2.2 Windows PowerShell 拷贝

```powershell
scp -r "D:\work\engineering_project\Dobot_Nova2\Dobot_Vision_Grasp" `
  nvidia@172.20.10.7:~/Desktop/
```

如果提示目录已存在/出现嵌套目录，说明板端没备份干净；先删掉/改名再拷。

### 2.3 板端检查文件

```bash
cd ~/Desktop/Dobot_Vision_Grasp
ls
ls yolov5_vision_grasp
ls llm_agent
ls scripts

chmod +x scripts/*.sh

python3 -m py_compile \
  yolov5_vision_grasp/mqtt_yolo_grasp.py \
  llm_agent/llm_grasp_agent.py \
  llm_agent/mock_executor.py
```

必须存在：

```text
yolov5_vision_grasp/weights/best.pt
yolov5_vision_grasp/mqtt_yolo_grasp.py
llm_agent/llm_grasp_agent.py
scripts/run_executor.sh
scripts/run_agent.sh
hand_eye_calibration_result.npz
```

---

## 3. 板端依赖确认（不要重装已有环境）

```bash
cd ~/Desktop/Dobot_Vision_Grasp

python3 -c "import torch; print('torch', torch.__version__, 'cuda', torch.cuda.is_available())"
python3 -c "import cv2, numpy, ultralytics, pyrealsense2, paho.mqtt; print('deps ok')"
```

- 新板环境已装好，**不要**再 `pip install torch/torchvision/ultralytics`，避免覆盖 CUDA 版。
- 如果只缺 `paho-mqtt`：

```bash
pip3 install paho-mqtt
```

---

## 4. 标定与位姿（迁移最关键）

### 4.1 标定

- 同一套相机+夹爪、装夹没动：直接用工程里的 `hand_eye_calibration_result.npz`。
- 拆装过相机/换过机器人：重新标定：

```bash
cd ~/Desktop/Dobot_Vision_Grasp/EyeInHand
python3 data_collect_in_dobot.py
python3 hand_in_eye_calibrate_dobot.py
python3 calibration_accuracy_test.py
```

### 4.2 位姿

`mqtt_yolo_grasp.py` 顶部已包含 Windows 实测值：

```text
DEFAULT_POSE    = [101.6937, -306.8226, 531.3492, -179.5421, 1.1446, -42.3958]
PICK_READY_POSE = [2.1051, -325.0033, 529.4345, -179.2598, 1.3728, -60.2370]
PLACE_POSE_1    = [-13.6281, -358.6630, 314.1713, -177.0779, 0.1696, -69.6481]
```

- **同一台机器人**：可直接用。
- **另一台机器人**：先保留/改回板子原来的示教值；第一次运动前用低速（20%）验证，超限位会报 `-2`。
- 原 `PICK_READY` `[356.4355, ...]` 在 Windows 这台机器人上触发过关节软限位，不要无条件沿用。

> 检查当前位姿/报警（只读）：
> ```bash
> cd ~/Desktop/Dobot_Vision_Grasp
> python3 - <<'PY'
> import sys, time
> sys.path.insert(0, 'EyeInHand')
> from dobot_api import DobotApiDashboard
> d = DobotApiDashboard('192.168.5.1', 29999)
> time.sleep(0.5)
> print('pose:', d.GetPose())
> print('mode:', d.RobotMode())
> print('err :', d.GetErrorID())
> d.close()
> PY
> ```
> `RobotMode=5` 是“使能且空闲”，`9` 是报警。

---

## 5. 网络与 MQTT

### 5.1 机械臂网络

```bash
ip a
ping -c 3 192.168.5.1
```

板子以太网需要配 `192.168.5.x/24`，不要和 Wi-Fi 的 `172.20.10.x` 冲突。

### 5.2 本地 broker（推荐）

```bash
sudo apt install -y mosquitto mosquitto-clients
sudo systemctl enable --now mosquitto
ss -ltnp | grep 1883
```

> 如果使用现有服务器，设置 `DOBOT_MQTT_HOST=<服务器IP>` 和账号即可，不装本地 broker。

演示阶段统一用：

```text
node/arm/99/board/down
node/arm/99/board/up
```

---

## 6. 启动执行端

确保：机械臂已上电/使能、相机插 USB3、工作区清空、物理急停在手。

```bash
cd ~/Desktop/Dobot_Vision_Grasp/scripts
./run_executor.sh
```

默认连接 `127.0.0.1:1883`，主题 `node/arm/99/board/down|up`。
需要换 broker：

```bash
DOBOT_MQTT_HOST=192.168.38.213 \
DOBOT_MQTT_USERNAME=device \
DOBOT_MQTT_PASSWORD=device1212 \
DOBOT_MQTT_TOPIC_DOWN=node/arm/down \
DOBOT_MQTT_TOPIC_UP=node/arm/up \
./run_executor.sh
```

启动过程：

1. 连接 Dobot、清报警、使能
2. 初始化 RealSense
3. 加载 YOLO（CUDA）
4. **自动回 `DEFAULT_POSE`**
5. 连接 MQTT，打印：

```text
🎮 系统就绪，等待MQTT指令...
```

看到这行再继续。

---

## 7. 启动 LLM agent 并分阶段验收

### 7.1 离线自检

```bash
cd ~/Desktop/Dobot_Vision_Grasp/scripts
./run_agent.sh --selftest
```

### 7.2 dry-run（不动机械臂）

```bash
./run_agent.sh --once "抓蓝色方块"
```

应显示 `grasp blue`，但**不发送 MQTT**。

### 7.3 第一次真机（加 --confirm）

```bash
./run_agent.sh --execute --confirm --once "抓蓝色方块"
```

确认：

- 机械臂到观测位、检测到蓝色；
- 回执 `result_code=0`；
- `~/dobot_llm_agent/frames/latest.jpg` 生成；
- agent 有 VLM 画面报告。

### 7.4 放置

```bash
./run_agent.sh --execute --confirm --once "放下"
```

### 7.5 交互式对话

```bash
./run_agent.sh --execute
```

可输入：

```text
抓蓝色方块
把绿色方块放到放置点
放下
看一眼
急停
退出
```

> agent 会记录持有状态：已持有同色时说“把X放到放置点”会自动按“放置”执行，不会重复抓取；
> 持有异色会被拒绝，提示先说“放下”。输入“状态”可查看；如与实际不符，加 `--reset-state` 或说“放下”同步。

> 默认直接执行；想逐条确认就加 `--confirm`。
> 真机运行时物理急停必须在手边。

### 7.6 语音（可选，Vosk 中文小模型）

Jetson 上不用 faster-whisper（CTranslate2 4.x 没有 aarch64 轮子），用 Vosk：

```bash
sudo apt install -y libportaudio2 unzip
pip3 install --user vosk sounddevice

mkdir -p ~/dobot_llm_agent/models
cd ~/dobot_llm_agent/models

# 国内可用镜像（也可用 alphacephei.com 官方源，支持断点续传）
wget -c "https://hf-mirror.com/guloooovoooo/vosk-model-small-cn/resolve/main/vosk-model-small-cn.zip"
unzip vosk-model-small-cn.zip
ls
```

如果解压出的目录名不是 `vosk-model-small-cn-0.22`：

```bash
mv vosk-model-small-cn ~/dobot_llm_agent/models/vosk-model-small-cn-0.22
```

检查麦克风：

```bash
arecord -l
python3 -c "import sounddevice as sd; print(sd.query_devices(kind='input'))"
```

没有输入设备就插 USB 麦克风。

先测识别：

```bash
cd ~/Desktop/Dobot_Vision_Grasp/llm_agent
python3 llm_grasp_agent.py --voice-test --voice-backend vosk
# 说：抓蓝色方块
```

板端脚本运行：

```bash
cd ~/Desktop/Dobot_Vision_Grasp/scripts
bash run_agent.sh --execute --voice --voice-backend vosk
```

进入 `>` 后直接回车说话即可。

---

## 8. 常见问题

| 现象 | 处理 |
|---|---|
| scp 后目录嵌套 | 板端先把旧 `Dobot_Vision_Grasp` 改名，再重新拷 |
| `torch.cuda.is_available()` False | 确认装的是 NVIDIA Jetson wheel，不是 PyPI 的 torch |
| `import cv2` 报 NumPy 错误 | 固定 `numpy<2`，只保留一个 opencv 包 |
| `pyrealsense2` 设备 0 | USB3 口/线、udev 规则、`lsusb` |
| Ollama 显示 CPU | 确认解压了 `jetpack6` 包、服务重启、`journalctl -u ollama` |
| MQTT 连接失败 | broker 未启动/IP/账号/防火墙；执行端和 agent 必须同一 broker/主题 |
| 执行端启动后 `-2` | 位姿关节限位/报警；停程序、ClearError、低速验证位姿 |
| 抓取偏 | 手眼标定或相机装夹变了；重新标定 |
| VLM 提示没有快照 | 执行端没检测到目标，或 `--frame-dir` 不一致 |
| 语音识别为空 | 麦克风/阈值；`--voice-test` 单独测，`--voice-threshold` 调整 |
| agent 等不到回执 | 执行端没启动、主题不一致、taskid 不匹配 |

---

## 9. 长期运行建议

```bash
sudo apt install -y tmux

# 执行端
tmux new -s executor
cd ~/Desktop/Dobot_Vision_Grasp/scripts && ./run_executor.sh
# Ctrl+B D 退出

# agent
tmux new -s agent
cd ~/Desktop/Dobot_Vision_Grasp/scripts && ./run_agent.sh --execute
```

Ollama 和 Mosquitto 已由 systemd 管理；执行端/agent 不建议直接做成 systemd（需要人工确认现场安全）。

---

## 10. 验收清单

| # | 检查 | 预期 |
|---|---|---|
| 1 | torch CUDA | True |
| 2 | torchvision | 0.20.0 |
| 3 | pyrealsense2 devices | 1 |
| 4 | ollama list | phi3:mini + qwen2.5vl:3b |
| 5 | ollama ps | GPU |
| 6 | `--selftest` | 全部通过 |
| 7 | dry-run 抓蓝色 | 解析 `grasp blue`，不动 |
| 8 | `--execute --confirm` 抓蓝色 | 回执 0，VLM 报告 |
| 9 | 放下 | 回执 0，回 DEFAULT |
| 10 | 空手放下 | 回执 9 |
| 11 | 急停 | 回执 0，物理急停优先 |
| 12 | 快照文件 | `latest.jpg` 存在 |

全部通过后，再把主题从演示主题切到真实主题，接入车端/上位机。

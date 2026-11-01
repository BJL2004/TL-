# Dobot 视觉抓取迁移到 NANO 板 · 操作清单

> 适用：老板子（已有 torch / CUDA / YOLO / RealSense 环境，只缺 Ollama 和大模型程序）
> 新板：先按《老板子Ollama安装与模型部署说明》装齐 Ollama/模型，再执行本清单第 2 步以后。
>
> 目标：把 Windows 上验证通过的
> `LLM → 安全层 → MQTT → mqtt_yolo_grasp.py → Dobot → VLM 参谋`
> 原样迁到板子上，并分阶段验收。

---

## 0. 迁移前必须知道的三件事

1. **`PICK_READY_POSE` 是示教位，不是通用常量。**
   - Windows 实测值：`[2.1051, -325.0033, 529.4345, -179.2598, 1.3728, -60.2370]`
   - 原值 `[356.4355, 99.1722, 459.5047, 178.6173, 3.4392, 42.1047]` 在 Windows 这台机器人上触发关节软限位。
   - 板子如果接的是**同一台机器人/同一装夹**，可以用 Windows 实测值；
     如果是另一台机器人，**先用板子原来的示教值**，低速验证，不要直接覆盖。

2. **`hand_eye_calibration_result.npz` 是装夹快照。**
   - 同一套相机+夹爪、装夹没动：可以沿用板子现有的 npz，**不要覆盖**。
   - 拆过相机、换过机器人：必须重新标定。

3. **大模型不输出坐标/轨迹。**
   抓取位置、深度、手眼变换全部由执行端算法完成；迁移时不需要改算法，只需要把程序和新版执行端带过去。

---

## 1. 文件清单

### 必须复制到板子

| 文件/目录 | 说明 |
|---|---|
| `llm_agent/` 整个目录 | agent、mock、requirements-voice、README |
| `yolov5_vision_grasp/mqtt_yolo_grasp.py` | 新版：VLM 快照钩子、MQTT estop、`DOBOT_MQTT_*` 环境变量、EOF 保护、实测 PICK_READY |

### 不要覆盖

| 文件 | 原因 |
|---|---|
| `hand_eye_calibration_result.npz` | 板子现场的标定 |
| `weights/best.pt` | 板子现有模型 |
| `yolo_vision_grasp.py` / `yolo_vision_grasp_v2.py` | 板子现场示教位姿可能不同 |
| `PLACE_POSE_1/2`、`PICK_READY_POSE` 的现场值 | 示教值，按机器人实际 |

### Windows 端复制命令（PowerShell）

```powershell
$BOARD = "nvidia@<板子IP>"

# 1) 整个 agent 目录
scp -r "D:\work\engineering_project\Dobot_Nova2\Dobot_Vision_Grasp\llm_agent" `
  "${BOARD}:~/Desktop/Dobot_Vision_Grasp/"

# 2) 新版执行端先传成 .new，避免直接覆盖
scp "D:\work\engineering_project\Dobot_Nova2\Dobot_Vision_Grasp\yolov5_vision_grasp\mqtt_yolo_grasp.py" `
  "${BOARD}:~/Desktop/Dobot_Vision_Grasp/yolov5_vision_grasp/mqtt_yolo_grasp.new.py"

# 3) 可选：把 Ollama 说明和真机记录也带过去
scp "C:\Users\DragonAfterAll\Desktop\老板子Ollama安装与模型部署说明.md" `
  "C:\Users\DragonAfterAll\Desktop\Dobot真机联调记录_2026-10-21.md" `
  "${BOARD}:~/Desktop/"
```

---

## 2. 板端备份与位姿合并（关键步骤）

```bash
cd ~/Desktop/Dobot_Vision_Grasp/yolov5_vision_grasp

cp mqtt_yolo_grasp.py mqtt_yolo_grasp.py.bak_$(date +%Y%m%d_%H%M)

echo "==== 板子现有位姿 ===="
grep -n "DEFAULT_POSE\|PICK_READY_POSE\|PLACE_POSE_1\|PLACE_POSE_2" mqtt_yolo_grasp.py

echo "==== 新文件位姿 ===="
grep -n "DEFAULT_POSE\|PICK_READY_POSE\|PLACE_POSE_1\|PLACE_POSE_2" mqtt_yolo_grasp.new.py

echo "==== 差异 ===="
diff -u mqtt_yolo_grasp.py mqtt_yolo_grasp.new.py | head -120
```

### 情况 A：板子接的是同一台机器人（推荐用法）

把新文件里的 `PICK_READY_POSE` 保留 Windows 实测值，其余位姿按板子现场值：

```bash
nano mqtt_yolo_grasp.new.py
# 只修改/确认：
# DEFAULT_POSE / PLACE_POSE_1 / PLACE_POSE_2 用板子原来示教值
# PICK_READY_POSE 可用 Windows 实测值（同一台机器人）
```

### 情况 B：板子接的是另一台机器人

**保留板子原来的 `PICK_READY_POSE`**，只把新版代码增量合并进板子的文件：

需要合并的增量只有 4 处：

1. 顶部 MQTT 配置支持 `DOBOT_MQTT_*` 环境变量（`os.environ.get`）
2. `on_mqtt_message` 里的 `estop` 分支
3. `_save_vlm_frame()` 方法 + 检测到目标后的调用
4. `run()` 里 `input()` 的 `EOFError` 保护

合并后校验：

```bash
python3 -m py_compile mqtt_yolo_grasp.new.py
cp mqtt_yolo_grasp.new.py mqtt_yolo_grasp.py
```

---

## 3. 板端依赖检查（不要重装 torch/CUDA）

```bash
cd ~/Desktop/Dobot_Vision_Grasp

python3 -c "import torch; print('torch', torch.__version__, 'cuda', torch.cuda.is_available())"
python3 -c "import cv2, numpy, ultralytics, pyrealsense2, paho.mqtt; print('deps ok')"
python3 -c "import paho.mqtt; print('paho', paho.mqtt.__version__)"
```

- 老板子 torch/CUDA/YOLO 已具备，**不要**重新 `pip install torch/torchvision/ultralytics`。
- 如果 `paho-mqtt` 缺失：`pip3 install paho-mqtt`。
- RealSense 必须能识别：

```bash
python3 -c "import pyrealsense2 as rs; print('devices:', len(rs.context().query_devices()))"
```

---

## 4. Ollama 与模型

按《老板子Ollama安装与模型部署说明》执行，核心验收：

```bash
ollama --version
systemctl is-active ollama
ollama list
ollama run phi3:mini "只回复OK"
ollama run qwen2.5vl:3b "只回复OK"
ollama ps
```

`ollama ps` 中 `PROCESSOR` 应为 `100% GPU`（或 GPU/CPU 混合）。
如果已经是装好的状态，跳过安装，只做验收。

---

## 5. MQTT broker

两种选择：

### A. 板上本地 Mosquitto（推荐现场）

```bash
sudo apt install -y mosquitto mosquitto-clients
sudo systemctl enable --now mosquitto
```

### B. 现有服务器

如果 `192.168.38.213:1883` 从板子可达，直接沿用，不装本地 broker。

后面用**演示主题**先测：

```text
node/arm/99/board/down
node/arm/99/board/up
```

---

## 6. 机械臂与相机

```bash
ip a
ping -c 3 192.168.5.1
```

- 板子以太网配 `192.168.5.x/24`，不要和 Wi-Fi 网段冲突。
- Dobot 端口：29999（dashboard）、30004（反馈）。
- RealSense 插 USB3，确认 `devices: 1`。
- 物理急停在手边，工作区清空。

---

## 7. 启动顺序

### 终端 1：执行端

```bash
cd ~/Desktop/Dobot_Vision_Grasp/yolov5_vision_grasp

export DOBOT_MQTT_HOST=127.0.0.1
export DOBOT_MQTT_USERNAME=
export DOBOT_MQTT_PASSWORD=
export DOBOT_MQTT_TOPIC_DOWN=node/arm/99/board/down
export DOBOT_MQTT_TOPIC_UP=node/arm/99/board/up

python3 mqtt_yolo_grasp.py
```

等它打印：

```text
🎮 系统就绪，等待MQTT指令...
```

### 终端 2：agent（先 dry-run）

```bash
cd ~/Desktop/Dobot_Vision_Grasp/llm_agent

python3 llm_grasp_agent.py --selftest

python3 llm_grasp_agent.py \
  --broker 127.0.0.1 \
  --topic-down node/arm/99/board/down \
  --topic-up node/arm/99/board/up \
  --once "抓蓝色方块"
```

dry-run 只显示解析计划，不发 MQTT、不动机械臂。

### 第一次真机：用 `--confirm`

```bash
python3 llm_grasp_agent.py \
  --broker 127.0.0.1 \
  --topic-down node/arm/99/board/down \
  --topic-up node/arm/99/board/up \
  --execute --confirm --once "抓蓝色方块"
```

确认机械臂动作、回执 `0`、VLM 快照生成后，再测“放下”。

### 正常使用（默认直接执行）

```bash
python3 llm_grasp_agent.py \
  --broker 127.0.0.1 \
  --topic-down node/arm/99/board/down \
  --topic-up node/arm/99/board/up \
  --execute
```

---

## 8. 语音（可选）

板子上文字 agent 已经够用；要语音再加：

```bash
pip3 install faster-whisper sounddevice
```

模型缓存可从 Windows 拷：

```text
C:\Users\DragonAfterAll\.cache\huggingface\hub\models--Systran--faster-whisper-small
```

到板子：

```text
~/.cache/huggingface/hub/models--Systran--faster-whisper-small
```

或用 `HF_ENDPOINT=https://hf-mirror.com` 首次下载。

测试：

```bash
python3 llm_grasp_agent.py --voice-test
```

注意：Jetson CPU 上 small 模型识别会慢一些；不行就用 `--voice-model base`，或先只用文字。

---

## 9. 验收清单

| # | 检查项 | 预期 |
|---|---|---|
| 1 | `python3 -c "import torch; print(torch.cuda.is_available())"` | True |
| 2 | `python3 -c "import pyrealsense2..."` | devices: 1 |
| 3 | `ollama ps` | phi3/qwen 100% GPU |
| 4 | `llm_grasp_agent.py --selftest` | 全部通过 |
| 5 | dry-run “抓蓝色方块” | `grasp blue`，不动机械臂 |
| 6 | `--execute --confirm` 抓蓝色 | 回执 0，VLM 有报告 |
| 7 | “放下” | 回执 0，回 DEFAULT |
| 8 | 空手“放下” | 回执 9，agent 如实报告 |
| 9 | “急停” | 回执 0；物理急停仍优先 |
| 10 | 快照路径 | `~/dobot_llm_agent/frames/latest.jpg` 存在 |

---

## 10. 回滚

```bash
cd ~/Desktop/Dobot_Vision_Grasp/yolov5_vision_grasp
cp mqtt_yolo_grasp.py.bak_YYYYmmdd_HHMM mqtt_yolo_grasp.py
```

`llm_agent/` 可整体删除或改名，不影响原来的 YOLO/MQTT 抓取程序。
Ollama 可按说明文档第 6 节卸载，模型保留/删除自选。

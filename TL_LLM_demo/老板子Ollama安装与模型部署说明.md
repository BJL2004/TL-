# 老板子 Ollama 安装与模型部署说明

> 适用对象：**已具备 torch / CUDA / YOLO / RealSense 环境的老板子**（Jetson Orin NX 16GB，JetPack 6.x，L4T R36）
>
> 本文只覆盖 **Ollama 安装 + phi3:mini / qwen2.5vl:3b 部署 + 与 Dobot LLM/VLM 程序对接**。
> 不重复安装 CUDA、PyTorch、torchvision、OpenCV、RealSense、ultralytics。
>
> 验证日期：2026-10-21
> 验证版本：Ollama `0.40.6`、`phi3:mini`（2.2GB）、`qwen2.5vl:3b`（3.2GB）

---

## 0. 先说明三个容易误解的点

1. **“只缺 Ollama”对“跑模型”成立，对“跑大模型抓取”不完全成立。**
   - 只跑 YOLO / MQTT 抓取：装好 Ollama 就够了。
   - 要跑“中文 → LLM → MQTT → 抓取 + VLM 参谋”，老板子上还必须同步：
     - `Dobot_Vision_Grasp/llm_agent/`（`llm_grasp_agent.py`、`mock_executor.py`、README）
     - 更新版 `Dobot_Vision_Grasp/yolov5_vision_grasp/mqtt_yolo_grasp.py`
       （VLM 快照钩子、MQTT 急停、`DOBOT_MQTT_*` 环境变量覆盖）
   - 否则 Ollama 能跑，但 agent 不存在，或 VLM 读不到执行端画面。

2. **CUDA 不需要再装，也不要单独装。**
   - CUDA 12.x 由 JetPack 自带；`torch.cuda.is_available()` 正常即可。
   - Ollama 的 GPU 能力来自 **`ollama-linux-arm64-jetpack6.tar.zst`** 叠加包。
   - 只装通用 `ollama-linux-arm64.tar.zst` 会退回 CPU。

3. **`pip install ollama` 只是 Python 客户端库，不是服务端。**
   - 服务端必须是官方二进制（本文的 tar 包）。
   - Python 程序通过 `http://127.0.0.1:11434` 访问服务端，不需要 `pip install ollama`。

---

## 1. 安装前检查（只读，不改环境）

```bash
cat /etc/nv_tegra_release
# 应包含 R36；R36 = JetPack 6

free -h
df -h /
# 建议剩余磁盘 ≥ 10GB（Ollama 解压 + 两个模型约 7-9GB）

python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"
# 期望 True

python3 -c "import cv2, ultralytics, pyrealsense2; print('yolo/realsense ok')"
# 老板子应已具备；若报错先不要继续，先修原有环境

which ollama
systemctl status ollama --no-pager
# 如果已经装过 Ollama，跳到第 4 节；版本过旧再考虑第 2 节重装
```

确认网络能访问 SourceForge（国内通常可达）：

```bash
curl -sI https://sourceforge.net/projects/ollama.mirror/files/v0.40.6/ | head -3
```

> 如果老板子没有 `curl` / `zstd`：
> ```bash
> sudo apt update
> sudo apt install -y curl zstd ca-certificates aria2
> ```

---

## 2. 安装 Ollama

### 方式 A：官方脚本（网络好时最简单）

```bash
sudo apt install -y curl zstd pciutils ca-certificates
curl -fsSL https://ollama.com/install.sh | sh
```

官方脚本会自动识别 `/etc/nv_tegra_release` 里的 `R36`，额外下载并解压
`ollama-linux-arm64-jetpack6.tar.zst`。

> 国内直连 GitHub 往往很慢；如果卡在 `Downloading ollama-linux-arm64.tar.zst`，
> 按 `Ctrl+C`，改用方式 B。

### 方式 B：SourceForge 镜像 + 手动安装（推荐）

镜像地址（v0.40.6，已实测可访问）：

- https://sourceforge.net/projects/ollama.mirror/files/v0.40.6/

需要两个文件：

| 文件 | 大小 |
|---|---|
| `ollama-linux-arm64.tar.zst` | 约 1.56GB |
| `ollama-linux-arm64-jetpack6.tar.zst` | 约 271MB |

#### B1. 下载（板子直下，多线程）

```bash
sudo apt install -y aria2
cd /tmp

aria2c -c -x 16 -s 16 -k 1M -o ollama-linux-arm64.tar.zst \
"https://sourceforge.net/projects/ollama.mirror/files/v0.40.6/ollama-linux-arm64.tar.zst/download"

aria2c -c -x 16 -s 16 -k 1M -o ollama-linux-arm64-jetpack6.tar.zst \
"https://sourceforge.net/projects/ollama.mirror/files/v0.40.6/ollama-linux-arm64-jetpack6.tar.zst/download"
```

也可以从 Windows 浏览器下载后 scp 到板子：

```powershell
scp .\ollama-linux-arm64.tar.zst .\ollama-linux-arm64-jetpack6.tar.zst nvidia@<板子IP>:/tmp/
```

#### B2. 校验（v0.40.6）

```bash
cd /tmp
sha256sum ollama-linux-arm64.tar.zst ollama-linux-arm64-jetpack6.tar.zst
```

应为：

```text
0a279d9428ad17b06fdb7dddf3b059b2c148433585fa3b30676a102fc459bb9f  ollama-linux-arm64.tar.zst
a01193f70ead22ce0adae54a7164df9fef587869ab58616eaa657eb9436c1c4d  ollama-linux-arm64-jetpack6.tar.zst
```

#### B3. 解压（顺序不能反：基础包 → jetpack6）

```bash
sudo rm -rf /usr/local/lib/ollama
sudo mkdir -p /usr/local/lib/ollama

sudo tar --zstd -xf /tmp/ollama-linux-arm64.tar.zst -C /usr/local
sudo tar --zstd -xf /tmp/ollama-linux-arm64-jetpack6.tar.zst -C /usr/local

ls -lh /usr/local/bin/ollama
ls /usr/local/lib/ollama | head -30
# 应能看到 cuda_jetpack6、cuda_v12、cuda_v13 等目录/库

/usr/local/bin/ollama --version
# 0.40.6（服务未启动时可能附带 “could not connect” 警告，正常）
```

### 2.1 创建 systemd 服务

```bash
sudo useradd -r -s /bin/false -U -m -d /usr/share/ollama ollama 2>/dev/null || true
sudo usermod -a -G render ollama 2>/dev/null || true
sudo usermod -a -G video ollama 2>/dev/null || true

sudo tee /etc/systemd/system/ollama.service >/dev/null <<'EOF'
[Unit]
Description=Ollama Service
After=network-online.target

[Service]
ExecStart=/usr/local/bin/ollama serve
User=ollama
Group=ollama
Restart=always
RestartSec=3
Environment="PATH=/usr/local/bin:/usr/bin:/bin"

[Install]
WantedBy=default.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now ollama
systemctl status ollama --no-pager
```

### 2.2 验证服务

```bash
curl -s http://127.0.0.1:11434/api/version
# {"version":"0.40.6"}

ollama --version
```

如果服务起不来：

```bash
journalctl -u ollama -n 100 --no-pager
sudo ss -ltnp | grep 11434
# 端口被占用：停掉旧进程，或改 OLLAMA_HOST
```

---

## 3. 部署模型

### 3.1 拉取

```bash
ollama pull phi3:mini
ollama pull qwen2.5vl:3b
ollama list
```

预期：

```text
NAME            ID              SIZE      MODIFIED
qwen2.5vl:3b    ...             3.2 GB    ...
phi3:mini       ...             2.2 GB    ...
```

> 拉取慢或中断后，重新执行同一条 `ollama pull` 会续传。
> 建议放 tmux 里：
> ```bash
> sudo apt install -y tmux
> tmux new -s ollama
> ollama pull phi3:mini
> ollama pull qwen2.5vl:3b
> # Ctrl+B 再按 D 退出；tmux attach -t ollama 回来
> ```

### 3.2 验证 LLM 与 GPU

```bash
ollama run phi3:mini "只回复OK"
ollama ps
```

`PROCESSOR` 一栏应为：

```text
100% GPU
```

如果显示 CPU：

```bash
journalctl -u ollama -n 100 --no-pager
ls /usr/local/lib/ollama | grep cuda
# 确认 cuda_jetpack6 存在；不存在说明 jetpack6 包没解压或顺序反了
sudo systemctl restart ollama
```

### 3.3 验证 VLM

```bash
ollama run qwen2.5vl:3b "只回复OK"
ollama ps
```

VLM 的图片能力由程序通过 `/api/chat` 的 `images` 字段调用，部署时先确认模型能加载、GPU 正常即可。

可选：用一张现场画面直接测 VLM（需要已有快照 `~/dobot_llm_agent/frames/latest.jpg`）：

```bash
python3 - <<'PY'
import base64, json, os, urllib.request

path = os.path.expanduser("~/dobot_llm_agent/frames/latest.jpg")
if not os.path.exists(path):
    print("没有快照，先运行一次执行端抓取或在工程里生成演示画面")
    raise SystemExit

image = base64.b64encode(open(path, "rb").read()).decode()
payload = {
    "model": "qwen2.5vl:3b",
    "messages": [{
        "role": "user",
        "content": "用一句话描述画面里有哪些颜色的方块，分别在什么位置。",
        "images": [image],
    }],
    "stream": False,
    "options": {"temperature": 0.1},
}
request = urllib.request.Request(
    "http://127.0.0.1:11434/api/chat",
    data=json.dumps(payload).encode("utf-8"),
    headers={"Content-Type": "application/json"},
)
with urllib.request.urlopen(request, timeout=180) as response:
    print(json.loads(response.read().decode("utf-8"))["message"]["content"])
PY
```

### 3.4 16GB 内存配置（可选）

两个模型常驻会占约 7-8GB；老板子 16GB 通常够用。建议限制并发并允许两个模型驻留：

```bash
sudo mkdir -p /etc/systemd/system/ollama.service.d
sudo tee /etc/systemd/system/ollama.service.d/override.conf >/dev/null <<'EOF'
[Service]
Environment="OLLAMA_MAX_LOADED_MODELS=2"
Environment="OLLAMA_NUM_PARALLEL=1"
EOF

sudo systemctl daemon-reload
sudo systemctl restart ollama
```

> 如果老板子是 8GB 版本，把 `OLLAMA_MAX_LOADED_MODELS` 改为 `1`，
> 并在 agent 侧用 `--no-vlm` 或只在需要时加载 VLM。

模型实际存放路径：

```bash
sudo du -sh /usr/share/ollama/.ollama/models
```

---

## 4. 与 Dobot LLM/VLM 程序对接

### 4.1 前提文件（老板子必须是最新版）

```bash
ls ~/Desktop/Dobot_Vision_Grasp/llm_agent/llm_grasp_agent.py
ls ~/Desktop/Dobot_Vision_Grasp/yolov5_vision_grasp/mqtt_yolo_grasp.py
```

- `llm_agent/`：意图层（LLM + 安全层 + VLM 参谋）
- `mqtt_yolo_grasp.py`：必须包含 VLM 快照钩子 `_save_vlm_frame`、`estop` 处理、`DOBOT_MQTT_*` 环境变量

如果老板子上还是旧文件，从 Windows 覆盖同步：

```powershell
# 整个 agent 目录（包含 mock_executor.py、README、留档）
scp -r "D:\work\engineering_project\Dobot_Nova2\Dobot_Vision_Grasp\llm_agent" nvidia@<板子IP>:~/Desktop/Dobot_Vision_Grasp/

# 更新版执行端（VLM 快照钩子 / MQTT 急停 / 环境变量覆盖）
scp "D:\work\engineering_project\Dobot_Nova2\Dobot_Vision_Grasp\yolov5_vision_grasp\mqtt_yolo_grasp.py" nvidia@<板子IP>:~/Desktop/Dobot_Vision_Grasp/yolov5_vision_grasp/
```

如果整个工程还没拷到老板子：

```powershell
scp -r "D:\work\engineering_project\Dobot_Nova2\Dobot_Vision_Grasp" nvidia@<板子IP>:~/Desktop/
```

### 4.2 默认连接方式

agent 默认：

```text
Ollama: http://127.0.0.1:11434
LLM:    phi3:mini
VLM:    qwen2.5vl:3b
```

老板子上直接运行即可，不需要改参数：

```bash
cd ~/Desktop/Dobot_Vision_Grasp/llm_agent

python3 llm_grasp_agent.py --selftest
python3 llm_grasp_agent.py --once "抓红色方块"       # dry-run，不发送 MQTT
python3 llm_grasp_agent.py --once "把蓝色方块放到放置点"
```

实跑（执行端与 broker 已就绪）：

```bash
python3 llm_grasp_agent.py --execute
```

### 4.3 如果 agent 不在老板子上（可选）

例如 agent 跑在 Windows，Ollama 仍在老板子：

```bash
# 老板子：让 Ollama 监听局域网
sudo systemctl edit ollama
# 加入：
# Environment="OLLAMA_HOST=0.0.0.0:11434"
sudo systemctl restart ollama
sudo ufw allow 11434/tcp   # 如果启用了 ufw
```

agent 侧：

```bash
python3 llm_grasp_agent.py --ollama-url http://<老板子IP>:11434 ...
```

> 不要把 11434 暴露到公网；Ollama 没有认证。

### 4.4 VLM 快照链路

- 执行端每次检测到目标时写：
  - `~/dobot_llm_agent/frames/latest.jpg`
  - `~/dobot_llm_agent/frames/latest.json`（时间戳）
- agent 任务结束后自动复核；交互中输入 `看一眼` 手动复核。
- 快照超过 `--vlm-max-age`（默认 180 秒）会被视为过期。
- 如果 agent 提示“没有新鲜画面快照”：
  1. 执行端是否更新到带 `_save_vlm_frame` 的版本；
  2. 执行端是否真的检测到过目标（只有检测到才写图）；
  3. agent 的 `--frame-dir` 是否与执行端一致（默认都是 `~/dobot_llm_agent/frames`）。

### 4.5 关闭 VLM

只想要 LLM 翻译、不需要视觉参谋：

```bash
python3 llm_grasp_agent.py --no-vlm ...
```

不装 `qwen2.5vl:3b` 也不影响抓取主流程。

---

## 5. 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| `curl: command not found` | `sudo apt install -y curl zstd ca-certificates` |
| 官方脚本下载卡住 | 国内 GitHub 慢；改用 SourceForge 方式 B |
| `sha256sum` 不一致 | 下载不完整；删文件重下，不要解压 |
| `tar --zstd` 报错 | `sudo apt install -y zstd` |
| `ollama: command not found` | 用 `/usr/local/bin/ollama`，或把 `/usr/local/bin` 加入 PATH |
| 服务起不来 | `journalctl -u ollama -n 100`；检查 11434 是否被占用 |
| `ollama ps` 显示 CPU | 确认 `cuda_jetpack6` 已解压；`ls /usr/local/lib/ollama | grep cuda`；重启服务；确认 ollama 用户在 `render`/`video` 组 |
| VLM 能跑但不看图 | 执行端没有写快照，或 agent `--frame-dir` 不一致 |
| 模型拉取很慢 | 重试可续传；放 tmux；避免同时拉两个 |
| 内存不足 / OOM | 16GB 一般够；8GB 用 `OLLAMA_MAX_LOADED_MODELS=1`，或 `--no-vlm` |
| 想手动调试 | `sudo systemctl stop ollama` 后运行 `ollama serve`，看终端日志 |
| JetPack 7.x | 不建议；Ollama 的 Jetson GPU 支持以 JP6 为准 |

---

## 6. 升级与回滚

### 升级 Ollama

模型不受影响，只替换二进制和 `lib/ollama`：

```bash
sudo systemctl stop ollama
# 下载新版 base + jetpack6（顺序同上）
sudo rm -rf /usr/local/lib/ollama
sudo mkdir -p /usr/local/lib/ollama
sudo tar --zstd -xf /tmp/ollama-linux-arm64.tar.zst -C /usr/local
sudo tar --zstd -xf /tmp/ollama-linux-arm64-jetpack6.tar.zst -C /usr/local
sudo systemctl start ollama
ollama --version
```

### 卸载 Ollama（保留模型可选）

```bash
sudo systemctl disable --now ollama
sudo rm -f /etc/systemd/system/ollama.service
sudo systemctl daemon-reload
sudo rm -rf /usr/local/bin/ollama /usr/local/lib/ollama

# 彻底删除模型（可选，约 5-6GB）
sudo rm -rf /usr/share/ollama/.ollama/models
```

---

## 7. 验收清单

| # | 检查项 | 预期 |
|---|---|---|
| 1 | `ollama --version` | `0.40.6` |
| 2 | `systemctl is-active ollama` | `active` |
| 3 | `curl -s http://127.0.0.1:11434/api/version` | 返回 version |
| 4 | `ollama list` | 有 `phi3:mini`、`qwen2.5vl:3b` |
| 5 | `ollama run phi3:mini "只回复OK"` | 输出 OK |
| 6 | `ollama ps` | `100% GPU` |
| 7 | `python3 llm_grasp_agent.py --selftest` | 全部通过 |
| 8 | `python3 llm_grasp_agent.py --once "抓红色方块"` | dry-run 解析为 `grasp red` |
| 9 | 实跑一次抓取后 agent 自动出视觉报告 | 有 VLM 画面描述 |
| 10 | 输入“看一眼” | 手动出 VLM 报告 |

---

## 附录：最短命令版（SourceForge 镜像）

```bash
sudo apt update
sudo apt install -y curl zstd ca-certificates aria2

cd /tmp
aria2c -c -x 16 -s 16 -k 1M -o ollama-linux-arm64.tar.zst \
"https://sourceforge.net/projects/ollama.mirror/files/v0.40.6/ollama-linux-arm64.tar.zst/download"
aria2c -c -x 16 -s 16 -k 1M -o ollama-linux-arm64-jetpack6.tar.zst \
"https://sourceforge.net/projects/ollama.mirror/files/v0.40.6/ollama-linux-arm64-jetpack6.tar.zst/download"

sudo rm -rf /usr/local/lib/ollama
sudo mkdir -p /usr/local/lib/ollama
sudo tar --zstd -xf /tmp/ollama-linux-arm64.tar.zst -C /usr/local
sudo tar --zstd -xf /tmp/ollama-linux-arm64-jetpack6.tar.zst -C /usr/local

sudo useradd -r -s /bin/false -U -m -d /usr/share/ollama ollama 2>/dev/null || true
sudo usermod -a -G render ollama 2>/dev/null || true
sudo usermod -a -G video ollama 2>/dev/null || true

sudo tee /etc/systemd/system/ollama.service >/dev/null <<'EOF'
[Unit]
Description=Ollama Service
After=network-online.target

[Service]
ExecStart=/usr/local/bin/ollama serve
User=ollama
Group=ollama
Restart=always
RestartSec=3
Environment="PATH=/usr/local/bin:/usr/bin:/bin"

[Install]
WantedBy=default.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now ollama

ollama pull phi3:mini
ollama pull qwen2.5vl:3b
ollama list
ollama run phi3:mini "只回复OK"
ollama ps
```

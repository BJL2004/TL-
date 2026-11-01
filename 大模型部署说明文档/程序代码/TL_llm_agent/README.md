# TL_llm_agent — TL 种子抓放 · 大模型(MQTT)控制代理

**一句话**：中文指令 → 本地大模型 phi3:mini（Ollama）→ 任务指令 JSON → 安全层校验
→ MQTT 下发 `node/arm/01/down` → TL 现成程序执行抓放 → 订阅回执把结果翻译成人话。

- **对 TL 工程只加了两处"存一帧"钩子**（VLM 视觉辅助用，不改变任何算法逻辑）：
  `mock_pick_place.py` 新增 `_save_vlm_frame()` + 两个调用点；已为
  `TL_lowerobs_test` 和 `TL/TL_Handback` 两个副本打好补丁（跑哪个就用哪份）
- **命令仍走现有 MQTT 主题**：与 `run_seed_mqtt.sh` 同时存在（broker 相同、client-id 不同）
- **大模型只输出任务级 JSON**（`cmd` + `targetId`），绝不输出坐标/轨迹/速度
- 支持两种现场说法：
  - 编号式：`把种子放到 A3-17`
  - 序号式：`把种子放到第三排第十七盆`（第1~4排=A1~A4，第5~8排=B1~B4，每排 24 盆）

## 文件

| 文件 | 说明 |
|---|---|
| `llm_tl_agent.py` | 主程序（解析 + 安全层 + MQTT） |
| `mock_tl.py` | 模拟 TL 执行端（99 演示主题） |
| `run_llm_agent.sh` | 启动脚本（参数透传） |
| `SKILL.md` | 操作 SKILL（启动/演示/真机/排错，不含语音） |
| `验证报告_2026-10-22.md` | 本次审查发现与修复记录 |

## 板上部署（Jetson Orin NX 16GB / JetPack 6）

```bash
# 0) 把整个 TL_llm_agent 文件夹拷到板上: ~/Desktop/TL_llm_agent
# 1) 依赖（paho 一般已装；没有则装）
python3 -c "import paho.mqtt.client" || sudo apt install -y python3-paho-mqtt

# 2) Ollama（JetPack 6 原生支持 Jetson CUDA）
curl -fsSL https://ollama.com/install.sh | sh
ollama pull phi3:mini        # ~2.3GB，建议 NVMe 盘
ollama pull qwen2.5vl:3b     # ~3.2GB，VLM 视觉辅助用（不需要可跳过）
ollama ps                    # 推理时可见 100% GPU

# 3) 自检（不需要 Ollama/MQTT/机械臂——纯解析器测试）
python3 llm_tl_agent.py --selftest

# 4) dry-run 试解析（不发送）
bash run_llm_agent.sh
#   > 把种子放到 A3-17
#   > 放到第三排第十七盆

# 5) 实跑（先确保 TL 驱动 + run_seed_mqtt.sh 已启动并在线）
#    真实主题必须显式加 --allow-real
bash run_llm_agent.sh --execute --allow-real --timeout 300
```

## 交互示例

```
> 把种子放到第三排第十七盆
[LLM] {"cmd":"seed","targetId":"A3-17"}
📋 执行计划: seed → 放种子（取种→识码→放置→回位）
   目标盆 A3-17（盆号 65）
📤 已下发 node/arm/01/down: {"cmd":"seed","targetId":"A3-17"}
📩 回执: cmd=seed retCode=1（已接受）
📩 回执: cmd=seed retCode=0（取种完成）
📩 回执: cmd=seed_place retCode=1（开始放置）
✅ 结果: retCode=0 成功 → A3-17
```

默认**直接执行，不再询问 y/N**（与 Dobot 版一致）；需要逐条确认时加 `--confirm`。
`急停`（或"停止/停下"）→ 立即发送 `{"cmd":"estop"}`，**无需确认**；物理急停始终优先。

## VLM 视觉辅助（qwen2.5vl:3b）

**定位**：只做画面复核报告，**不参与任何动作决策**——抓放动作仍是 TL 底层算法的活。

**数据从哪来**：TL 的 RealSense 被其进程独占，而 TL 程序在观测位检测花盆码时会取帧
（`mock_pick_place.py` 两处）。补丁在这两处各加了一次 `_save_vlm_frame(image)`，
把当前帧写到 `~/tl_llm_agent/frames/latest.jpg`（+ `latest.json` 时间戳）。**仅写文件，不改变检测/放置逻辑。**

**什么时候触发**：
1. 每次任务回执返回后自动复核（retCode=3 识码失败时重点提示码的问题）
2. 交互中输入 `看一眼` / `看看` / `check` 手动复核最新画面

**输出示例**：
```
🔍 视觉辅助分析中（仅供参考，不参与动作决策）…
🔍 视觉辅助: 画面中央能看到花盆，二维码清晰、未被遮挡，没有明显异常。
```

**注意**：
- 快照超过 `--vlm-max-age`（默认 180 秒）视为过期，不会用旧画面误导你
- VLM 判断力有限（3B），结论仅供参考；关键判断仍以 TL 的 aruco 检测与人工观察为准
- 板上未 `ollama pull qwen2.5vl:3b` 时会提示"视觉辅助不可用"，不影响主流程
- 不需要视觉辅助：`--no-vlm` 关闭（零开销）

## Windows 演示（不接机械臂）

**演示 A：纯大模型解析 + 视觉辅助（零配置，现在就能演）**

```bash
cd TL_llm_agent
python llm_tl_agent.py          # dry-run，不会发送任何 MQTT
#   > 把种子放到 A3-17
#   > 放到第三排第十七盆
#   > 看一眼                    ← VLM 复核 ~/tl_llm_agent/frames/latest.jpg
```

（已准备一张 ArUco 码演示帧在 `~/tl_llm_agent/frames/latest.jpg`，可直接演示"看一眼"）

**演示 B：完整链路（LLM → MQTT → 模拟执行 → 回执 → 视觉辅助）**

```bash
# 终端 1：模拟 TL 执行端（不接机械臂）
python mock_tl.py                          # 本地 broker 127.0.0.1:1883
#   或走云 broker 的【演示主题】：
python mock_tl.py --host 116.205.245.172 --username shapandevice --password "qwer1234!!"

# 终端 2：LLM 控制端（实跑模式，但只发演示主题）
python llm_tl_agent.py --execute \
    --topic-down node/arm/99/down --topic-up node/arm/99/up
#   本地 broker 时再加：--broker 127.0.0.1 --username "" --password ""
```

⚠️ **演示必须用 `99` 演示主题**；真实主题 `node/arm/01/*` 会让在线的真实机器人执行动作！

## 安全层（为什么它不会放错盆）

1. **白名单**：只允许 `seed / seed_place / trim_pick / trim_place / estop`
2. **targetId 规则校验**：`A1-01 ~ B4-24` 正则 + 盆号 1~24，非法直接拒绝
3. **目标必须由规则层解析出来**：编号式/序号式目标由程序确定性解析；
   LLM 猜的 targetId **不再采用**（防“放到那边”被模型猜成 A1-01）
4. **动作以规则层为准**：句子里没有“叉取/修剪/放回”等动作词时，不会让 LLM 凭空选择
   `trim_pick/trim_place`；只有目标没有动作时默认 `seed`
5. **规则 × LLM 交叉核对**：两边都解析出目标且不一致时拒绝执行并要求重说
6. **真实主题保护**：`--execute` 使用 `node/arm/01/*` 时必须显式加 `--allow-real`；
   测试/演示默认拒绝连接真实主题
7. **单任务互斥**：发出任务后等待回执（默认 120s），期间不接受新任务（`estop` 除外）
8. **默认直接执行**：`--execute` 下发即执行，不询问 y/N（与 Dobot 版一致）；
   需要逐条确认时加 `--confirm`；`--yes` 保留仅为兼容旧命令，已无效；
   **急停始终直发，不等待确认**
9. **dry-run 默认**：不加 `--execute` 绝不发送

## 回执语义（沿用 TL 协议）

| cmd | retCode=0 | 1 | 2 | 3 |
|---|---|---|---|---|
| seed / seed_place | **最终成功**（seed_place:0） | 已开始（中间态，继续等） | 设备忙 | 取种/识码/放置失败 |
| trim_pick / trim_place | 成功 | 已开始动作（继续等） | 设备忙 | 识码/放置失败 |

种子程序是**两段式回执**：`seed:1`（已接受）→ `seed:0`（取种完成，**不是最终结果**）
→ `seed_place:1`（开始放置）→ `seed_place:0`（最终成功）/ `2` / `3`。
agent 会等到 `seed_place` 的最终结果再报成功；旧执行端只回一条 `seed:0` 时加 `--legacy-seed`。

## 常用参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--execute` | 关 | 实跑发送（默认 dry-run） |
| `--allow-real` | 关 | 允许向真实主题 `node/arm/01/*` 下发；不加则拒绝连接 |
| `--confirm` | 关 | 执行前逐条询问 y/N（默认直接执行） |
| `--yes` | 关 | 兼容旧参数：默认已不询问，已无效 |
| `--once "..."` | — | 单条测试 |
| `--no-llm` | 关 | 禁用大模型，只用规则解析（测试用） |
| `--model` | `phi3:mini` | Ollama 模型 |
| `--ollama-url` | `http://127.0.0.1:11434` | 若 Ollama 在别的机器，改成该机 IP |
| `--no-vlm` | 关（默认启用） | 禁用 VLM 视觉辅助 |
| `--vlm-model` | `qwen2.5vl:3b` | 视觉模型名 |
| `--frame-dir` | `~/tl_llm_agent/frames` | TL 快照目录（与补丁写入路径一致） |
| `--broker/--port/--username/--password` | TL 现有配置 | 默认与 `run_seed_mqtt.sh` 一致 |
| `--timeout` | 120 | 等回执超时（秒）；seed 完整周期建议 300 |
| `--legacy-seed` | 关 | 兼容旧执行端：收到 seed:0 即视为整个周期成功 |

## 注意

- TL 端**同一时间只能跑一个任务**（并发会回 retCode=2 设备忙）
- 本程序与 `run_seed_mqtt.sh` 用的 broker 相同（`116.205.245.172:1883`），需网络可达
- 版本记录：2026-10-20 首版（文字输入，无语音）；2026-10-22 安全层加固（目标/动作确定性、真实主题保护、急停免确认），VLM 已接入；2026-10-22 种子两段式回执（seed→seed_place）修正，新增 `--legacy-seed`；2026-10-22 默认去掉 y/N（新增 `--confirm` 可选），`--yes` 兼容但无效。

---
name: tl-llm-agent-operation
description: 操作 TL 种子抓放大模型控制端（phi3:mini + MQTT + 可选 VLM）。用于启动、dry-run、演示主题测试、真机执行、故障排查；不含语音。
---

# SKILL：TL 机械臂大模型操作

## 何时使用

当需要通过中文指令控制 TL 机械臂执行
`seed / seed_place / trim_pick / trim_place / estop`，
或需要检查 `TL_llm_agent` 是否正常、怎么安全演示/实跑时使用。

## 前提

- TL 驱动和抓放程序已启动（`run_seed_mqtt.sh` 或对应程序在线）。
- Ollama 已运行：`ollama ps` 能看到 `phi3:mini`；VLM 可选 `qwen2.5vl:3b`。
- 板子能连 broker `116.205.245.172:1883`（或使用本地 broker）。
- 物理急停在手边。
- **本 SKILL 不含语音**；只接受文字输入。

## 核心安全规则（必须遵守）

1. **演示/测试只用 99 主题**：
   `node/arm/99/down` / `node/arm/99/up`。
2. **真实主题必须显式允许**：
   `--execute` + `--allow-real` 才会连接 `node/arm/01/*`；否则程序直接拒绝。
3. **目标和动作以规则层为准**：
   - 目标必须能解析成 `A1-01 ~ B4-24` / `第三排第十七盆`；
   - LLM 猜的目标、凭空猜的 `trim_pick/trim_place` 不会执行；
   - 句子里没有动作词时，只有目标 → 默认 `seed`。
4. **急停不需要 y/N**：直接发送；物理急停永远优先。
5. **默认直接执行**（与 Dobot 版一致）；需要逐条确认加 `--confirm`；`--yes` 已无效。
6. 同一时间只跑一个任务；TL 并发会回 `retCode=2` 设备忙。

## 标准操作流程

### 1. 自检（不需要机械臂/Ollama/MQTT）

```bash
cd ~/Desktop/TL_llm_agent
python3 llm_tl_agent.py --selftest
```

预期：规则、目标校验、安全层全部通过。

### 2. dry-run（不动机械臂）

```bash
python3 llm_tl_agent.py
```

输入：

```text
把种子放到 A3-17
把种子放到第三排第十七盆
叉取 A4-01 的花篮
把花篮放回 A4-01
急停
退出
```

只打印执行计划，不发送 MQTT。

### 3. 演示链路（mock + 99 主题）

```bash
# 终端 1
cd ~/Desktop/TL_llm_agent
python3 mock_tl.py --delay 0.5

# 终端 2
cd ~/Desktop/TL_llm_agent
python3 llm_tl_agent.py --execute \
  --broker 127.0.0.1 --username= --password= \
  --topic-down node/arm/99/down --topic-up node/arm/99/up
```

预期：

- `seed` → retCode 0
- `trim_pick` → 先 1（已开始），再 0（最终）
- `trim_place` → 先 1，再 0
- `estop` → 直接发送

### 4. 真机

```bash
cd ~/Desktop/TL_llm_agent
bash run_llm_agent.sh --execute --allow-real --timeout 300
```

- 默认使用真实 broker/主题（与 `run_seed_mqtt.sh` 一致）。
- 首次先发一条 `seed`，确认机械臂动作、回执 0、VLM 报告，再做其他盆。
- 首次实跑建议加 `--confirm` 逐条确认；确认无误后再直接执行。

## 支持的说话方式

| 说法 | cmd | targetId |
|---|---|---|
| 把种子放到 A3-17 | seed | A3-17 |
| 放到第三排第十七盆 | seed | A3-17 |
| 把种子放到 B2 区第六盆 | seed | B2-06 |
| 叉取 A4-01 的花篮 | trim_pick | A4-01 |
| 把花篮放到 A4-01 / 放回 A4-01 | trim_place | A4-01 |
| 急停 / 停止 / 停下 | estop | — |

排号映射：第 1~4 排 = A1~A4，第 5~8 排 = B1~B4；每排 24 盆。

不支持的说话方式（会被拒绝）：

- 只说“放到那边”“那个盆”（没有明确编号）
- “叉取花篮放到 A4-01”（同时含叉取和放置，动作歧义）
- `A3-25 / C1-01 / A5-01`（越界）

## 回执语义

| cmd | 0 | 1 | 2 | 3 |
|---|---|---|---|---|
| seed / seed_place | 最终成功（seed_place:0） | 已开始，继续等 | 设备忙 | 取种/识码/放置失败 |
| trim_pick / trim_place | 成功 | 已开始，继续等 | 设备忙 | 识码/放置失败 |

种子是两段式：`seed:1 → seed:0（取种完成，非终态）→ seed_place:1 → seed_place:0`。
`retCode=1` 对 seed/trim 都不是最终结果，agent 会继续等；不要手工中断。
旧执行端只回 `seed:0` 时加 `--legacy-seed`。

## 视觉辅助（可选）

- TL 程序在观测位检测时写 `~/tl_llm_agent/frames/latest.jpg` + `latest.json`。
- 任务结束后 agent 自动调用 `qwen2.5vl:3b` 复核；输入“看一眼”手动复核。
- 只报告，不参与动作；没有快照会提示。
- 关闭：`--no-vlm`。

## 故障排查

| 现象 | 处理 |
|---|---|
| 拒绝连接 / 真实主题保护 | 演示加 99 主题；真机加 `--allow-real` |
| 目标不明确被拒绝 | 用 `A3-17` 或 `第三排第十七盆` 重说 |
| 动作不明确被拒绝 | 明确说“叉取/修剪”或“放回/放到花篮” |
| 一直等回执 | TL 程序未启动、主题不对、taskId 不匹配；检查 broker |
| retCode=2 | TL 正在执行别的任务，等结束 |
| retCode=3 | 识码/放置失败；看 VLM 快照、二维码遮挡/贴反 |
| Ollama 连不上 | `ollama ps` / `systemctl status ollama` |
| VLM 无快照 | TL 补丁未生效或还没检测过目标 |
| 急停没反应 | 物理急停优先；检查 MQTT 连接和 TL 的 estop 锁存 |

## 验收清单

- [ ] `--selftest` 全部通过
- [ ] dry-run 两种说法都解析正确（A3-17 / 第三排第十七盆）
- [ ] 99 主题 mock：seed 0、trim 1→0、estop 立即发送
- [ ] 真实主题不加 `--allow-real` 被拒绝
- [ ] 真机 seed 一次回执 0
- [ ] 真机 trim_pick/trim_place 能看到 1→0
- [ ] VLM“看一眼”有报告（如果启用）
- [ ] 物理急停在手边

## 文件

```text
TL_llm_agent/
├── llm_tl_agent.py      # 主程序
├── mock_tl.py           # 模拟执行端
├── run_llm_agent.sh     # 启动脚本
├── README.md            # 部署/协议/安全
├── SKILL.md             # 本文件
└── 验证报告_2026-10-22.md
```

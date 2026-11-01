#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TL 种子抓放 · 大模型(MQTT)控制代理
=====================================
中文指令 → 本地大模型(Ollama/phi3:mini) → 任务指令 JSON → 安全层校验
        → MQTT 下发(node/arm/01/down) → TL 现成程序执行(取种→识码→放置→回位)
        → 订阅回执(node/arm/01/up) → 人话报告结果

设计原则（与项目文档一致）：
  - MQTT 协议不变，本程序只做"语言 → 任务指令"的前端，不碰 TL 的任何文件
  - 大模型只输出任务级 JSON（cmd + targetId），绝不输出坐标/轨迹/速度
  - 安全层：cmd 白名单 + targetId 规则校验 + 规则解析交叉核对 + 单任务互斥
             + 默认直接执行（--confirm 才逐条 y/N）+ estop 直通

支持两种现场说法：
  ① 编号式："把种子放到 A3-17"            → {"cmd":"seed","targetId":"A3-17"}
  ② 序号式："把种子放到第三排第十七盆"      → {"cmd":"seed","targetId":"A3-17"}
     （第1~4排=A1~A4，第5~8排=B1~B4；每排 24 盆）

VLM 视觉辅助（可选，qwen2.5vl:3b）：
  TL 程序在观测位检测时写入快照（~/tl_llm_agent/frames/latest.jpg），
  任务结束或输入“看一眼”时由 VLM 复核画面（二维码清晰度/遮挡/异常），
  只做文字报告，不参与任何动作决策。

用法：
  python3 llm_tl_agent.py                # dry-run：只解析+校验，不发送（可无机械臂/无MQTT）
  python3 llm_tl_agent.py --execute      # 实跑：连接 MQTT 并下发
  python3 llm_tl_agent.py --once "把种子放到第三排第十七盆"   # 单条测试
  python3 llm_tl_agent.py --selftest     # 离线自检（解析器/校验，不需要 Ollama/MQTT）
"""

import argparse
import base64
import json
import os
import queue
import random
import re
import sys
import threading
import time
import urllib.error
import urllib.request

# ==================== 默认配置（与 TL 工程一致） ====================
DEFAULT_OLLAMA_URL = "http://127.0.0.1:11434"
DEFAULT_MODEL = "phi3:mini"

# VLM 视觉辅助（qwen2.5vl 只做画面复核报告，不参与任何动作决策）
DEFAULT_VLM_MODEL = "qwen2.5vl:3b"
DEFAULT_FRAME_DIR = "~/tl_llm_agent/frames"   # TL 程序在此写入观测位最新一帧
VLM_PROMPT = (
    "这是机械臂相机拍到的花盆架图片。请用简短中文回答三个问题：\n"
    "1. 画面里有没有黑白方块二维码（ArUco 码）？\n"
    "2. 二维码是否清晰完整（有没有贴反、被遮挡、模糊）？\n"
    "3. 花盆附近有没有明显异常？\n"
    "看不清或与问题无关时直接回答\"看不清\"。"
)

DEFAULT_MQTT_HOST = "116.205.245.172"
DEFAULT_MQTT_PORT = 1883
DEFAULT_MQTT_USERNAME = "shapandevice"
DEFAULT_MQTT_PASSWORD = "qwer1234!!"
TOPIC_DOWN = "node/arm/01/down"
TOPIC_UP = "node/arm/01/up"

# ==================== 协议常量（与 tl_hand/mqtt_protocol.py 保持一致） ====================
SUPPORTED_COMMANDS = {"seed", "seed_place", "trim_pick", "trim_place", "estop"}

# Basket rack layout: 2 regions (A/B) x 4 rows x 24 baskets = 192 baskets.
ROW_PREFIXES = {
    "A1": 0, "A2": 24, "A3": 48, "A4": 72,
    "B1": 96, "B2": 120, "B3": 144, "B4": 168,
}
ROW_INDEX_RE = re.compile(r"^(A[1-4]|B[1-4])-(\d{1,2})$")
# 口语排号 1~8 → 行前缀
ROW_ORDER = ["A1", "A2", "A3", "A4", "B1", "B2", "B3", "B4"]

# retCode 语义（与 TL 协议一致）
RET_SUCCESS = 0
RET_FAILED = 1          # seed*: 失败
RET_STARTED = 1         # trim*: 已开始动作（非终态，继续等最终结果）
RET_OCCUPIED = 2
RET_RECOG_FAILED = 3    # trim 终态：识码/放置失败

RETCODE_TEXT = {
    "seed": {0: "成功", 1: "已开始（中间态）", 2: "设备忙（有任务在跑）",
             3: "取种/识码/放置失败"},
    "seed_place": {0: "成功", 1: "已开始（中间态）", 2: "设备忙（有任务在跑）",
                   3: "识码/放置失败"},
    "trim_pick": {0: "成功", 3: "识码/放置失败", 2: "设备忙（有任务在跑）"},
    "trim_place": {0: "成功", 3: "识码/放置失败", 2: "设备忙（有任务在跑）"},
}

CN_DIGITS = {"零": 0, "一": 1, "二": 2, "三": 3, "四": 4,
             "五": 5, "六": 6, "七": 7, "八": 8, "九": 9}

# ==================== 大模型提示词 ====================
SYSTEM_PROMPT = """你是 TL 机械臂种子抓放系统的指令翻译器。
把用户的一句话翻译成 JSON。只输出 JSON，不要解释，不要输出其它任何内容。

可用命令 cmd：
- "seed"      放种子/种种子/播种（完整周期：取种+识别花盆码+放置）
- "seed_place" 种子放置（车端"放种子放置"命令，与 seed 相同完整周期）
- "trim_pick" 叉取/修剪/取花篮
- "trim_place" 放回/放置花篮
- "estop"     急停/停止/停下（不带 targetId）
- "none"      听不懂、缺目标或无法确定

targetId 规则（花架布局：A/B 两区 × 4排 × 24盆 = A1-01 ~ B4-24）：
- 编号式直接用：A3-17（两位数字补零，如 A3-05）
- 口语"第N排第M盆"：第1、2、3、4排 = A1、A2、A3、A4；第5、6、7、8排 = B1、B2、B3、B4；
  M 范围 1~24。例："第三排第十七盆" → A3-17；"第五排第三盆" → B1-03
- 如果用户只说"放到第X排第Y盆"，命令默认用 seed

输出格式（三选一）：
{"cmd":"seed","targetId":"A3-17"}
{"cmd":"estop"}
{"cmd":"none"}
"""

FEWSHOT = [
    {"role": "user", "content": "把种子放到 A3-17"},
    {"role": "assistant", "content": '{"cmd":"seed","targetId":"A3-17"}'},
    {"role": "user", "content": "放到第三排第十七盆"},
    {"role": "assistant", "content": '{"cmd":"seed","targetId":"A3-17"}'},
    {"role": "user", "content": "种到第五排第三盆"},
    {"role": "assistant", "content": '{"cmd":"seed","targetId":"B1-03"}'},
    {"role": "user", "content": "把种子放到 B2 区第六盆"},
    {"role": "assistant", "content": '{"cmd":"seed","targetId":"B2-06"}'},
    {"role": "user", "content": "叉取 A4-01 的花篮"},
    {"role": "assistant", "content": '{"cmd":"trim_pick","targetId":"A4-01"}'},
    {"role": "user", "content": "急停"},
    {"role": "assistant", "content": '{"cmd":"estop"}'},
    {"role": "user", "content": "今天天气怎么样"},
    {"role": "assistant", "content": '{"cmd":"none"}'},
]


# ==================== 工具函数 ====================
def parse_cn_number(text):
    """解析中文/阿拉伯数字：一..二十四。返回 int 或 None。"""
    text = str(text).strip()
    if not text:
        return None
    if text.isdigit():
        return int(text)
    if text == "十":
        return 10
    if text.startswith("十"):
        rest = text[1:]
        if len(rest) == 1 and rest in CN_DIGITS:
            return 10 + CN_DIGITS[rest]
        return None
    if "十" in text:
        tens_text, _, ones_text = text.partition("十")
        tens = CN_DIGITS.get(tens_text)
        if tens is None:
            return None
        if ones_text == "":
            return tens * 10
        if len(ones_text) == 1 and ones_text in CN_DIGITS:
            return tens * 10 + CN_DIGITS[ones_text]
        return None
    if len(text) == 1 and text in CN_DIGITS:
        return CN_DIGITS[text]
    return None


def normalize_target(value):
    """把各种写法的目标统一成 'A3-17'（两位补零）。非法返回 None。"""
    if value is None:
        return None
    s = str(value).strip().upper()
    s = s.replace("－", "-").replace("—", "-").replace("_", "-").replace(" ", "")
    match = ROW_INDEX_RE.match(s)
    if not match:
        return None
    row, index = match.group(1), int(match.group(2))
    if not 1 <= index <= 24:
        return None
    return f"{row}-{index:02d}"


def basket_number(target_id):
    """'A3-17' → 盆号 49（1..192）。"""
    match = ROW_INDEX_RE.match(target_id or "")
    if not match:
        return None
    return ROW_PREFIXES[match.group(1)] + int(match.group(2))


def rule_parse_target(text):
    """规则解析目标盆（确定性，用于和 LLM 结果交叉核对）。返回 targetId 或 None。"""
    if not text:
        return None
    # ① 编号式：A3-17 / a3 - 5 / A3排第17盆
    match = re.search(r"([ABab])\s*([1-4])\s*[-－—_]\s*(\d{1,2})", text)
    if match:
        target = normalize_target(f"{match.group(1).upper()}{match.group(2)}-{match.group(3)}")
        if target:
            return target
    match = re.search(r"([ABab])\s*([1-4])\s*排[^0-9一二三四五六七八九十]{0,4}"
                      r"(?:第\s*)?(\d{1,2})\s*(?:盆|号|位)", text)
    if match:
        target = normalize_target(f"{match.group(1).upper()}{match.group(2)}-{match.group(3)}")
        if target:
            return target
    # "B2 区第六盆" 式：区域码 + 中文盆号
    match = re.search(r"([ABab])\s*([1-4])\s*区[^0-9一二三四五六七八九十]{0,4}"
                      r"(?:第\s*)?([0-9一二三四五六七八九十]{1,3})\s*(?:盆|号|位)", text)
    if match:
        pot = parse_cn_number(match.group(3))
        if pot is not None and 1 <= pot <= 24:
            return f"{match.group(1).upper()}{match.group(2)}-{pot:02d}"
    # ② 序号式：第X排第Y盆（第1-4排=A1-A4，第5-8排=B1-B4）
    match = re.search(r"第\s*([一二三四五六七八1-8])\s*排[^0-9一二三四五六七八九十]{0,6}"
                      r"(?:第\s*)?([0-9一二三四五六七八九十]{1,3})\s*(?:盆|号|位)", text)
    if match:
        row = parse_cn_number(match.group(1))
        pot = parse_cn_number(match.group(2))
        if row is not None and pot is not None and 1 <= row <= 8 and 1 <= pot <= 24:
            return f"{ROW_ORDER[row - 1]}-{pot:02d}"
    return None


def rule_parse_cmd(text):
    """规则解析命令（关键词）。返回 cmd 或 None。"""
    if not text:
        return None
    if re.search(r"急停|停止|停下|停机|\bstop\b", text):
        return "estop"
    if re.search(r"花篮|叉取|叉走|修剪|叉", text):
        has_pick = bool(re.search(r"叉取|叉走|修剪|叉", text))
        has_place = bool(re.search(r"放回|放好|放到|放在|放置|摆|卸", text))
        if has_pick and has_place:
            return None  # “叉取…放到…”动作歧义，交给安全层拒绝
        if has_place:
            return "trim_place"
        return "trim_pick"
    if re.search(r"种子|种球|播种|种到|种下", text):
        return "seed_place" if re.search(r"种子放置", text) else "seed"
    return None


def extract_json(text):
    """从模型输出里提取含 cmd 的 JSON 对象（容错 think 块/代码块/截断）。"""
    if not text:
        return None
    cleaned = re.sub(r"```(?:json)?", "", text)
    cleaned = re.sub(r"[\s\S]*?<｜end▁of▁thinking｜>", "", cleaned)
    candidates = re.findall(r"\{[^{}]*\}", cleaned)
    for candidate in reversed(candidates):
        if '"cmd"' in candidate or "'cmd'" in candidate:
            try:
                obj = json.loads(candidate)
                if isinstance(obj, dict):
                    return obj
            except json.JSONDecodeError:
                continue
    # 截断修复：从第一个 '{' 起补右括号
    start = cleaned.find("{")
    if start != -1:
        repaired = cleaned[start:].strip()
        if not repaired.endswith("}"):
            repaired += "}"
        try:
            obj = json.loads(repaired)
            if isinstance(obj, dict):
                return obj
        except json.JSONDecodeError:
            pass
    return None


def validate_command(obj):
    """校验 LLM 输出 → (cmd, targetId)。非法抛 ValueError。"""
    if not isinstance(obj, dict):
        raise ValueError("模型输出不是 JSON 对象")
    cmd = str(obj.get("cmd", "")).strip().lower()
    if cmd not in SUPPORTED_COMMANDS:
        raise ValueError(f"未知命令 cmd={cmd!r}（白名单: {sorted(SUPPORTED_COMMANDS)}）")
    if cmd == "estop":
        return "estop", ""
    target = normalize_target(obj.get("targetId"))
    if not target:
        raise ValueError(f"targetId 非法: {obj.get('targetId')!r}（应为 A1-01 ~ B4-24）")
    return cmd, target


# ==================== Ollama 调用 ====================
class LlmClient:
    def __init__(self, url=DEFAULT_OLLAMA_URL, model=DEFAULT_MODEL, timeout=120):
        self.url = url.rstrip("/")
        self.model = model
        self.timeout = timeout

    def _post(self, path, payload, timeout=None):
        request = urllib.request.Request(
            self.url + path,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=timeout or self.timeout) as response:
            return json.loads(response.read().decode("utf-8"))

    def chat(self, user_text):
        """调用 phi3:mini 解析指令，返回原始文本。失败抛异常。"""
        messages = [{"role": "system", "content": SYSTEM_PROMPT}]
        messages.extend(FEWSHOT)
        messages.append({"role": "user", "content": user_text})
        payload = {
            "model": self.model,
            "messages": messages,
            "stream": False,
            "format": "json",
            "options": {"temperature": 0.1, "num_predict": 120},
        }
        try:
            response = self._post("/api/chat", payload)
        except (urllib.error.URLError, TimeoutError, OSError) as error:
            raise RuntimeError(f"连不上 Ollama({self.url}): {error}") from error
        content = (response.get("message") or {}).get("content", "")
        if not content.strip():
            # 已知问题兜底：空回复 → 卸载模型后重试一次
            try:
                self._post("/api/generate", {"model": self.model, "keep_alive": 0}, timeout=30)
                response = self._post("/api/chat", payload)
                content = (response.get("message") or {}).get("content", "")
            except Exception:
                pass
        if not content.strip():
            raise RuntimeError("Ollama 返回空回复（模型可能需重新加载，重试一次）")
        return content

    def parse(self, text):
        """返回 (cmd, targetId, raw_text)；解析/校验失败返回 (None, None, raw)。"""
        raw = self.chat(text)
        obj = extract_json(raw)
        if obj is None:
            return None, None, raw
        try:
            cmd, target = validate_command(obj)
            return cmd, target, raw
        except ValueError:
            return None, None, raw


# ==================== VLM 视觉辅助 ====================
class VlmClient:
    """qwen2.5vl:3b —— 只做画面复核报告，不参与任何动作决策。"""

    def __init__(self, url=DEFAULT_OLLAMA_URL, model=DEFAULT_VLM_MODEL, timeout=120):
        self.url = url.rstrip("/")
        self.model = model
        self.timeout = timeout

    def analyze(self, image_path):
        with open(image_path, "rb") as handle:
            image_b64 = base64.b64encode(handle.read()).decode("ascii")
        payload = {
            "model": self.model,
            "messages": [{
                "role": "user",
                "content": VLM_PROMPT,
                "images": [image_b64],
            }],
            "stream": False,
            "options": {"temperature": 0.1, "num_predict": 200},
        }
        request = urllib.request.Request(
            self.url + "/api/chat",
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=self.timeout) as response:
            result = json.loads(response.read().decode("utf-8"))
        content = (result.get("message") or {}).get("content", "").strip()
        if not content:
            raise RuntimeError("VLM 返回空内容")
        return content


def latest_snapshot(frame_dir, max_age=180.0):
    """返回 TL 写入的最新快照路径（过期返回 None）。"""
    root = os.path.expanduser(frame_dir)
    image_path = os.path.join(root, "latest.jpg")
    if not os.path.exists(image_path):
        return None
    stamp = None
    try:
        with open(os.path.join(root, "latest.json"), "r", encoding="utf-8") as handle:
            stamp = float(json.load(handle).get("time"))
    except Exception:
        stamp = None
    if stamp is None:
        stamp = os.path.getmtime(image_path)
    if time.time() - stamp > max_age:
        return None
    return image_path


def vlm_check(vlm_client, args, note=""):
    """读取最新快照做视觉复核（仅供人工参考）。"""
    if vlm_client is None:
        return
    path = latest_snapshot(args.frame_dir, args.vlm_max_age)
    if path is None:
        print("🔍 视觉辅助: 没有新鲜画面快照（TL 需先运行过观测位检测；不需要可用 --no-vlm 关闭）")
        return
    print("🔍 视觉辅助分析中（仅供参考，不参与动作决策）…")
    try:
        answer = vlm_client.analyze(path)
        print(f"🔍 视觉辅助: {answer}" + (f"  [{note}]" if note else ""))
    except Exception as error:
        print(f"🔍 视觉辅助不可用: {error}")


# ==================== MQTT 客户端（下发 + 等回执） ====================
class TlMqttLink:
    def __init__(self, host, port, username, password, topic_down, topic_up,
                 client_id=None, keepalive=60):
        self.host = host
        self.port = int(port)
        self.username = username
        self.password = password
        self.topic_down = topic_down
        self.topic_up = topic_up
        self.client_id = client_id or f"llm-tl-agent-{random.randint(1000, 9999)}"
        self.keepalive = keepalive
        self.feedback = queue.Queue()
        self._client = None
        self._connected = threading.Event()
        self._lock = threading.Lock()

    def connect(self, timeout=10.0):
        try:
            import paho.mqtt.client as mqtt
        except ImportError as error:
            raise RuntimeError("缺少 paho-mqtt：请先 pip install paho-mqtt（或 apt install python3-paho-mqtt）") from error
        try:
            self._client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=self.client_id)
        except AttributeError:
            self._client = mqtt.Client(client_id=self.client_id)
        if self.username:
            self._client.username_pw_set(self.username, self.password)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.connect_async(self.host, self.port, self.keepalive)
        self._client.loop_start()
        print(f"📡 连接 MQTT broker {self.host}:{self.port} ...")
        if not self._connected.wait(timeout=timeout):
            self.stop()
            raise RuntimeError(f"MQTT broker {self.host}:{self.port} 连接超时（检查网络/账号/防火墙）")
        print(f"✅ MQTT 已连接，订阅回执: {self.topic_up}")

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        code = getattr(rc, "value", rc)
        if int(code) == 0:
            client.subscribe(self.topic_up, qos=1)
            self._connected.set()
        else:
            print(f"❌ MQTT 连接被拒: rc={code}")

    def _on_message(self, client, userdata, message):
        try:
            payload = json.loads(message.payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            print(f"⚠️ 收到非 JSON 回执: {message.payload!r}")
            return
        cmd = str(payload.get("cmd", ""))
        target = str(payload.get("targetId", ""))
        code = payload.get("retCode")
        text = payload.get("message")
        print(f"\n📩 回执: cmd={cmd} targetId={target or '-'} retCode={code}"
              f"{' message=' + str(text) if text else ''}")
        self.feedback.put((cmd, target, code, text))

    def publish_command(self, cmd, target_id=""):
        payload = {"cmd": cmd}
        if cmd != "estop":
            payload["targetId"] = target_id
        data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        with self._lock:
            result = self._client.publish(self.topic_down, data, qos=1)
            if result.rc != 0:
                raise RuntimeError(f"MQTT 发布失败: rc={result.rc}")
        print(f"📤 已下发 {self.topic_down}: {data}")

    def wait_result(self, cmd, target_id="", timeout=120.0, legacy_seed=False):
        """等匹配的回执。返回 (retCode, message, 是否超时, 最终阶段命令名)。

        种子程序是两段式反馈（TL_lowerobs_test / TL_Handback 均如此）：
          seed:1 已接受 → seed:0 取种完成（非终态）→ seed_place:1 开始放置
          → seed_place:0 最终成功（失败为 2/3）
        旧执行端只回一条 seed:0 时用 legacy_seed=True（见 --legacy-seed）。
        """
        deadline = time.time() + timeout
        last_note = 0
        seed_two_phase = cmd in ("seed", "seed_place") and not legacy_seed
        final_cmd = cmd
        while time.time() < deadline:
            try:
                fcmd, ftarget, code, text = self.feedback.get(timeout=1.0)
            except queue.Empty:
                remain = deadline - time.time()
                if remain <= 60 and (time.time() - last_note) >= 10:
                    print(f"⏳ 等待回执中… 剩余 {remain:.0f}s（可用 MQTT 急停或物理急停）")
                    last_note = time.time()
                continue
            if code is None:
                print("[提示] 回执缺少 retCode，忽略")
                continue
            if fcmd != cmd:
                # 种子两段式：取种阶段回 seed、放置阶段回 seed_place，两者都收
                if not (seed_two_phase and fcmd in ("seed", "seed_place")):
                    print(f"[提示] 忽略其它任务回执: cmd={fcmd}")
                    continue
            if cmd != "estop" and target_id and ftarget and ftarget.upper() != target_id:
                print(f"[提示] 忽略其它目标回执: targetId={ftarget}")
                continue
            if seed_two_phase:
                final_cmd = fcmd
                if fcmd == "seed" and code == RET_STARTED:
                    print("▶ 已接受，开始取种…")
                    continue
                if fcmd == "seed" and code == RET_SUCCESS:
                    print("✅ 取种完成，等待放置阶段（seed_place）…")
                    continue
                if fcmd == "seed_place" and code == RET_STARTED:
                    print("▶ 开始放置（等待最终结果）…")
                    continue
                # seed 阶段的失败，或 seed_place 的最终结果（0/2/3）
                return code, text, False, final_cmd
            # trim 协议：retCode=1 表示"已开始动作"，非终态
            if cmd.startswith("trim") and code == RET_STARTED:
                print("▶ 已开始动作（等待最终结果）…")
                continue
            return code, text, False, fcmd
        return None, None, True, final_cmd

    def stop(self):
        if self._client is None:
            return
        try:
            self._client.loop_stop()
            self._client.disconnect()
        except Exception:
            pass
        self._client = None


# ==================== 安全层：合并规则与 LLM 结果 ====================
def resolve_command(text, llm_client):
    """解析一句话 → (cmd, target_id) 或抛 ValueError。"""
    rule_cmd = rule_parse_cmd(text)
    rule_target = rule_parse_target(text)

    llm_cmd, llm_target, raw = None, None, ""
    if llm_client is not None:
        try:
            llm_cmd, llm_target, raw = llm_client.parse(text)
            print(f"[LLM] {raw.strip()[:160]}")
        except RuntimeError as error:
            print(f"⚠️ 大模型不可用，改用规则解析: {error}")
    else:
        print("[LLM] 已禁用（--no-llm），使用规则解析")

    if llm_target and rule_target and llm_target != rule_target:
        raise ValueError(
            f"大模型解析({llm_target})与规则解析({rule_target})不一致，"
            f"为避免放错盆已拒绝。请用明确编号重说，例如：放到 {rule_target}"
        )

    # 急停允许没有目标；其他命令必须有规则层解析出的目标
    if rule_cmd == "estop" or (rule_cmd is None and llm_cmd == "estop"):
        return "estop", ""
    if rule_target is None:
        raise ValueError(
            "没有从话里解析出明确的目标盆编号；LLM 猜测的目标不予采用。"
            "请说 A3-17 或 第三排第十七盆"
        )
    target = rule_target

    # 动作以规则层为准；LLM 不能凭空补出“叉取/放回”
    if rule_cmd is None:
        if llm_cmd in (None, "seed", "seed_place"):
            cmd = "seed"  # 只说了目标盆 → 默认放种子
        else:
            raise ValueError(
                f"动作不明确：大模型解析为 {llm_cmd}，但句子里没有对应的动作词；"
                "请明确说 叉取/放回/种子"
            )
    else:
        cmd = rule_cmd
        if llm_cmd and llm_cmd != rule_cmd:
            print(f"[提示] 规则动作({rule_cmd})与 LLM({llm_cmd}) 不一致，按规则执行")

    return cmd, target


def confirm_and_print(cmd, target_id, execute, confirm):
    number = basket_number(target_id) if target_id else None
    desc = {
        "seed": "放种子（取种→识码→放置→回位）",
        "seed_place": "放种子放置（完整周期）",
        "trim_pick": "叉取花篮",
        "trim_place": "放置花篮",
        "estop": "急停（软件锁存，物理急停优先）",
    }.get(cmd, cmd)
    target_text = f"目标盆 {target_id}" + (f"（盆号 {number}）" if number else "")
    print(f"\n📋 执行计划: {cmd} → {desc}")
    if target_id:
        print(f"   {target_text}")
    if not execute:
        print("   (dry-run：不发送 MQTT；加 --execute 实跑)")
        return True
    if cmd == "estop":
        # 急停不等待人工确认：安全优先
        return True
    if not confirm:
        # 默认直接执行（与 Dobot 版一致）；需要逐条确认请加 --confirm
        return True
    answer = input("   确认执行? [y/N] > ").strip().lower()
    return answer in ("y", "yes", "是")


# ==================== 自检 ====================
def selftest():
    cases = [
        ("把种子放到 A3-17", "seed", "A3-17"),
        ("放到 a3-5", "seed", "A3-05"),
        ("放到第三排第十七盆", "seed", "A3-17"),
        ("种到第五排第三盆", "seed", "B1-03"),
        ("放到第二排第二十四盆", "seed", "A2-24"),
        ("把种子放到 B2 区第六盆", "seed", "B2-06"),
        ("叉取 A4-01 的花篮", "trim_pick", "A4-01"),
        ("把花篮放到 A4-01", "trim_place", "A4-01"),
        ("把花篮放回 A4-01", "trim_place", "A4-01"),
        ("A3排第17盆", "seed", "A3-17"),
        ("急停", "estop", None),
    ]
    failures = 0
    for text, want_cmd, want_target in cases:
        cmd = rule_parse_cmd(text)
        target = rule_parse_target(text)
        if cmd is None and target is not None:
            cmd = "seed"
        ok = (cmd == want_cmd and target == want_target)
        failures += 0 if ok else 1
        print(f"{'✅' if ok else '❌'}  {text!r:28s} → cmd={cmd}, target={target}")
    # 非法值
    for bad in ["A3-25", "C1-01", "A5-01", "A3-0"]:
        if normalize_target(bad) is not None:
            failures += 1
            print(f"❌ 非法值未被拒绝: {bad}")

    # 安全层：LLM 不能补目标、不能凭空补叉取/放回动作
    class FakeLlm:
        def __init__(self, cmd, target):
            self.cmd = cmd
            self.target = target

        def parse(self, text):
            return self.cmd, self.target, ""

    safety_cases = [
        ("把种子放到那边", "seed", "A1-01", False),       # 规则没有目标 → 拒绝 LLM 猜的 A1-01
        ("A3-17", "trim_pick", "A3-17", False),           # 规则没有动作 → 拒绝 LLM 猜的叉取
        ("A3-17", "seed", "A3-17", True),                 # 只给目标 → 默认放种子
        ("叉取花篮放到 A4-01", "trim_pick", "A4-01", False),  # 动作歧义 → 拒绝
    ]
    for text, cmd, target, should_pass in safety_cases:
        try:
            result = resolve_command(text, FakeLlm(cmd, target))
            ok = should_pass
            print(f"{'✅' if ok else '❌'}  安全层 {'放行' if ok else '未拦截'}: {text!r} → {result}")
        except ValueError as error:
            ok = not should_pass
            print(f"{'✅' if ok else '❌'}  安全层 {'拦截' if ok else '误拦截'}: {text!r} → {error}")
        failures += 0 if ok else 1

    # 两段式种子回执（TL_lowerobs_test / TL_Handback 的真实协议）
    wait_cases = [
        ("两段式成功 seed:1→seed:0→seed_place:1→seed_place:0", [
            ("seed", "A3-17", 1, None),
            ("seed", "A3-17", 0, None),
            ("seed_place", "A3-17", 1, None),
            ("seed_place", "A3-17", 0, None),
        ], False, 0),
        ("放置阶段失败 seed_place:3", [
            ("seed", "A3-17", 0, None),
            ("seed_place", "A3-17", 3, None),
        ], False, 3),
        ("旧执行端 seed:0 即最终成功", [
            ("seed", "A3-17", 0, None),
        ], True, 0),
    ]
    for label, messages, legacy, want in wait_cases:
        link = TlMqttLink("127.0.0.1", 1883, "", "", "d", "u")
        for message in messages:
            link.feedback.put(message)
        code = None
        try:
            code, _text, timed_out, _final = link.wait_result(
                "seed", "A3-17", timeout=2.0, legacy_seed=legacy)
            ok = (not timed_out and code == want)
        except Exception as error:
            ok = False
            code = f"异常: {error}"
        failures += 0 if ok else 1
        print(f"{'✅' if ok else '❌'}  种子回执: {label} → code={code}")

    print(f"\n自检: {'全部通过' if failures == 0 else f'{failures} 项失败'}")
    return 0 if failures == 0 else 1


# ==================== 主程序 ====================
def main():
    parser = argparse.ArgumentParser(description="TL 种子抓放 · 大模型(MQTT)控制代理")
    parser.add_argument("--execute", action="store_true", help="实跑：连接 MQTT 并下发（默认 dry-run）")
    parser.add_argument("--confirm", action="store_true",
                        help="执行前逐条询问 y/N（默认直接执行）")
    parser.add_argument("--yes", action="store_true",
                        help="兼容旧参数：默认已不询问，--yes 不再需要")
    parser.add_argument("--once", default=None, help="只处理一条指令后退出")
    parser.add_argument("--selftest", action="store_true", help="离线自检后退出")
    parser.add_argument("--no-llm", action="store_true", help="禁用大模型，只用规则解析")
    parser.add_argument("--ollama-url", default=DEFAULT_OLLAMA_URL, help=f"Ollama 地址（默认 {DEFAULT_OLLAMA_URL}）")
    parser.add_argument("--model", default=DEFAULT_MODEL, help=f"模型名（默认 {DEFAULT_MODEL}）")
    parser.add_argument("--broker", default=DEFAULT_MQTT_HOST, help=f"MQTT broker（默认 {DEFAULT_MQTT_HOST}）")
    parser.add_argument("--port", type=int, default=DEFAULT_MQTT_PORT)
    parser.add_argument("--username", default=DEFAULT_MQTT_USERNAME)
    parser.add_argument("--password", default=DEFAULT_MQTT_PASSWORD)
    parser.add_argument("--topic-down", default=TOPIC_DOWN)
    parser.add_argument("--topic-up", default=TOPIC_UP)
    parser.add_argument("--allow-real", action="store_true",
                        help="允许向真实主题 node/arm/01/* 下发（真机执行必须显式加）")
    parser.add_argument("--timeout", type=float, default=120.0, help="等待回执超时秒数（默认120）")
    parser.add_argument("--legacy-seed", action="store_true",
                        help="兼容旧执行端：收到 seed:0 即视为整个周期成功（不等待 seed_place）")
    parser.add_argument("--no-vlm", action="store_true", help="禁用 VLM 视觉辅助")
    parser.add_argument("--vlm-model", default=DEFAULT_VLM_MODEL, help=f"视觉模型（默认 {DEFAULT_VLM_MODEL}）")
    parser.add_argument("--frame-dir", default=DEFAULT_FRAME_DIR, help="TL 快照目录（默认 ~/tl_llm_agent/frames）")
    parser.add_argument("--vlm-max-age", type=float, default=180.0, help="快照最大有效秒数（默认180）")
    args = parser.parse_args()

    if args.selftest:
        sys.exit(selftest())

    # 真实主题保护：测试默认必须用 99 演示主题
    real_topics = (TOPIC_DOWN, TOPIC_UP)
    if args.execute and (args.topic_down in real_topics or args.topic_up in real_topics):
        if not args.allow_real:
            print("❌ 当前使用真实主题 node/arm/01/*，已拒绝连接。")
            print("   测试/演示请用：")
            print("   --topic-down node/arm/99/down --topic-up node/arm/99/up")
            print("   确认真机执行请加 --allow-real")
            sys.exit(1)
        print("⚠️ 已允许真实主题：指令会直接发给在线的 TL 机械臂！物理急停在手边。")

    llm_client = None if args.no_llm else LlmClient(args.ollama_url, args.model)
    vlm_client = None if args.no_vlm else VlmClient(args.ollama_url, args.vlm_model)
    link = None

    if args.execute:
        link = TlMqttLink(args.broker, args.port, args.username, args.password,
                          args.topic_down, args.topic_up)
        try:
            link.connect()
        except RuntimeError as error:
            print(f"❌ {error}")
            sys.exit(1)
    else:
        print("ℹ️ dry-run 模式（只解析不发送）。实跑请加 --execute")
    if vlm_client is not None:
        print(f"ℹ️ VLM 视觉辅助: {args.vlm_model}（任务结束后自动复核；输入“看一眼”手动复核）")
    else:
        print("ℹ️ VLM 视觉辅助: 已禁用（--no-vlm）")
    if args.execute:
        print(f"ℹ️ 执行确认: {'逐条 y/N' if args.confirm else '直接执行（需要 y/N 请加 --confirm）'}")
    if args.yes:
        print("ℹ️ --yes 已无效：默认就不需要 y/N")

    def handle(text):
        text = text.strip()
        if not text:
            return
        try:
            cmd, target = resolve_command(text, llm_client)
        except ValueError as error:
            print(f"⛔ {error}\n")
            return
        if not confirm_and_print(cmd, target, args.execute, args.confirm):
            print("已取消。\n")
            return
        if not args.execute:
            print("✅ 解析通过（dry-run 未发送）\n")
            return
        try:
            link.publish_command(cmd, target)
        except RuntimeError as error:
            print(f"❌ 发送失败: {error}\n")
            return
        if cmd == "estop":
            print("🛑 急停指令已发送（软件锁存）。物理急停优先！\n")
            return
        code, message, timed_out, final_cmd = link.wait_result(
            cmd, target, args.timeout, legacy_seed=args.legacy_seed)
        if timed_out:
            print(f"⌛ 等待回执超时（{args.timeout:.0f}s），请检查 TL 程序是否在运行\n")
            return
        code_int = int(code) if code is not None else -1
        meaning = RETCODE_TEXT.get(final_cmd, RETCODE_TEXT.get(cmd, {})).get(
            code_int, "未知结果")
        ok = (code_int == RET_SUCCESS)
        print(f"{'✅' if ok else '❌'} 结果: retCode={code} {meaning}"
              f"{' message=' + str(message) if message else ''} → {target}")
        note = "识码/放置失败，重点看二维码是否清晰、是否被遮挡" if code_int == RET_RECOG_FAILED else ""
        vlm_check(vlm_client, args, note=note)
        print()

    try:
        if args.once is not None:
            handle(args.once)
        else:
            print("=" * 60)
            print(" TL 种子抓放 · 大模型控制代理")
            print(" 输入示例: 把种子放到 A3-17 / 放到第三排第十七盆 / 急停 / 退出")
            print("=" * 60)
            while True:
                try:
                    text = input("\n> ")
                except (EOFError, KeyboardInterrupt):
                    print("\n退出")
                    break
                if text.strip().lower() in ("q", "quit", "exit", "退出"):
                    print("退出")
                    break
                if text.strip().lower() in ("看一眼", "看看", "检查", "check", "look"):
                    vlm_check(vlm_client, args)
                    continue
                handle(text)
    finally:
        if link is not None:
            link.stop()


if __name__ == "__main__":
    main()

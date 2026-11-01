#!/usr/bin/env python3
# coding=utf-8
"""
大模型控制天链机械臂演示（TL / MQTT 版）
==========================================
中文 → Ollama(phi3:mini) → JSON 指令 → 白名单/编号校验 → MQTT → 你原有程序执行（一行不改）

命令协议(与 TL_lowerobs_test 完全一致, 发布到 node/arm/01/down):
  {"cmd":"trim_pick","targetId":"A3-11"}   取花: 从指定花盆取花到修剪位
  {"cmd":"trim_place","targetId":"A3-11"}  放回: 把花放回指定花盆
  {"cmd":"seed","targetId":"A1-05"}        播种: 给指定花盆完整放种周期(较慢)
  {"cmd":"estop"}                          急停: 停止运动流(锁存, 需在Nano端重启程序解锁)
反馈(订阅 node/arm/01/up): {"cmd":..,"targetId":..,"retCode":N}
  1=已收到开始动作   0=成功   2=位置被占用/忙碌   3=识别/放置失败

用法:
  python llm_tl_agent.py --check                  # broker 自检(只读, 不发命令)
  python llm_tl_agent.py --llm-only "修剪A3第11号"  # 只测 LLM→JSON(不连broker)
  python llm_tl_agent.py                           # 试运行: 连broker, 打印将发送的指令, 不发
  python llm_tl_agent.py --execute                 # 真实发送(每条需按 y 确认)

运行前提: Nano 上已运行 run_seed_mqtt.sh 或 run_fork_mqtt.sh(否则没人接收命令)。
"""
import os
import sys
import json
import time
import queue
import argparse
import threading

import requests

# ==================== MQTT 默认参数(与 TL_lowerobs_test/scripts 默认一致) ====================
MQTT_HOST = os.environ.get("TL_MQTT_HOST", "116.205.245.172")
MQTT_PORT = int(os.environ.get("TL_MQTT_PORT", "1883"))
MQTT_USER = os.environ.get("TL_MQTT_USERNAME", "shapandevice")
MQTT_PASSWORD = os.environ.get("TL_MQTT_PASSWORD", "qwer1234!!")
CMD_TOPIC = os.environ.get("TL_MQTT_CMD_TOPIC", "node/arm/01/down")
FB_TOPIC = os.environ.get("TL_MQTT_FB_TOPIC", "node/arm/01/up")
CLIENT_ID = "tl-llm-agent"

# ==================== 大模型参数 ====================
OLLAMA_URL = "http://127.0.0.1:11434"
DEFAULT_MODEL = "phi3:mini"   # 本机实测对中文+JSON最稳; 也可 --model 换
NUM_PREDICT = 500

# retCode 语义(与 TL_lowerobs_test/src/tl_hand/tl_hand/mqtt_protocol.py 一致)
RET_SUCCESS = 0
RET_STARTED = 1
RET_OCCUPIED = 2
RET_RECOG_FAILED = 3
RET_TEXT = {
    RET_SUCCESS: "成功",
    RET_STARTED: "已收到, 开始动作",
    RET_OCCUPIED: "位置被占用/机械臂忙碌",
    RET_RECOG_FAILED: "识别/放置失败",
}

ALLOWED_CMDS = {"trim_pick", "trim_place", "seed", "estop"}
EXIT_WORDS = {"q", "quit", "exit", "退出", "结束"}

# 少样本示例(多轮对话, 纠正小模型输出格式很有效)
FEW_SHOT = [
    ("修剪A3第11号", '{"cmd":"trim_pick","targetId":"A3-11"}'),
    ("给B2第5号播种", '{"cmd":"seed","targetId":"B2-05"}'),
    ("急停", '{"cmd":"estop"}'),
]

SYSTEM_PROMPT = """你是园艺机器人(六轴机械臂)的指令生成器。根据用户的话，输出且仅输出一个 JSON 对象，不要任何解释、不要 markdown 代码块。

可用指令(严格选其中一个):
1. {{"cmd":"trim_pick","targetId":"A3-11"}}   修剪/取花: 从指定花盆把花取到修剪位
2. {{"cmd":"trim_place","targetId":"A3-11"}}  放回/放花: 把花放回指定花盆
3. {{"cmd":"seed","targetId":"A1-05"}}        播种: 给指定花盆播种(完整放种周期,较慢)
4. {{"cmd":"estop"}}                          急停: 立即停止运动
5. {{"cmd":"none","reason":"原因"}}            只有当用户说的不是任何机械臂动作时使用

targetId 规则: 区域 A 或 B, 行 1~4, 编号 01~24, 格式 "行-两位编号", 例如 A3-11、B2-05、A1-24。
用户说"第11号/11号/11"都换算成 11; 个位数补零: 5 -> 05。

示例:
用户:修剪A3第11号 → {{"cmd":"trim_pick","targetId":"A3-11"}}
用户:把A3-11的花放回去 → {{"cmd":"trim_place","targetId":"A3-11"}}
用户:给B2第5号播种 → {{"cmd":"seed","targetId":"B2-05"}}
用户:停下 → {{"cmd":"estop"}}
用户:今天天气怎样 → {{"cmd":"none","reason":"这不是机械臂任务"}}
"""


# ==================== MQTT 客户端 ====================
class TlMqtt:
    """发布命令到 node/arm/01/down, 订阅 node/arm/01/up 等待反馈"""

    def __init__(self, host, port, user, password, cmd_topic, fb_topic):
        self.host = host
        self.port = int(port)
        self.user = user
        self.password = password
        self.cmd_topic = cmd_topic
        self.fb_topic = fb_topic
        self.client = None
        self.connected = False
        self.messages = queue.Queue()
        self._connect_event = threading.Event()
        self._connect_error = None
        self._last_seen = None  # 最近一条反馈原文(供 --check 显示)

    def connect(self, timeout=10.0):
        try:
            import paho.mqtt.client as mqtt
        except ImportError as e:
            raise ConnectionError("缺少 paho-mqtt, 先运行: pip install paho-mqtt") from e
        try:
            self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=CLIENT_ID)
        except AttributeError:
            self.client = mqtt.Client(client_id=CLIENT_ID)
        if self.user:
            self.client.username_pw_set(self.user, self.password)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.connect_async(self.host, self.port, 60)
        self.client.loop_start()
        if not self._connect_event.wait(timeout=timeout):
            self.client.loop_stop()
            raise ConnectionError(f"broker {self.host}:{self.port} {timeout:.0f}s 内未接受连接")
        if self._connect_error:
            raise ConnectionError(self._connect_error)
        return True

    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        rc = getattr(reason_code, "value", reason_code)
        if int(rc) != 0:
            self._connect_error = f"broker 拒绝连接 (reason={rc}); 检查用户名/密码"
            self._connect_event.set()
            return
        self.connected = True
        client.subscribe(self.fb_topic, qos=1)
        self._connect_event.set()

    def _on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode("utf-8"))
        except Exception:
            data = {"raw": msg.payload.decode(errors="ignore")}
        self._last_seen = json.dumps(data, ensure_ascii=False)
        self.messages.put(data)

    def publish_cmd(self, cmd, target_id=None):
        payload = {"cmd": cmd}
        if target_id:
            payload["targetId"] = target_id
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        result = self.client.publish(self.cmd_topic, body, qos=1)
        return result.rc == 0, body

    def wait_result(self, cmd, target_id, ack_timeout=20.0, total_timeout=300.0):
        """等本命令的最终反馈。返回 dict(success, code, text)"""
        t0 = time.time()
        saw_started = False
        while True:
            elapsed = time.time() - t0
            if not saw_started and elapsed > ack_timeout:
                return {"success": False, "code": None,
                        "text": f"{ack_timeout:.0f}s 内没有收到任何反馈: "
                                f"检查 Nano 上 run_seed_mqtt / run_fork_mqtt 是否在运行"}
            if elapsed > total_timeout:
                return {"success": False, "code": None,
                        "text": f"总等待超过 {total_timeout:.0f}s, 未收到最终结果"}
            try:
                msg = self.messages.get(timeout=0.5)
            except queue.Empty:
                continue
            print(f"     [反馈] {json.dumps(msg, ensure_ascii=False)}")
            if str(msg.get("cmd")) != cmd:
                continue
            tid = msg.get("targetId")
            if target_id and tid and str(tid) != str(target_id):
                continue
            code = msg.get("retCode")
            if code == RET_STARTED:
                saw_started = True
                print(f"     ⏳ 机械臂已收到, 开始动作 ...")
                continue
            if code == RET_SUCCESS:
                return {"success": True, "code": RET_SUCCESS, "text": "成功"}
            if code == RET_OCCUPIED:
                return {"success": False, "code": RET_OCCUPIED, "text": RET_TEXT[RET_OCCUPIED]}
            if code == RET_RECOG_FAILED:
                extra = msg.get("message", "")
                return {"success": False, "code": RET_RECOG_FAILED,
                        "text": RET_TEXT[RET_RECOG_FAILED] + (f" ({extra})" if extra else "")}
            # 未知 retCode: 打印后继续等待

    def close(self):
        if self.client is not None:
            try:
                self.client.loop_stop()
                self.client.disconnect()
            except Exception:
                pass


# ==================== 大模型部分 ====================
def build_system_prompt():
    return SYSTEM_PROMPT


def extract_json(text):
    import re
    if not text:
        return None
    text = re.sub(r'<think[^>]*>[\s\S]*?</think\s*>', ' ', text, flags=re.I)
    text = re.sub(r'```(?:json)?', ' ', text)
    t = text.strip()
    # 小模型偶尔截断(少右括号/引号), 尝试补齐
    for cand in (t, t + "}", t + '"}', t + '""}'):
        try:
            d = json.loads(cand)
            if isinstance(d, dict) and "cmd" in d:
                return d
        except Exception:
            pass
    best = None
    for m in re.finditer(r'\{[^{}]*\}', text):
        try:
            d = json.loads(m.group(0))
        except Exception:
            continue
        if isinstance(d, dict) and "cmd" in d:
            best = d
    return best


def _post_chat_with_heal(payload, url, model, timeout):
    """POST /api/chat, 带自动自愈。

    Ollama 0.40.2 有已知问题: 模型运行一段后"重复token保护"偶发误触发,
    之后所有请求返回空内容或 HTTP 500。检测到后自动"卸载+重载"模型再重试一次。
    """
    note = ""
    for attempt in (1, 2):
        r = requests.post(f"{url}/api/chat", json=payload, timeout=timeout)
        if r.status_code == 200:
            content = r.json().get("message", {}).get("content", "") or ""
            if content.strip():
                return content
            note = "模型返回空内容"
        else:
            try:
                note = f"HTTP {r.status_code}: {r.json().get('error', '')}"
            except Exception:
                note = f"HTTP {r.status_code}"
        if attempt == 1:
            print(f"   (模型异常: {note} — 自动重载模型后重试 ...)")
            try:
                requests.post(f"{url}/api/generate",
                              json={"model": model, "keep_alive": 0, "prompt": ""},
                              timeout=15)
            except Exception:
                pass
            time.sleep(1.5)
    return ""


def ask_llm(user_text, model, url, timeout=180):
    messages = [{"role": "system", "content": build_system_prompt()}]
    for u, a in FEW_SHOT:
        messages.append({"role": "user", "content": u})
        messages.append({"role": "assistant", "content": a})
    messages.append({"role": "user", "content": user_text})
    payload = {
        "model": model,
        "stream": False,
        "format": "json",
        "messages": messages,
        "options": {"temperature": 0.1, "num_predict": NUM_PREDICT},
    }
    content = _post_chat_with_heal(payload, url, model, timeout)
    return content, extract_json(content)


def check_ollama(url, model):
    try:
        r = requests.get(f"{url}/api/tags", timeout=5)
        names = [m.get("name", "") for m in r.json().get("models", [])]
        hit = any(n == model or n.split(":")[0] == model.split(":")[0] for n in names)
        if hit:
            print(f"[OK] Ollama 在线, 模型 {model} 可用")
            return True
        print(f"[警告] Ollama 在线, 但没有模型 {model}; 现有: {', '.join(names)}")
        return False
    except Exception as e:
        print(f"[警告] Ollama 未运行或不可达({e})")
        return False


# ==================== 校验层 ====================
class CmdError(Exception):
    pass


def normalize_target(text):
    """'A3-11' / 'a3 11' / 'A311' -> 'A3-11'; 非法返回 None"""
    import re
    s = str(text).strip().upper().replace(" ", "").replace("－", "-")
    m = re.match(r'^(A[1-4]|B[1-4])-?(\d{1,2})$', s)
    if not m:
        return None
    row, idx = m.group(1), int(m.group(2))
    if not 1 <= idx <= 24:
        return None
    return f"{row}-{idx:02d}"


def validate_command(cmd):
    """返回 (cmd, targetId) 或抛 CmdError"""
    name = str(cmd.get("cmd", "")).strip().lower()
    if name == "none":
        raise CmdError(f"非动作: {cmd.get('reason', '')}")
    if name not in ALLOWED_CMDS:
        raise CmdError(f"未知指令 {name!r}, 只允许 {sorted(ALLOWED_CMDS)}")
    if name == "estop":
        return "estop", None
    target = normalize_target(cmd.get("targetId", ""))
    if target is None:
        raise CmdError(f"编号不合法: {cmd.get('targetId')!r}; "
                       f"应为 A1-01..A4-24 / B1-01..B4-24")
    return name, target


def describe(cmd, target):
    text = {"trim_pick": "取花(修剪)", "trim_place": "放回", "seed": "播种", "estop": "急停"}
    if cmd == "estop":
        return "急停"
    return f"{text.get(cmd, cmd)} {target}"


def wire_json(cmd, target):
    """生成将发送的 MQTT 报文字本"""
    wire = {"cmd": cmd}
    if target:
        wire["targetId"] = target
    return json.dumps(wire, ensure_ascii=False, separators=(",", ":"))


# ==================== 各模式 ====================
def make_mqtt(args):
    return TlMqtt(args.host, args.port, args.username, args.password,
                  args.cmd_topic, args.fb_topic)


def run_check(args):
    """只读自检: 连接 broker + 订阅反馈话题"""
    print(f"[自检] 连接 broker {args.host}:{args.port} (只读, 不会发任何命令) ...")
    mqtt = make_mqtt(args)
    try:
        mqtt.connect()
    except ConnectionError as e:
        print(f"[失败] {e}")
        print("       检查: 网络能否访问该地址; 用户名密码是否正确")
        return
    print(f"[OK] broker 连接成功, 已订阅: {args.fb_topic}")
    print(f"[信息] 命令将发布到: {args.cmd_topic}")
    print("[信息] 监听反馈 8 秒 (若 Nano 端程序正在运行并执行任务, 这里会显示) ...")
    t0 = time.time()
    while time.time() - t0 < 8:
        time.sleep(0.2)
    if mqtt._last_seen:
        print(f"[流量] 最近一条反馈: {mqtt._last_seen}")
    else:
        print("[流量] 8 秒内无反馈 (正常: 没任务就没反馈; 说明不了程序是否在运行)")
    print("[自检完成] broker 链路可用")
    mqtt.close()


def llm_only(args):
    check_ollama(args.ollama, args.model)
    print(f"[输入] {args.llm_only}")
    raw, cmd = ask_llm(args.llm_only, args.model, args.ollama)
    if cmd is None:
        print("(解析失败, 自动重试一次 ...)")
        raw, cmd = ask_llm(args.llm_only, args.model, args.ollama)
    print(f"[模型原始输出] {raw.strip()[:500]}")
    if cmd is None:
        print("[结果] 解析失败: 模型没有输出合法 JSON")
        return
    print(f"[解析结果] {json.dumps(cmd, ensure_ascii=False)}")
    try:
        name, target = validate_command(cmd)
    except CmdError as e:
        print(f"[校验层] 拒绝: {e}")
        return
    print(f"[将发送] {wire_json(name, target)}   # {describe(name, target)}")


def interactive(args):
    check_ollama(args.ollama, args.model)
    mqtt = make_mqtt(args)
    online = False
    try:
        mqtt.connect()
        online = True
        print(f"[OK] broker 已连接 {args.host}:{args.port}")
    except ConnectionError as e:
        print(f"[警告] broker 连接失败({e}); 仍可查看指令翻译, 但不会发送")
    if args.execute and not online:
        print("[错误] --execute 需要 broker 在线")
        return

    # 语音模式: 加载本地语音识别(提示词偏向园艺指令词汇)
    stt = None
    if args.voice:
        try:
            import voice_stt
            stt = voice_stt.SttEngine(
                model_size=args.stt_model, device=args.stt_device,
                initial_prompt="机械臂指令。修剪A3第11号，给B1第5号播种，"
                               "把A2-01的花放回去，摘A4-24，急停，停下。")
        except ImportError as e:
            print(f"[错误] 语音依赖缺失: {e}")
            print("       先运行: pip install faster-whisper sounddevice")
            return

    mode = "真实发送" if args.execute else "试运行(dry-run)"
    print("=" * 62)
    print(f" 大模型控制天链机械臂 | 模式: {mode} | 模型: {args.model}"
          + (" | 🎤语音模式" if stt else ""))
    print(f" 发布: {args.cmd_topic}   订阅: {args.fb_topic}")
    if args.execute:
        print(" ⚠ 请确认 Nano 上 run_seed_mqtt / run_fork_mqtt 已运行; 手放急停旁")
    print(" 直接输入中文, 如: 修剪A3第11号 / 给B2第5号播种 / 把A3-11放回去 / 急停")
    if stt:
        print(" 🎤 语音模式: 空回车=开始说话(停顿自动结束); 打字+回车=文本输入")
    print(" 输入 exit/q/quit 或 退出 结束程序")
    print("=" * 62)

    try:
        while True:
            try:
                hint = "(回车=说话)" if stt else ""
                text = input(f"\n🗣️  > {hint}").strip()
            except EOFError:
                break
            if not text and stt:
                # 语音输入: 录音 -> 识别 -> 走同一条大模型管线
                audio, err = voice_stt.record_utterance(max_sec=args.voice_max_sec)
                if audio is None:
                    print(f"🎤 {err}")
                    continue
                print("🎤 识别中 ...")
                try:
                    text = stt.transcribe(audio)
                except Exception as e:
                    print(f"🎤 识别失败: {e}")
                    continue
                if not text:
                    print("🎤 没识别出内容, 再说一次")
                    continue
                print(f"🎤 听到: {text}")
            if not text:
                continue
            if text.lower() in EXIT_WORDS or text.strip() in ("退出", "结束"):
                break

            print("🧠 大模型思考中 ...")
            t0 = time.time()
            try:
                raw, cmd = ask_llm(text, args.model, args.ollama)
                if cmd is None:
                    print("   (解析失败, 自动重试一次 ...)")
                    raw, cmd = ask_llm(text, args.model, args.ollama)
            except requests.exceptions.ConnectionError:
                print("[错误] Ollama 连不上, 请确认它正在运行")
                continue
            except requests.exceptions.ReadTimeout:
                print("[错误] 模型响应超时, 再试一次")
                continue
            except requests.exceptions.RequestException as e:
                print(f"[错误] Ollama 请求异常: {e}")
                continue
            dt = time.time() - t0
            if raw.strip():
                print(f"   (模型输出 {dt:.1f}s): {' '.join(raw.split())[:200]}")
            if cmd is None:
                print("   [解析失败] 模型没有给出合法 JSON, 换个说法再试")
                continue
            print(f"   指令: {json.dumps(cmd, ensure_ascii=False)}")

            try:
                name, target = validate_command(cmd)
            except CmdError as e:
                print(f"   [校验层] 拒绝: {e}")
                continue
            print(f"   → {describe(name, target)}  ({wire_json(name, target)})")

            if not args.execute:
                print("   [试运行] 未发送 (加 --execute 真实发送)")
                continue

            if not args.yes:
                ans = input("   确认发送? [y/N] ").strip().lower()
                if ans != "y":
                    print("   已取消")
                    continue

            ok, body = mqtt.publish_cmd(name, target)
            if not ok:
                print("   [错误] MQTT 发布失败(连接断了?)")
                continue
            print(f"   已发送: {body}")

            if name == "estop":
                print("   ✓ 急停已发送(锁存)。恢复需在 Nano 端重启对应程序")
                continue

            result = mqtt.wait_result(name, target,
                                      ack_timeout=args.ack_timeout,
                                      total_timeout=args.timeout)
            if result["success"]:
                print(f"   ✅ {describe(name, target)} 完成 (retCode=0)")
            else:
                code = result["code"]
                print(f"   ❌ {describe(name, target)} 未成功: {result['text']}"
                      + (f" (retCode={code})" if code is not None else ""))
    except KeyboardInterrupt:
        print("\n[中断] Ctrl+C")
    finally:
        mqtt.close()
        print("再见 👋")


def main():
    ap = argparse.ArgumentParser(
        description="大模型(Ollama)控制天链机械臂演示 — MQTT 版",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument("--host", default=MQTT_HOST)
    ap.add_argument("--port", type=int, default=MQTT_PORT)
    ap.add_argument("--username", default=MQTT_USER)
    ap.add_argument("--password", default=MQTT_PASSWORD)
    ap.add_argument("--cmd-topic", default=CMD_TOPIC)
    ap.add_argument("--fb-topic", default=FB_TOPIC)
    ap.add_argument("--model", default=DEFAULT_MODEL)
    ap.add_argument("--ollama", default=OLLAMA_URL)
    ap.add_argument("--ack-timeout", type=float, default=20.0, help="等首条反馈秒数")
    ap.add_argument("--timeout", type=float, default=300.0, help="等最终结果秒数")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--execute", action="store_true", help="真实发送(默认试运行)")
    mode.add_argument("--check", action="store_true", help="broker 只读自检")
    mode.add_argument("--llm-only", metavar="文字", help="只测 LLM→JSON")
    ap.add_argument("--yes", action="store_true", help="跳过逐条确认")
    ap.add_argument("--voice", action="store_true", help="🎤 语音模式: 空回车说话, 停顿自动结束")
    ap.add_argument("--stt-model", default="small",
                    help="语音识别模型 tiny/base/small/medium (默认 small)")
    ap.add_argument("--stt-device", default="cpu", choices=["cpu", "cuda"],
                    help="语音识别设备 (默认 cpu)")
    ap.add_argument("--voice-max-sec", type=float, default=8.0, help="单次录音最长秒数")
    ap.add_argument("--init-voice", action="store_true", help="列出麦克风设备后退出")
    args = ap.parse_args()

    if args.init_voice:
        try:
            import voice_stt
            voice_stt.list_mics()
        except ImportError as e:
            print(f"[错误] 语音依赖缺失: {e}; 先运行: pip install faster-whisper sounddevice")
        return

    if args.check:
        run_check(args)
    elif args.llm_only:
        llm_only(args)
    else:
        interactive(args)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
TCB605 VLA 推理服务（SmolVLA 零样本演示版）
============================================
在《TCB605_找寻目标_VLA部署方案》原版 tcb_find_infer_server.py 基础上扩展，支持三种模式：

  1) --skeleton              骨架模式：不加载模型，返回零动作（调链路用）
  2) --policy demo           保底演示模式：输出缓慢小圈运动，保证"机械臂一定会动"
  3) --policy smolvla        零样本 SmolVLA：加载 lerobot/smolvla_base（无需微调/无需采集数据）
  另：--model-path <ckpt>    仍支持加载你们自己微调好的 LeRobot checkpoint（原逻辑保留）

运行示例：
  # 零样本 SmolVLA（首次联网自动下载模型；国内用 hf-mirror）
  export HF_ENDPOINT=https://hf-mirror.com
  python3 tcb_find_infer_server.py --policy smolvla --port 18080

  # 只做自检（加载模型 + 跑一条假数据，不动机械臂）
  python3 tcb_find_infer_server.py --policy smolvla --check

  # 保底演示模式（演示当天模型翻车时切这个）
  python3 tcb_find_infer_server.py --policy demo

关键参数：
  --action-scale 300.0   零样本输出 → 毫米的放大系数（动作不明显就调大）
  --smooth 0.3           动作指数平滑（0 关闭；越大越柔和）

端点（与你们原版完全一致）：
  GET  /health   -> {"status":"ok","policy":...,"device":...}
  POST /predict  <- {"image":"<base64 jpg>","state":[12 floats],"task":"..."}
                 -> {"action":[dx,dy,dz],"inference_time":s,"policy":...}
"""

import os
import sys
import json
import math
import time
import base64
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np


# ============================================================
# 零样本 SmolVLA 封装（多重兼容：不同 LeRobot 版本字段可能不同）
# ============================================================
class SmolVLAZeroShot:
    def __init__(self, model_id="lerobot/smolvla_base", device=None,
                 action_scale=300.0, smooth=0.3):
        import torch
        self.torch = torch
        self.action_scale = float(action_scale)
        self.smooth = float(smooth)
        self._prev = None

        if device is None:
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(device)
        if self.device.type == "cpu":
            print("[警告] 运行在 CPU! 检查 PyTorch GPU 是否正常"
                  "（应装 jetson-ai-lab 的 GPU wheel）")

        print(f"[加载] 零样本模型 {model_id} …（首次会自动下载 ~1-2GB）")
        self.policy = self._load_policy(model_id)
        self.policy.eval()
        try:
            self.policy.to(self.device)
        except Exception as e:
            print(f"[提示] policy.to() 跳过: {e}")

        # 找它期望的图像 key / state 维度（不同版本字段不同，做兼容）
        cfg = getattr(self.policy, "config", None)
        img_feats = getattr(cfg, "image_features", None) if cfg else None
        if img_feats:
            self.image_keys = list(img_feats.keys())
        else:
            self.image_keys = [
                "observation.images.camera1",
                "observation.images.camera2",
                "observation.images.camera3",
            ]
        self.state_dim = 6
        try:
            self.state_dim = int(cfg.input_features["observation.state"].shape[0])
        except Exception:
            pass

        try:
            self.policy.reset()
        except Exception:
            pass
        print(f"[SmolVLA] 就绪 device={self.device} "
              f"图像keys={self.image_keys} state_dim={self.state_dim} "
              f"action_scale={self.action_scale} smooth={self.smooth}")

    @staticmethod
    def _load_policy(model_id):
        """兼容新旧 LeRobot 的 import 路径"""
        try:
            from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
        except ImportError:
            try:
                from lerobot.common.policies.smolvla.modeling_smolvla import (
                    SmolVLAPolicy)
            except ImportError as e:
                raise RuntimeError(
                    "找不到 SmolVLA 模块：LeRobot 未安装或版本不兼容。\n"
                    "请先执行 install_nano_smolvla.sh（注意用 --no-deps 防覆盖 GPU torch）。\n"
                    f"原始错误: {e}")
        return SmolVLAPolicy.from_pretrained(model_id)

    def predict(self, image_bgr, state12, task):
        """image_bgr(np 640x480 BGR) + state12(12维) + task -> [dx,dy,dz] mm"""
        import cv2
        torch = self.torch

        # ---- 图像：224x224，第一路给真实画面，其余给黑图（base 模型期望最多3路）----
        img = cv2.resize(image_bgr, (224, 224))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        t_img = torch.from_numpy(img).permute(2, 0, 1).float() / 255.0  # [3,224,224]

        batch = {}
        for i, key in enumerate(self.image_keys):
            src = t_img if i == 0 else torch.zeros_like(t_img)
            batch[key] = src.unsqueeze(0).to(self.device)

        # ---- state：用关节角（度→弧度）喂给它（SO100 的 state 是 6 维关节角）----
        joints_deg = list(state12[6:12]) if len(state12) >= 12 else [0.0] * 6
        s6 = [math.radians(float(v)) for v in joints_deg]
        if self.state_dim != len(s6):
            s6 = (s6 + [0.0] * self.state_dim)[:self.state_dim]
        batch["observation.state"] = torch.tensor(
            [s6], dtype=torch.float32, device=self.device)

        # ---- 语言指令（多数版本接受 list[str]）----
        batch["task"] = [str(task)]

        # ---- 推理（兼容 select_action / predict_action_chunk 两种调用）----
        out = None
        err1 = err2 = None
        with torch.no_grad():
            try:
                out = self.policy.select_action(batch)
            except Exception as e1:
                err1 = e1
                try:
                    out = self.policy.predict_action_chunk(batch)
                except Exception as e2:
                    err2 = e2
        if out is None:
            print(f"[SmolVLA] 推理调用失败:\n  select_action: {err1}\n  chunk: {err2}")
            return [0.0, 0.0, 0.0]

        # ---- 取动作（拍平任何前导 batch/chunk 维度）----
        arr = out.detach().float().cpu().numpy()
        while arr.ndim > 1:
            arr = arr[0]
        a = arr.reshape(-1)

        # ---- 映射成 [dx,dy,dz] (mm) ----
        # SO100 类动作=关节目标位置(rad)；增量 = 目标-当前，再乘放大系数
        base = np.asarray(s6[:3], dtype=np.float32)
        if a.size >= 6 and base.size == 3:
            d = a[:3] - base
        else:
            d = a[:3]
        act = d.astype(np.float32) * self.action_scale
        act = np.clip(act, -60.0, 60.0)

        # ---- 指数平滑（让运动柔和）----
        if self.smooth > 0:
            if self._prev is None:
                self._prev = act
            else:
                self._prev = self.smooth * act + (1.0 - self.smooth) * self._prev
            act = self._prev

        return [float(v) for v in act[:3]]


# ============================================================
# 推理服务
# ============================================================
class InferServer:
    """推理引擎: skeleton / demo / smolvla / 微调checkpoint 四种来源"""

    def __init__(self, policy="skeleton", model_path="", stats_path="",
                 action_scale=300.0, smooth=0.3):
        self.policy_name = policy
        self.smolvla = None
        self.finetuned = None
        self.norm_stats = None
        self.device = None
        self._demo_t0 = time.time()

        self._load_stats(stats_path)

        if policy == "smolvla":
            self.smolvla = SmolVLAZeroShot(
                action_scale=action_scale, smooth=smooth)
            self.device = self.smolvla.device
            self.policy_name = "smolvla(零样本)"
        elif policy == "finetuned":
            self._load_finetuned(model_path)
        elif policy == "demo":
            print("[模式] 保底演示模式（缓慢小圈运动，机械臂一定会动）")
        else:
            print("[模式] 骨架模式（返回零动作，用于调通链路）")

    # ---------- 原版微调模型加载（保留） ----------
    def _load_stats(self, stats_path):
        if not stats_path:
            stats_path = os.path.expanduser(
                "~/tcb_find_data/lerobot_dataset/meta/stats.json")
        if os.path.exists(stats_path):
            with open(stats_path, "r", encoding="utf-8") as f:
                self.norm_stats = json.load(f)
            print(f"[OK] 归一化参数: {stats_path}")
        else:
            self.norm_stats = {
                "observation.state": {"mean": [0.0] * 12, "std": [1.0] * 12},
                "action": {"mean": [0.0, 0.0, 0.0],
                           "std": [10.0, 10.0, 10.0]},
            }

    def _load_finetuned(self, model_path):
        try:
            import torch
            from lerobot.policies.smolvla.modeling_smolvla import (
                SmolVLAPolicy)
            self.finetuned = SmolVLAPolicy.from_pretrained(model_path)
            self.finetuned.eval()
            self.device = torch.device(
                "cuda" if torch.cuda.is_available() else "cpu")
            self.finetuned.to(self.device)
            self.policy_name = "finetuned"
            print(f"[OK] 微调模型加载成功 device={self.device}")
        except Exception as e:
            print(f"[错误] 微调模型加载失败: {e} -> 回退骨架模式")
            self.policy_name = "skeleton"
            self.finetuned = None

    # ---------- 主入口 ----------
    def predict(self, image_b64, state, task):
        if self.policy_name.startswith("smolvla"):
            try:
                import cv2
                img_bytes = base64.b64decode(image_b64)
                img = cv2.imdecode(np.frombuffer(img_bytes, np.uint8),
                                   cv2.IMREAD_COLOR)
                if img is None:
                    return [0.0, 0.0, 0.0]
                return self.smolvla.predict(img, state, task)
            except Exception as e:
                print(f"[零样本推理错误] {e}")
                return [0.0, 0.0, 0.0]

        if self.policy_name == "demo":
            t = time.time() - self._demo_t0
            r = 8.0  # mm 半径
            return [r * math.sin(2 * math.pi * t / 15.0),
                    r * math.cos(2 * math.pi * t / 15.0),
                    0.0]

        if self.policy_name == "finetuned" and self.finetuned is not None:
            return self._predict_finetuned(image_b64, state, task)

        return [0.0, 0.0, 0.0]  # skeleton

    def _predict_finetuned(self, image_b64, state, task):
        """你们原版的 12维state -> 3维动作 推理逻辑（保留）"""
        try:
            import torch
            import cv2
            img_bytes = base64.b64decode(image_b64)
            img = cv2.imdecode(np.frombuffer(img_bytes, np.uint8),
                               cv2.IMREAD_COLOR)
            if img is None:
                return [0.0, 0.0, 0.0]
            img = cv2.resize(img, (224, 224))
            img_tensor = (torch.from_numpy(img[:, :, ::-1].copy())
                          .permute(2, 0, 1).float() / 255.0)
            img_tensor = img_tensor.unsqueeze(0).to(self.device)

            s_mean = np.array(self.norm_stats["observation.state"]["mean"],
                              dtype=np.float32)
            s_std = np.array(self.norm_stats["observation.state"]["std"],
                             dtype=np.float32)
            state_norm = (np.array(state, dtype=np.float32) - s_mean) / s_std
            state_tensor = torch.from_numpy(state_norm).unsqueeze(0).to(
                self.device)

            observation = {
                "observation.images.wrist": img_tensor,
                "observation.state": state_tensor,
                "task": task,
            }
            with torch.no_grad():
                action = self.finetuned.select_action(observation)
            action_np = action.cpu().numpy()[0]
            a_mean = np.array(self.norm_stats["action"]["mean"],
                              dtype=np.float32)
            a_std = np.array(self.norm_stats["action"]["std"],
                             dtype=np.float32)
            action_real = action_np * a_std + a_mean
            return [float(v) for v in action_real.tolist()]
        except Exception as e:
            print(f"[微调推理错误] {e}")
            return [0.0, 0.0, 0.0]


class Handler(BaseHTTPRequestHandler):
    server = None

    def _send_json(self, code, data):
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def do_GET(self):
        if self.path == "/health":
            self._send_json(200, {
                "status": "ok",
                "policy": self.server.infer.policy_name,
                "device": str(self.server.infer.device)
                if self.server.infer.device else "n/a",
            })
        else:
            self._send_json(404, {"error": "not found"})

    def do_POST(self):
        if self.path == "/predict":
            try:
                length = int(self.headers["Content-Length"])
                req = json.loads(self.rfile.read(length))
                image_b64 = req.get("image", "")
                state = req.get("state", [0.0] * 12)
                task = req.get("task", "pick up the object")
                t0 = time.time()
                action = self.server.infer.predict(image_b64, state, task)
                dt = time.time() - t0
                self._send_json(200, {
                    "action": action,
                    "inference_time": round(dt, 3),
                    "policy": self.server.infer.policy_name,
                })
            except Exception as e:
                self._send_json(500, {"error": str(e)})
        else:
            self._send_json(404, {"error": "not found"})

    def log_message(self, *args):
        pass


def main():
    parser = argparse.ArgumentParser(
        description="TCB605 VLA 推理服务（SmolVLA 零样本演示版）")
    parser.add_argument("--policy", default="skeleton",
                        choices=["skeleton", "demo", "smolvla", "finetuned"],
                        help="skeleton=零动作 | demo=保底演示 | "
                             "smolvla=零样本 | finetuned=--model-path 微调模型")
    parser.add_argument("--model-path", default="",
                        help="你们微调好的 checkpoint 路径（policy=finetuned 时用）")
    parser.add_argument("--model-id", default="lerobot/smolvla_base",
                        help="零样本模型 HF 名称")
    parser.add_argument("--stats-path", default="",
                        help="归一化参数 stats.json（微调模式用）")
    parser.add_argument("--action-scale", type=float, default=300.0,
                        help="零样本输出→毫米放大系数（动作太小就调大）")
    parser.add_argument("--smooth", type=float, default=0.3,
                        help="动作指数平滑系数（0=关闭）")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18080)
    parser.add_argument("--check", action="store_true",
                        help="自检：加载后跑一条假数据并打印，不启动 HTTP 服务")
    args = parser.parse_args()

    policy = args.policy
    if policy == "skeleton" and args.model_path:
        policy = "finetuned"

    infer = InferServer(
        policy=policy,
        model_path=args.model_path,
        stats_path=args.stats_path,
        action_scale=args.action_scale,
        smooth=args.smooth,
    )

    if args.check:
        fake_img = np.zeros((480, 640, 3), np.uint8)
        fake_state = [400.0, 0.0, 300.0, 0.0, 0.0, 0.0] + [0.0] * 6
        action = infer.predict(
            base64.b64encode(
                __import__("cv2").imencode(
                    ".jpg", fake_img)[1]).decode(),
            fake_state,
            "pick up the red cube")
        print(f"\n[自检] 模式={infer.policy_name} 假数据动作输出: {action}")
        print("[自检] 上步只验证链路/模型能出数；真实动作以实际画面为准。")
        return

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.infer = infer
    print(f"\n推理服务启动: http://{args.host}:{args.port}")
    print(f"模式: {infer.policy_name}")
    print("端点: GET /health | POST /predict")
    print("按 Ctrl+C 退出\n")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n关闭服务")
        server.shutdown()


if __name__ == "__main__":
    main()

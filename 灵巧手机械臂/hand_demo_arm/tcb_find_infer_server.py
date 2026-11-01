#!/usr/bin/env python3
"""
TCB605 找寻目标 VLA 推理服务
============================
HTTP 推理服务，加载 SmolVLA/ACT policy，接收图像+状态，返回 [dx,dy,dz]。

⚠ 关键: 用系统 Python 3.10 + jetson-ai-lab PyTorch 运行，不要用 conda。
  LeRobot 用 pip3 install --no-deps lerobot 安装，避免覆盖 GPU PyTorch。

运行:
  # 真实推理
  python3 tcb_find_infer_server.py \
    --model-path ~/tcb_find_models/checkpoints/last \
    --stats-path ~/tcb_find_data/lerobot_dataset/meta/stats.json

  # 骨架模式 (不加载模型，返回零动作，用于调通链路)
  python3 tcb_find_infer_server.py --skeleton

端点:
  GET  /health   -> {"status":"ok", "skeleton": bool}
  POST /predict  <- {"image": "<base64 jpg>", "state": [12 floats], "task": "..."}
                 -> {"action": [dx,dy,dz], "inference_time": s, "skeleton": bool}
"""

import os
import sys
import json
import time
import base64
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np


class InferServer:
    """推理引擎: 支持 SmolVLA / ACT policy，骨架模式返回零动作"""

    def __init__(self, model_path='', stats_path='', skeleton=False):
        self.skeleton = skeleton
        self.policy = None
        self.norm_stats = None
        self.device = None

        # 加载归一化参数 (推理服务反归一化必需)
        self._load_stats(stats_path)

        if not skeleton and model_path:
            self._load_model(model_path)
        else:
            print('[模式] 骨架模式 (返回零动作，用于调通链路)')

    def _load_stats(self, stats_path):
        """加载归一化参数"""
        if not stats_path:
            stats_path = os.path.expanduser(
                '~/tcb_find_data/lerobot_dataset/meta/stats.json')
        if os.path.exists(stats_path):
            with open(stats_path, 'r', encoding='utf-8') as f:
                self.norm_stats = json.load(f)
            print(f'[OK] 归一化参数: {stats_path}')
        else:
            print('[警告] 未找到归一化参数，用默认值')
            self.norm_stats = {
                'observation.state': {
                    'mean': [0.0] * 12, 'std': [1.0] * 12},
                'action': {
                    'mean': [0.0, 0.0, 0.0], 'std': [10.0, 10.0, 10.0]},
            }

    def _load_model(self, model_path):
        """加载 SmolVLA / ACT policy"""
        try:
            import torch
            print(f'[加载模型] {model_path}')

            # 优先尝试 SmolVLA，失败则尝试 ACT
            try:
                from lerobot.common.policies.smolvla.modeling_smolvla import (
                    SmolVLAPolicy)
                self.policy = SmolVLAPolicy.from_pretrained(model_path)
                print('[OK] SmolVLA 加载成功')
            except Exception:
                from lerobot.common.policies.act.modeling_act import (
                    ACTPolicy)
                self.policy = ACTPolicy.from_pretrained(model_path)
                print('[OK] ACT 加载成功')

            self.policy.eval()
            self.device = torch.device(
                'cuda' if torch.cuda.is_available() else 'cpu')
            self.policy.to(self.device)
            print(f'[OK] 模型就绪, device={self.device}')
            if self.device.type == 'cpu':
                print('[警告] 运行在 CPU! 检查 PyTorch GPU 是否正常')
        except Exception as e:
            print(f'[错误] 模型加载失败: {e}')
            print('[回退] 切换到骨架模式')
            self.skeleton = True
            self.policy = None

    def predict(self, image_b64, state, task):
        """推理: image_b64 + state(12维) + task -> action [dx,dy,dz] (mm)"""
        if self.skeleton or self.policy is None:
            return [0.0, 0.0, 0.0]

        try:
            import torch
            import cv2

            # 解码图像 → 224x224 RGB tensor
            img_bytes = base64.b64decode(image_b64)
            img_arr = np.frombuffer(img_bytes, np.uint8)
            img = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)
            if img is None:
                return [0.0, 0.0, 0.0]
            img = cv2.resize(img, (224, 224))
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img_tensor = torch.from_numpy(img).permute(2, 0, 1).float() / 255.0
            img_tensor = img_tensor.unsqueeze(0).to(self.device)

            # 归一化 state
            s_mean = np.array(
                self.norm_stats['observation.state']['mean'], dtype=np.float32)
            s_std = np.array(
                self.norm_stats['observation.state']['std'], dtype=np.float32)
            state_norm = (np.array(state, dtype=np.float32) - s_mean) / s_std
            state_tensor = torch.from_numpy(state_norm).unsqueeze(0).to(
                self.device)

            # 构造 observation dict (字段名与训练时一致)
            observation = {
                'observation.images.wrist': img_tensor,
                'observation.state': state_tensor,
                'task': task,
            }

            with torch.no_grad():
                action = self.policy.select_action(observation)

            action_np = action.cpu().numpy()[0]  # 归一化的 [dx,dy,dz]

            # 反归一化 → 真实 mm
            a_mean = np.array(
                self.norm_stats['action']['mean'], dtype=np.float32)
            a_std = np.array(
                self.norm_stats['action']['std'], dtype=np.float32)
            action_real = action_np * a_std + a_mean

            return [float(v) for v in action_real.tolist()]
        except Exception as e:
            print(f'[推理错误] {e}')
            return [0.0, 0.0, 0.0]


class Handler(BaseHTTPRequestHandler):
    server = None

    def _send_json(self, code, data):
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def do_GET(self):
        if self.path == '/health':
            self._send_json(200, {
                'status': 'ok',
                'skeleton': self.server.infer.skeleton,
                'device': str(self.server.infer.device) if self.server.infer.device else 'skeleton',
            })
        else:
            self._send_json(404, {'error': 'not found'})

    def do_POST(self):
        if self.path == '/predict':
            try:
                length = int(self.headers['Content-Length'])
                body = self.rfile.read(length)
                req = json.loads(body)
                image_b64 = req.get('image', '')
                state = req.get('state', [0.0] * 12)
                task = req.get('task', '接近红色方块')

                t0 = time.time()
                action = self.server.infer.predict(image_b64, state, task)
                dt = time.time() - t0

                self._send_json(200, {
                    'action': action,
                    'inference_time': round(dt, 3),
                    'skeleton': self.server.infer.skeleton,
                })
            except Exception as e:
                self._send_json(500, {'error': str(e)})
        else:
            self._send_json(404, {'error': 'not found'})

    def log_message(self, *args):
        pass  # 静默 HTTP 日志


def main():
    parser = argparse.ArgumentParser(description='TCB605 找寻目标 VLA 推理服务')
    parser.add_argument('--model-path', default='',
                        help='模型 checkpoint 路径')
    parser.add_argument('--stats-path', default='',
                        help='归一化参数 stats.json 路径')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=18080)
    parser.add_argument('--skeleton', action='store_true',
                        help='骨架模式，不加载模型，返回零动作')
    args = parser.parse_args()

    skeleton = args.skeleton or not args.model_path
    infer = InferServer(
        model_path=args.model_path,
        stats_path=args.stats_path,
        skeleton=skeleton)

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.infer = infer
    print(f'\n推理服务启动: http://{args.host}:{args.port}')
    print(f'模式: {"骨架(零动作)" if skeleton else "真实推理"}')
    print(f'端点: GET /health | POST /predict')
    print(f'按 Ctrl+C 退出\n')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\n关闭服务')
        server.shutdown()


if __name__ == '__main__':
    main()

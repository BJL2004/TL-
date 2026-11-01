#!/usr/bin/env bash
# ============================================================
# Nano 端一键安装：SmolVLA 零样本推理环境
#   - 关键纪律：绝不覆盖 jetson-ai-lab 的 GPU PyTorch（用 --no-deps 装 lerobot）
#   - 国内网络：pip 走清华镜像，模型下载走 hf-mirror
# 用法（在 Nano 上）：bash install_nano_smolvla.sh
# ============================================================
set -eo pipefail

echo "=== [1/4] 检查 GPU PyTorch（必须输出 True 才能继续）==="
python3 - <<'PY'
import torch
print("torch:", torch.__version__, "| cuda available:", torch.cuda.is_available())
assert torch.cuda.is_available(), "PyTorch 不是 GPU 版！先修好 torch（jetson-ai-lab wheel）再跑本脚本"
PY

echo
echo "=== [2/4] 安装 LeRobot（--no-deps，防止把 GPU torch 换成 CPU 版）==="
PIP_MIRROR="https://pypi.tuna.tsinghua.edu.cn/simple"
pip3 install -i "$PIP_MIRROR" --no-deps lerobot
echo "--- 装 lerobot 的轻量依赖（不会碰 torch）---"
pip3 install -i "$PIP_MIRROR" \
  transformers safetensors huggingface_hub einops accelerate \
  num2words draccus imageio pillow requests

echo
echo "=== [3/4] 复查 torch 没被覆盖 ==="
python3 - <<'PY'
import torch
assert torch.cuda.is_available(), "!!! Torch 被覆盖成 CPU 版了：重装 jetson-ai-lab 的 GPU wheel 后再跑"
print("OK, torch 还是 GPU 版")
PY

echo
echo "=== [4/4] 下载 SmolVLA base 模型（约 1~2GB，走 hf-mirror）==="
export HF_ENDPOINT="${HF_ENDPOINT:-https://hf-mirror.com}"
python3 - <<'PY'
from huggingface_hub import snapshot_download
p = snapshot_download("lerobot/smolvla_base")
print("模型已下载到:", p)
PY

echo
echo "=== 完成！下一步自检（不动机械臂）==="
echo "cd ~/Desktop/vla_demo"
echo "python3 tcb_find_infer_server.py --policy smolvla --check"

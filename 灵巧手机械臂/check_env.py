#!/usr/bin/env python3
"""
Jetson Orin Nano 16GB — 环境验证脚本
对照《Jetson16GB部署文档》第 2 节逐项检查
"""

import subprocess, sys, os, importlib

PASS = "✅"
FAIL = "❌"
WARN = "⚠️"
results = []

def check(name, ok, detail=""):
    mark = PASS if ok else FAIL
    msg = f"{mark} {name}"
    if detail:
        msg += f"  ({detail})"
    print(msg)
    results.append((name, ok, detail))

def run(cmd):
    try:
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=15)
        return r.returncode, (r.stdout + r.stderr).strip()
    except Exception as e:
        return -1, str(e)

print("=" * 60)
print("Jetson 环境验证  — 对照部署文档 2.1~2.7 节")
print("=" * 60)

# ── 2.1 JetPack ──────────────────────────────────────────
code, out = run("cat /etc/nv_tegra_release 2>/dev/null")
if code == 0 and "R36" in out:
    rev_line = [l for l in out.split("\n") if "REVISION" in l]
    check("2.1 JetPack 6.2", True, rev_line[0].strip() if rev_line else out.split("\n")[0].strip())
else:
    check("2.1 JetPack 6.2", False, out[:80] if out else "无法读取 nv_tegra_release")

# ── 2.2 CUDA ─────────────────────────────────────────────
code, out = run("nvcc --version 2>/dev/null")
if code == 0:
    for line in out.split("\n"):
        if "release" in line:
            check("2.2 CUDA (nvcc)", True, line.strip())
            break
else:
    check("2.2 CUDA (nvcc)", False, "PATH 问题? 试 source ~/.bashrc")

# ── 2.3 ROS2 Humble ──────────────────────────────────────
code, out = run("bash -c 'source /opt/ros/humble/setup.bash && echo $ROS_DISTRO'")
check("2.3 ROS2 Humble", code == 0 and "humble" in out, out.strip())

# colcon
code, out = run("bash -lc 'source /opt/ros/humble/setup.bash; colcon version-check' 2>&1")
check("2.3 colcon", code == 0, out.strip()[:80] if out.strip() else "not found")

# ── 2.4 PyTorch + torchvision (GPU) ──────────────────────
try:
    import torch
    cuda_ok = torch.cuda.is_available()
    check("2.4 PyTorch", True, f"v{torch.__version__}  CUDA={cuda_ok}")
    if cuda_ok:
        check("2.4 CUDA GPU 可用", True, f"设备: {torch.cuda.get_device_name(0)}")
    else:
        check("2.4 CUDA GPU", False, "torch.cuda.is_available()=False!")
except Exception as e:
    check("2.4 PyTorch", False, str(e)[:60])

try:
    import torchvision
    import torchvision.transforms as T
    check("2.4 torchvision", True, f"v{torchvision.__version__}")
except Exception as e:
    check("2.4 torchvision", False, str(e)[:60])

# ── 2.5 Python 依赖 ─────────────────────────────────────
deps = [
    ("ultralytics", "YOLO"),
    ("pyrealsense2", "D435 驱动"),
    ("cv2", "OpenCV"),
    ("numpy", "NumPy"),
    ("scipy", "SciPy"),
    ("h5py", "HDF5"),
    ("transforms3d", "变换工具"),
]
for mod, desc in deps:
    try:
        m = importlib.import_module(mod)
        v = getattr(m, "__version__", "?")
        ok = True
        if mod == "numpy" and v >= "2.0":
            ok = False
            v = f"{v} (需要 <2.0!)"
        check(f"2.5 {desc} ({mod})", ok, str(v))
    except Exception as e:
        check(f"2.5 {desc} ({mod})", False, str(e)[:60])

# ── 2.6 YOLO 模型 ──────────────────────────────────────
yolo_path = os.path.expanduser("~/.config/Ultralytics/yolov8n.pt")
if os.path.exists(yolo_path):
    size = os.path.getsize(yolo_path) / (1024*1024)
    check("2.6 YOLO 模型", True, f"{yolo_path}  ({size:.1f}MB)")
else:
    check("2.6 YOLO 模型", False, "未找到 ~/.config/Ultralytics/yolov8n.pt")

# ── 2.7 Jetson IP / 网络 ─────────────────────────────────
code, out = run("ip addr show 2>/dev/null | grep -E 'inet ' | grep -v 127.0.0.1")
ips = []
for line in out.split("\n"):
    parts = line.strip().split()
    if parts:
        ips.append(parts[1].split("/")[0])
check("2.7 网络 IP", len(ips) > 0, ", ".join(ips))

# 机械臂可达
code, out = run("ping -c 2 -W 2 192.168.1.13 2>/dev/null")
ping_ok = code == 0 and "0% packet loss" in out
check("2.7 机械臂可达 (192.168.1.13)", ping_ok, out.split("\n")[-2].strip() if out else "不可达")

# ── 磁盘空间 ─────────────────────────────────────────────
code, out = run("df -h / | tail -1")
disk_info = out.strip() if out else ""
check("磁盘空间", len(out) > 0, disk_info)

# ── 汇总 ─────────────────────────────────────────────────
print()
print("=" * 60)
passed = sum(1 for _, ok, _ in results if ok)
total = len(results)
print(f"汇总: {passed}/{total} 项通过")
if passed == total:
    print("环境就绪，可以进入阶段 3（文件部署）！")
elif passed >= total - 3:
    print("基本可用，少量项目需修复后再进入下一阶段。")
else:
    print("多项未通过，请优先修复 ❌ 项目。")
print("=" * 60)

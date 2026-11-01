# 机械臂灵巧手 VA 模型自主抓取方案

> 硬件：Jetson Orin Nano (157 TOPS) + TCB605 机械臂 + ROH-LiteS001 灵巧手 + Intel D435
> 配置：单目相机，眼在手上 (eye-in-hand)
> 日期：2026-08

---

## 1. 方案概述

采用 **Diffusion Policy** 视觉-动作模型，端到端地从 RGB 图像预测机械臂关节轨迹和灵巧手动作，替代现有的 YOLO + 深度 + 手眼矩阵模块化管线。

```
┌──────────┐    ┌──────────────┐    ┌──────────────┐    ┌──────────┐
│ D435 RGB │───→│ ResNet-18    │───→│ 1D Conv UNet │───→│ 16步关节  │
│ 224×224  │    │ 视觉编码器   │    │ 扩散去噪     │    │ 轨迹+手   │
└──────────┘    └──────────────┘    └──────────────┘    └──────────┘
       ↑                                                    │
       当前关节角度(6维)                                    ↓
       ───────────────────→ 拼接 ←────────────────── ServoJ + ModBus
```

## 2. 模型架构

### 2.1 网络结构

| 组件 | 选型 | 参数量 | 说明 |
|------|------|:---:|------|
| 视觉编码器 | ResNet-18 | ~11M | ImageNet 预训练，冻结或微调 |
| 噪声预测网络 | 1D Conv UNet | ~18M | 去噪扩散，预测动作轨迹 |
| 输入 | 224×224×3 RGB + 6维关节角 | - | 观察 + 当前状态 |
| 输出 | 16步×7维动作 | - | 6关节增量 + 手开/合指令 |
| 总参数 | - | ~30M | |
| 推理显存 | - | ~3GB | TensorRT 后约 1.5GB |

### 2.2 动作空间

```
动作 = [ΔJ1, ΔJ2, ΔJ3, ΔJ4, ΔJ5, ΔJ6_lock, grip]
       └────── 关节增量(度) ──────┘              └─ 0=不动 1=张开 2=握拳
```

- J1~J5：关节角度增量（度），每步最大 ±5°
- J6：锁定不输出（关节损坏）
- grip：离散动作，0=保持，1=张开，2=握拳

### 2.3 推理流程

```
1. 机械臂移动到观测位（相机朝下，距离桌面 ~30cm）
2. D435 拍摄 RGB 图像 (224×224)
3. 读取当前关节角度
4. 模型推理（50ms，TensorRT 加速）
5. 扩散去噪 100 步 → 16 步关节轨迹
6. ServoJ 逐帧执行轨迹（每步 20ms，共 320ms）
7. 到达抓取位 → 握拳 → 抬起 → 回零
```

## 3. 数据采集

### 3.1 采集方式

使用 `vision_grasp_demo.py` 进行数据采集：

```bash
cd ~/Desktop/hand_demo_arm
source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
python3 vision_grasp_demo.py
# 按 g 抓取，自动保存到 ~/grasp_data/
```

### 3.2 数据格式

每个 episode 保存为 HDF5 文件：

```
episode_0000.hdf5
├── image        : (224, 224, 3) uint8  观测图像
├── joints       : (6,) float32         抓取时关节角度
└── object_name  : str                  物体类别
```

### 3.3 采集要求

| 要求 | 说明 |
|------|------|
| 物体多样性 | 至少 10 种不同物体（方块、圆柱、日常物品） |
| 位置多样性 | 桌面不同位置（覆盖工作空间） |
| 姿态多样性 | 物体不同朝向 |
| 光照变化 | 不同时间/开关灯 |
| 总量 | **≥ 500 条**，建议 1000+ |
| 数据增强 | 训练时随机裁剪、颜色抖动、旋转 |

## 4. 模型训练

### 4.1 环境准备

```bash
pip install torch torchvision diffusers accelerate
pip install h5py opencv-python einops tensorboard
```

### 4.2 训练配置

| 参数 | 值 | 说明 |
|------|-----|------|
| batch_size | 64 | |
| learning_rate | 1e-4 | AdamW |
| diffusion_steps | 100 | 训练时去噪步数 |
| horizon | 16 | 预测步数 |
| obs_horizon | 1 | 观察帧数 |
| epochs | 500 | |
| GPU | A800 或 Jetson | A800 约 4 小时，Jetson 约 20 小时 |

### 4.3 训练策略

```
阶段1 (epoch 0-100)：
  - 冻结 ResNet-18 视觉编码器
  - 只训练 UNet 去噪网络

阶段2 (epoch 100-300)：
  - 解冻 ResNet-18 后两层
  - 联合微调

阶段3 (epoch 300-500)：
  - 学习率衰减 0.1×
  - 增加数据增强强度
```

## 5. 部署到 Jetson

### 5.1 模型优化

```bash
# 1. PyTorch → ONNX
python3 export_onnx.py --model_path checkpoints/best.pt --output model.onnx

# 2. ONNX → TensorRT (Jetson上执行)
trtexec --onnx=model.onnx --saveEngine=model.engine --fp16
```

### 5.2 推理节点

创建 `va_inference_node.py`：

```python
#!/usr/bin/env python3
"""VA 模型推理节点"""
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

class VAInference:
    def __init__(self, engine_path):
        # 加载 TensorRT 引擎
        self.engine = self._load_engine(engine_path)
        self.context = self.engine.create_execution_context()

    def infer(self, image, current_joints):
        # 预处理: resize → normalize → concat
        input_tensor = self._preprocess(image, current_joints)
        # 推理
        output = self._run_inference(input_tensor)
        # 后处理: 解析轨迹 + 手动作
        trajectory, grip = self._postprocess(output)
        return trajectory, grip

    def _load_engine(self, path):
        runtime = trt.Runtime(trt.Logger(trt.Logger.WARNING))
        with open(path, 'rb') as f:
            return runtime.deserialize_cuda_engine(f.read())
```

### 5.3 运行

```bash
# 终端1: tl_driver
source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
ros2 launch tl_driver tl_driver.launch.py arm_type:=tcb605

# 终端2: 相机 D435
cd ~/Desktop/hand_demo_arm
source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
python3 camera_node.py

# 终端3: VA 推理
python3 va_inference_node.py
```

## 6. 新旧方案对比

| 维度 | 现有模块化管线 | VA 模型管线 |
|------|:---:|:---:|
| 模型 | YOLO + 深度 + 手眼矩阵 | 单次端到端推理 |
| 检测 | 显式目标检测 | 隐式学习抓取点 |
| 定位 | D435 深度计算 3D | 网络隐式推理空间关系 |
| 泛化 | 限 YOLO 训练类别 | 可泛化到未见物体 |
| 推理时间 | ~200ms | ~50ms (TensorRT) |
| 依赖 | YOLO模型 + D435 + 矩阵 | 仅 RGB 图像 |
| 灵巧手配合 | 硬编码握拳/张开 | 模型自主决定 |

## 7. 实施路线图

```
Phase 1 (当前)：跑通现有管线
  ├── 工具标定（测量灵巧手 TCP 偏移）
  ├── vision_grasp_demo.py 稳定抓取
  └── 确认 seq_move.py 机械臂运动正确

Phase 2 (1-2周)：数据采集
  ├── 准备 10+ 种物体
  ├── 采集 ≥ 500 条 episode
  └── 数据清洗 + 可视化检查

Phase 3 (1周)：模型训练
  ├── 编写训练脚本
  ├── A800 或 Jetson 本地训练
  └── TensorBoard 监控 loss 曲线

Phase 4 (1周)：部署测试
  ├── ONNX/TensorRT 导出优化
  ├── 实机推理测试
  └── 安全保护（力限制、急停）

Phase 5 (持续)：迭代优化
  ├── 增加数据量
  ├── 尝试更大视觉编码器 (EfficientNet-B3)
  └── 支持多阶段任务（抓取→放置）
```

## 8. 安全注意事项

1. **推理前检查**：当前关节角度在安全范围内才执行
2. **力限制**：灵巧手握拳时电流限制 ≤ 500mA
3. **Z 轴下限**：末端 Z 坐标不低于桌面高度 + 30mm
4. **急停**：键盘 `q` 或示教器急停按钮
5. **先验验证**：推断的抓取点深度值 > 0 且 < 1.5m

## 9. 文件清单

```
~/Desktop/hand_demo_arm/
├── vision_grasp_demo.py       # 现有抓取+数据采集
├── camera_node.py             # D435 相机驱动
├── va_inference_node.py       # VA 模型推理（待实现）
├── train_va.py                # 训练脚本（待实现）
└── export_onnx.py             # 模型导出（待实现）

~/grasp_data/                  # 训练数据
├── episode_0000.hdf5
├── episode_0001.hdf5
└── ...

~/va_model/                    # 模型文件
├── best.pt                    # PyTorch 权重
├── model.onnx                 # ONNX 导出
└── model.engine               # TensorRT 引擎
```

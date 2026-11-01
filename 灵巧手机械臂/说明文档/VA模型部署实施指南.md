# 灵巧手机械臂 VA 模型部署实施指南

> 日期：2026-08-05
> 硬件：Jetson Orin Nano 16GB + 天链 TCB605 六轴机械臂 + 傲意 ROH-LiteS001 灵巧手 + Intel RealSense D435（眼在手）
> 目标：Diffusion Policy 视觉-动作模型端到端自主抓取，替代 YOLO+深度+手眼矩阵模块化管线
> 前置文档：《VA模型自主抓取方案.md》（模型架构设计）、`hand_demo_arm/` 现有模块化管线（保留作回退）

---

## 一、模型选型结论

**推荐：Diffusion Policy（ResNet-18 视觉编码器 + 1D Conv UNet 去噪网络，~30M 参数）**

### 1.1 候选模型对比

| 维度 | BC（ResNet18-MLP） | **Diffusion Policy** ✅ | Octo (93M) | OpenVLA (7B) |
|------|:---:|:---:|:---:|:---:|
| 参数量 | 12M | **30M** | 93M | 7B (INT4 ~3.5GB) |
| 最少数据量 | 200 条 | **500 条** | 1000+ 条 | 1000+ 条 + 语言标注 |
| Jetson 推理延迟 | ~5ms | **~50ms (TRT FP16)** | ~80-100ms | 200-500ms ❌ |
| 训练复杂度 | 低 | 中（自写可控） | 高（LoRA/环境重） | 极高 |
| J6 损坏适配 | 简单 | **简单（输出通道锁 0）** | 需改动作空间 | 需改 tokenizer |
| ONNX/TRT 导出 | 易 | **可控** | 麻烦 | 麻烦 |
| 语言指令（VLA） | — | 可后接 CLIP 增量升级 | 原生支持 | 原生支持 |

### 1.2 选型理由

1. **动作空间吻合**：动作 = `[ΔJ1..ΔJ5, J6=锁0, grip]` 7 维，DP 输出为向量可直接裁剪约束；OpenVLA/Octo 需改造动作 tokenizer，工作量大
2. **数据量匹配**：500 条带轨迹数据即可训练；OpenVLA 需 1000+ 条且逐帧语言标注
3. **部署可控**：纯 PyTorch 网络，ONNX→TensorRT FP16 导出完全可控
4. **升级路径清晰**：训通后在视觉编码器旁并联 CLIP 文本编码器即可升级语言指令（VLA），架构无需推倒重来

### 1.3 升级条件

DP 抓取成功率稳定 ≥80% 后，再评估 Octo（需数据 1000+ 条）。

---

## 二、⚠ 关键前置：数据采集必须改造为轨迹流

**现状缺口**：现有采集程序（`vision_grasp_demo.py` 的 `_save_episode`）每条 episode 只保存 **1 帧图像 + 1 个末端关节角**（抓取时刻静帧），而 Diffusion Policy 训练需要 **16 步轨迹序列**。**采集 500 条静帧无法训练轨迹扩散模型**，Phase 0 必须先改造采集程序。

### 2.1 眼在手 + 拖动采集：可行，且是主流做法

- 原理：DP 训练时每步观测 = 该时刻相机画面；拖动时相机跟着末端走，每步画面即"此刻该看到的"，与部署时输入分布一致。眼在手不存在"不能拖动"的问题。
- 公开验证：UMI（手持夹爪 + 腕装相机采集 → UR5e/Franka 部署）、Dobb-E（iPhone 手持采集）均为此类方案。

### 2.2 现有 `drag_collect.py` 的不足与改造方向

现有脚本只记录 2 个状态（`r` 观测位 + `d` 抓取位），**不是轨迹**。需升级为**连续轨迹流录制**：

- 拖动全程 30fps 存图 + 关节角，按 ROS2 时间戳离线对齐（UMI 同款做法）
- 键盘键绑定"张开/握拳"，切换时刻打时间戳，作为每步离散 grip 通道记录
- 输出 HDF5（见 2.4 数据格式）

### 2.3 硬件注意点

| 注意点 | 现状 | 处理 |
|--------|------|------|
| 拖拽模式 | ARM 版无 `set_drag_mode` API | 用示教器 **P 键**进出拖拽（已确认可用） |
| 关节角采样率 | `/joint_states` 仅 **10Hz**（16 步×20ms=320ms 轨迹仅 ~3 个采样点） | ① 查 `tl_driver` 发布频率可否调高；② 按 10Hz 记录、动作间隔放宽至 100ms/步、horizon=16（1.6s）；③ 图像与关节按时间戳插值对齐 |
| 灵巧手动作记录 | 拖拽时手不会自己动 | 键盘键绑"张开/握拳"，记录动作时刻作为 grip 通道标签 |
| J6 损坏 | 拖拽时 J6 物理不动 | 数据中 J6 天然≈恒定，训练时该通道 mask 掉 loss |

### 2.4 数据格式（HDF5，兼容 DP 训练）

```
episode_0000.hdf5
├── obs_image   : (224,224,3) uint8      # 观测位图像（抓取前）
├── obs_joints  : (6,) float32           # 观测位关节角
├── actions     : (16,7) float32         # 16 步增量轨迹 [ΔJ1-5, 0, grip]
├── grip        : int                    # 0保持/1张开/2握拳
└── object_name : str
```

### 2.5 采集规模与多样性

| 要求 | 说明 |
|------|------|
| 物体多样性 | 至少 10 种不同物体（方块、圆柱、日常物品） |
| 位置多样性 | 桌面 3×3 网格覆盖工作空间 |
| 姿态/光照变化 | 不同朝向、白天/夜晚/补光 |
| 总量 | **≥ 500 条**，推荐 1000+ |
| 数据增强 | 训练时随机裁剪、颜色抖动、旋转 |

---

## 三、分阶段实施步骤

### Phase 0：数据采集改造（1-2 天）— 最高优先级

改造 `vision_grasp_demo.py` 自动循环或新建 `record_trajectory.py` / 升级 `drag_collect.py`：

```python
# 核心改动：抓取执行/拖动期间同步录制
traj_images  = []   # 轨迹中每步图像（或每步观测帧）
traj_joints  = []   # 每步 6 关节角（/joint_states 订阅回调）
actions      = []   # 每步 [ΔJ1..ΔJ5, 0, grip]   ← 训练标签
```

- 自动循环模式（摆好物体→自动抓取→自动记录→归位）一人可操作
- 拖动示教同步升级为全程轨迹记录

### Phase 1：训练环境准备（0.5 天）

Jetson 自训（无需 A800，慢但可行；有 A800 优先服务器）：

```bash
pip3 install h5py einops tensorboard diffusers   # 轻量依赖
```

**建议自写轻量 DP 实现**（不依赖官方 diffusion-policy repo）：官方 repo 依赖 PyTorch Lightning + Hydra，ONNX 导出极其麻烦；本项目仅 1 观测帧 + 16 步 horizon，自写 ~300 行即可，导出全可控。

### Phase 2：模型训练（2-5 天）

| 组件 | 配置 | 说明 |
|------|------|------|
| 视觉编码器 | ResNet-18，ImageNet 预训练 | 冻结→解冻后两层 |
| 条件注入 | FiLM（每层 scale+shift） | 图像特征 + 6 维关节角 |
| 噪声网络 | 1D Conv UNet，~18M | 时间步正弦嵌入 |
| 扩散 | DDPM，100 去噪步，EMA | 训练时随机采样 t∈[0,1000] |
| 输出 | 16 步 × 7 维 | ΔJ 归一化到 [-1,1] |
| 超参数 | batch 64（Jetson 16）、lr 1e-4 AdamW、epochs 500 | 训练/推理分离 |

训练策略（三阶段）：epoch 0-100 冻结 ResNet → 100-300 解冻后两层联合微调 → 300-500 学习率 ×0.1。

**关键细节**：
- J6 输出通道恒置 0（损坏关节，训练时 mask loss）
- ΔJ 限制 ±5°/步（超限 clamp）
- **先 50 条小数据跑通 训练→导出→推理 全闭环验证数据质量，再上量**

### Phase 3：ONNX → TensorRT 导出（0.5 天）

```bash
python3 export_onnx.py --checkpoint best.pt --output model.onnx   # opset 17（GroupNorm 需要）
/usr/src/tensorrt/bin/trtexec --onnx=model.onnx --saveEngine=model.engine --fp16
# 验证：测试集比对 PyTorch 与 TRT 输出，最大误差 <1e-3
```

导出注意：时间步条件作为额外输入张量传入（勿用 Python 循环内嵌）；固定 224×224 输入与固定 16 步输出，避免动态 shape。

### Phase 4：ROS2 推理节点 `va_inference_node.py`（1-2 天）

复用现有 `tl_driver` + `hand_controller.py`，仅替换"决策"部分：

```
/camera/color/image_raw ─┐
                         ├─▶ 预处理(224×224) ─▶ TRT 推理(~50ms) ─▶ 16步轨迹
/joint_states ───────────┘                                        │
                                                                  ▼
                   安全校验(限位/Z下限/ΔJ≤5°) → /tl_driver/set_servoj_pos (20ms/步)
                                                                  ▼
                                               modbus_write → 灵巧手 grip
```

**安全清单（必须实现）**：
1. 推理前检查：当前关节角在安全范围内才执行
2. Z 轴下限：末端 Z ≥ 桌面高度 + 30mm（复用 `z_min_limit` 参数）
3. 灵巧手电流限制 ≤500mA
4. 键盘 `q` + 示教器急停兜底
5. 轨迹先验：网络输出首点距观测位偏差 >15cm 直接拒绝执行

### Phase 5：联调验证（1-2 天）

- **验收标准**：50 次连续抓取成功率 ≥80%；单次决策到执行 <500ms（预处理 5ms + 推理 50ms + ServoJ 320ms）
- **回退方案**：现有 YOLO+深度+手眼矩阵管线保留为 fallback，一条命令切回

---

## 四、成功率预期与公开实机案例

### 4.1 公开成功率数据

| 任务 | 成功率 | 来源 |
|------|:---:|------|
| RoboMimic Lift（仿真） | 98-100% | [Diffusion Policy (arXiv:2303.04137)](https://arxiv.org/abs/2303.04137) |
| RoboMimic Can / Square | 66-96% / 90-92% | 同上 |
| Push-T（实机） | 95% | 同上 |
| Mug Flip（实机） | 90% | 同上 |
| 6DoF Pour（实机） | 79% | 同上 |
| UMI 抓取类任务（腕装相机，UR5e/Franka） | 70-90% | [UMI (arXiv:2402.10329)](https://arxiv.org/abs/2402.10329) |
| FARM 插植物/摘葡萄/拧螺丝（D405 眼在手） | 95/95/100% | TU Darmstadt 实机 |
| MinD 腕装相机 Pick&Place / 拔充电器（Franka FR3） | 60% / 80% | arXiv:2506.18897 |

**预期**：论文 80-100% 为专业示范+充分调参的稳态；本系统首版合理预期 60-70%，数据质量与多样性提升后冲 80%+；"桌面固定位姿单物体抓取"受限任务稳态 85-95% 可达。

### 4.2 相关实机案例参考

| 项目 | 硬件 | 借鉴点 |
|------|------|------|
| [UMI (Chi et al., RSS 2024)](https://arxiv.org/abs/2402.10329) | 手持夹爪+GoPro 腕装鱼眼 → UR5e/Franka | 与本案几乎同构：拖动式采集 + 眼在手 + DP，跨机械臂部署 |
| [Diffusion Policy 原论文](https://arxiv.org/abs/2303.04137) | Aperture/Franka 等 10+ 任务 | 实时控制、任务无关策略框架 |
| [Dobb-E (NYU)](https://arxiv.org/abs/2311.16098) | iPhone 手持采集 → Stretch，30 任务 77% | 非专业人手把手示范数据验证拖动采集 |
| [AnyTeleop (RSS 2023)](https://arxiv.org/abs/2307.04577) | 视觉遥操作臂+灵巧手 | 灵巧手+DP 标准数据管道 |
| [Open-TeleVision](https://arxiv.org/abs/2407.01512) | 遥操作 + 3D 扩散策略，人形+灵巧手 | 灵巧手动作与 DP 结合 |
| Allegro 手拧瓶盖 (arXiv:2503.02587) | Allegro 四指手，300 条遥操作示范 | 灵巧手+DP 实机案例，HDBSCAN 清洗异常轨迹 |

**结论**：本系统组合（6 轴臂 + ROH-LiteS001 + D435 眼在手）即"UMI 式眼在手 DP + 灵巧手动作通道"——手无需逐指连续控制（ModBus 寄存器位置控制），作为 1 个离散 grip 通道（0保持/1张开/2握拳）处理，比逐指控制更简单更稳。

---

## 五、时间线与里程碑

| 阶段 | 周期 | 产出 |
|------|------|------|
| **Phase 0 轨迹采集改造** | 1-2 天 | 轨迹版 HDF5 采集器 |
| 数据积累（可与 Phase 1-2 并行） | 1 周 | 500-1000 条轨迹 |
| Phase 1-2 训练 | 2-5 天 | best.pt（先 50 条小闭环） |
| Phase 3 导出 | 0.5 天 | model.engine (FP16) |
| Phase 4 推理节点 | 1-2 天 | va_inference_node.py |
| Phase 5 联调 | 1-2 天 | 成功率 ≥80% |

**总计约 2-3 周**

---

## 六、风险与对策

| 风险 | 对策 |
|------|------|
| 轨迹数据质量差（模糊/位姿未对齐） | 录制后可视化回放检查；50 条小闭环先行 |
| TRT 导出失败（GroupNorm/动态维度） | 训练时固定 224×224 与 16 步，避免动态 shape |
| Jetson 训练过慢（~20h） | 冻结 ResNet 阶段先训；有 A800 直接上服务器 |
| 模型学到"假抓取"（grip 时序错乱） | grip 独立通道 + 加大损失权重 |
| J6 锁零导致轨迹奇异 | 数据侧 J6 恒 0 + 训练时 mask，约束在数据侧解决 |
| 关节角采样率不足（10Hz） | 见 §2.3：调频率 / 放宽步长 / 时间戳插值 |

---

## 七、参考来源

- Chi et al., "Diffusion Policy: Visuomotor Policy Learning via Action Diffusion", RSS 2023 — https://arxiv.org/abs/2303.04137
- Chi et al., "Universal Manipulation Interface: In-The-Wild Robot Teaching Without In-The-Wild Robots", RSS 2024 — https://arxiv.org/abs/2402.10329
- Doshi et al., "Dobb-E" — https://arxiv.org/abs/2311.16098
- Qin et al., "AnyTeleop: A General Vision-Based Dexterous Robot Arm-Hand Teleoperation System", RSS 2023 — https://arxiv.org/abs/2307.04577
- Cheng et al., "Open-TeleVision: Teleoperation with Immersive Active Visual Feedback" — https://arxiv.org/abs/2407.01512
- "Learning Dexterous In-Hand Manipulation via Visuomotor Diffusion" (Allegro 实机) — https://arxiv.org/abs/2503.02587

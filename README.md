# TL 机械臂 / 灵巧手项目资料总集

## 项目简介
本仓库汇总 **天链(TL)系列机械臂与灵巧手** 相关的代码、固件、ROS 包、标定工具、演示程序与说明文档,覆盖"底层电机 / 固件 → ROS2 控制 → 手眼标定视觉 → 上层 VLA / LLM 智能体"的完整技术链路。

> 原仓库标题 **TL-VISON**:TL 机械臂视觉抓取项目。视觉抓取是其中的核心应用之一。

## 目录结构总览

| 目录 | 内容概要 |
|---|---|
| `TL/` | 天链主工程,内含 `TL_Dobot`(Dobot 机械臂)、`TL_Handback`(灵巧手背部)、`aruco`(ArUco 标定)、`scripts`、构建脚本与 `开发` 等 |
| `TL_Dobot/` | DOBOT 机械臂工程:源码 `src/`、脚本 `scripts/`、文档 `docs/`、`assets/`、Jetson 依赖 `requirements-jetson.txt` |
| `TL_Hand/` | 灵巧手控制:源码 `src/`、示例图 `picture.jpg` |
| `TL_Handback/` | 灵巧手背部:ArUco 码(`aruco_codes` / `aruco_codes_192`)、`docs`、`scripts`、`src`、`tmp_review` |
| `TL_LLM_demo/` | 基于 LLM 的机械臂智能体:`llm_tl_agent.py`、`voice_stt.py` 及多篇 Nano / Ollama 部署笔记 |
| `TL_TCB/` | TCB(天链控制盒)控制与抓取:`arm_hand_demo.py`、`grasp_test.py`、`hand_controller.py`、`vision_grasp_demo.py`、`collect_data_arm.py` 等 |
| `TL_back/` | 备份:`_snapshot.zip` 与 `_snapshot` |
| `TL_lowerobs_test/` | 下视观测(lower observation)测试:`scripts/`、`src/` 及说明文档 |
| `tl_robot_ros2_py-arm/` | 标准 ROS2 Python 机械臂工程:`src/`、`pyproject.toml`、`.gitmodules`、`AGENTS.md`、LICENSE |
| `vla_demo/` | VLA 演示:`install_nano_smolvla.sh`(Jetson 部署 SmolVLA)、`tcb_find_infer_server.py`、`tcb_find_policy_node.py` |
| `园艺车机械臂灵巧手资料/` | 园艺车场景资料:操作说明书、电机、控制文档 |
| `大模型部署说明文档/` | 大模型本地部署:Ollama 部署、视觉部署、控制文档 |
| `开发/` | 开发相关(占位目录) |
| `新建文件夹 (2)/` | 汇总资料:`DOBOT_6Axis_ROS2_V4-main`(6 轴 ROS2 V4)、`robot_hand`(灵巧手全套)、`HandEyeCalibrateForRobot`(手眼标定)及多份 zip / calib.io 标定板 PDF |
| `灵巧手机械臂/` | 灵巧手综合资料:`d435`(RealSense)、`hand_demo` / `hand_demo_arm`、`roh_firmware`(ROH 固件)、`tl_vision`(视觉)、YOLO+D435 演示、环境检查及多份说明 |

## 重点子模块

### robot_hand(灵巧手全套,位于 `新建文件夹 (2)/robot_hand`)
- `tl_vision/` — 手眼标定视觉代码与模型(含 `hand_eye_calibration` 标定工程、模型权重)。
- `roh_firmware/` — ROH 灵巧手固件与 SDK:`OHandSetting`(Ubuntu22 / Ubuntu24 / Windows 跨平台设置)、`firmware`、`protocol`、`UserManual`。
- `ROS2/` — 机械臂 ROS2 包:`cpp/`(`tl_description` 模型描述、`tl_teleop` 遥操作、`tl_driver` 驱动、`tl_bringup`、`tl_moveit2_config` 等)、`py/`(`arm` / `x86` Python 包)。

### DOBOT 6 轴 ROS2
`新建文件夹 (2)/DOBOT_6Axis_ROS2_V4-main`:6 轴机械臂 ROS2 V4 工程。

### 视觉抓取相关
- `TL_TCB/vision_grasp_demo.py` — TCB 视觉抓取演示
- `灵巧手机械臂/yolo_d435_demo.py` — YOLO + RealSense D435 抓取演示
- `robot_hand/tl_vision` — 手眼标定视觉

### VLA / LLM 智能体
- `vla_demo/` — SmolVLA 在 Jetson 上的部署与推理服务
- `TL_LLM_demo/` — 基于本地 LLM(Ollama)的机械臂智能体 + 语音 STT

## 技术栈
- **控制 / 机器人**:ROS2(C++ / Python)、MoveIt2、RealSense(D435)、ArUco 标定
- **视觉 / 抓取**:OpenCV、YOLO、手眼标定(eye-in-hand)、SmolVLA(VLA 模型)
- **智能体**:本地 LLM(Ollama)、语音识别(STT)、VLA 推理服务(zenoh / gRPC)
- **固件 / 硬件**:ROH 灵巧手固件与 SDK、TCB 控制盒、Dobot、Jetson

## 使用与注意事项
1. **大文件 / 压缩包**:本仓库含多个 `.zip`、模型权重、固件 SDK 等较大文件。如需仅查看结构,可用 `git clone --filter=blob:none <repo>` 进行 blobless 克隆(只下载目录树、不下载文件内容)。
2. **跨平台二进制**:`robot_hand/roh_firmware/OHandSetting` 含 `.so` / `.dll` 等原生库,使用前注意目标平台与执行权限。
3. **ROS2 环境**:`tl_robot_ros2_py-arm`、`robot_hand/ROS2` 等需对应 ROS2 发行版(Humble / Iron 等)及依赖(`pyproject.toml` / `.gitmodules`)。
4. **边缘部署**:`vla_demo` 的 SmolVLA 脚本面向 Jetson Nano / Orin 等边缘设备。
5. **重复副本**:资料中同一工程可能存在"目录源码 + `.zip` 归档"的多份副本(例如 `tl_robot_ros2_py-arm`),以目录源码为最新,zip 多为备份归档。

---

*本 README 由仓库结构自动梳理生成,用于说明各目录用途与整体技术链路。*

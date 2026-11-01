# hand_demo_arm — Jetson Orin Nano 灵巧手+机械臂联调

## 文件清单

```
hand_demo_arm/
├── control_node.py        # 主控制节点 (ARM 适配版)
├── control_node.yaml      # 参数配置文件
├── hand_controller.py     # 灵巧手 ROH-LiteS001 ModBus 控制库
├── start_hand_demo.sh     # 一键启动脚本
└── README.md              # 本文件
```

## ARM 版 vs x86 VM 版的区别

| 功能 | x86 VM 版 | ARM 版 (本目录) |
|------|-----------|----------------|
| 六维力传感器 GetSensor6DData | ✅ SWIG | ❌ ctypes 不支持 |
| 六维力传感器 MarkBaseSensor | ✅ SWIG | ❌ ctypes 不支持 |
| SetToolParam 服务 | ✅ | ⚠️ 可选 (try/except) |
| 关节状态话题 | `/nano/joint_states` | `/joint_states` (直接) |
| 视觉接收 (TCP) | ✅ | ✅ (同 x86，依赖 Windows 视觉) |
| ServoJ 插值运动 | ✅ | ✅ |
| 坐标变换 | ✅ | ✅ |
| ModBus 灵巧手控制 | ✅ | ✅ |
| 自动循环抓取 | ✅ | ✅ |

## 前置条件

1. **tl_driver_node 已启动并连接机械臂**
   ```bash
   source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
   ros2 launch tl_driver tl_driver.launch.py arm_type:=TCB605
   ```
   等待日志显示连接成功、上电完成。

2. **灵巧手已连接到机械臂末端 RS485 端口 (端口2)**

3. **Windows 视觉程序已运行**（如果需要视觉引导抓取）
   - vision_receiver.py 在 Ubuntu 端运行以接收 Windows 检测结果
   - 或者直接将检测结果发布到 `/tl_vision/object_3d_pos_camera` 话题

## 启动方式

### 方式一：一键启动
```bash
cd ~/Desktop/hand_demo_arm
bash start_hand_demo.sh
```

### 方式二：手动启动
```bash
cd ~/Desktop/hand_demo_arm
source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
python3 control_node.py --ros-args --params-file control_node.yaml
```

## 键盘控制

| 按键 | 功能 |
|------|------|
| `h` | 检测→抓取→放下→回零（完整流程） |
| `t` | 灵巧手开/闭切换 |
| `p` | 依次经过 after_grasp_waypoints 点位 |
| `z` | 回零点 |
| `c` | 中断当前操作 |
| `q` | 退出 |

> **ARM 版不支持 `f`(读力) 和 `m`(力标零)**

## 灵巧手控制 (ROH-LiteS001)

灵巧手通过机械臂末端 RS485 (端口2) 通信，由 tl_driver 的 ModBus 服务透传。

**关键寄存器:**
- `POS_TARGET` (1135-1140): 6指目标位置, 0=张开 ~ 65535=闭合
- `START_INIT` (1013): 初始化, 写入1触发
- `SELF_TEST_LEVEL` (1008): 自检等级, 先写1再写START_INIT

**首次使用灵巧手:**
```python
from hand_controller import ROHHandController

hand = ROHHandController(node)  # node 是任意 ROS2 Node
hand.init()                     # 读取固件版本
hand.start_init()               # 初始化（手会咔咔响，正常）
hand.open()                     # 张开
hand.close()                    # 握拳
```

## 配置说明

编辑 `control_node.yaml` 修改：

- `grasp_target_z`: 抓取时的 Z 高度 (mm)
- `grasp_approach_z`: 接近抓取点的过渡高度 (mm)
- `after_grasp_waypoints`: 抓取后经过的点位
- `modbus_gripper_close_val`: 闭合时各指位置值
- `modbus_gripper_open_val`: 张开时各指位置值
- `descend_y_offset`: 下降时 Y 方向补偿 (mm)
- `servoj_vmax/amax/jmax`: ServoJ 速度/加速度/加加速度限制

## 故障排查

### 连接不上机械臂
- 确认 tl_driver 已启动: `ros2 node list | grep tl_driver`
- 确认机械臂 IP 可 ping 通: `ping 192.168.1.13`

### 灵巧手无响应
- 检查 ModBus 参数: slave_id=2, port=2, baudrate=115200
- 机械臂 RS485 线是否接好（末端14芯航插 9-10针）
- 首次需先 `start_init()` 初始化

### ServoJ 不动
- 确认 tl_driver 已上电: `ros2 service call /tl_driver/power_on std_srvs/srv/Trigger`
- 确认 open_servoj 成功: 查看启动日志

### 视觉无数据
- 确认 vision_receiver.py 运行并接收到 Windows 数据
- 检查话题: `ros2 topic echo /tl_vision/object_3d_pos_camera`

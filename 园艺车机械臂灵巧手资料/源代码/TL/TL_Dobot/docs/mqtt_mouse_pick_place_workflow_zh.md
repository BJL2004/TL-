# MQTT + 鼠标点击抓取流程

## 1. 当前功能边界

- 目标物体：只使用 D435 画面中的鼠标点击位置，不再自动识别前景物体。
- `seed`：接收任务后等待人工点击物体并按 `C`；在 `TL_PLACE_ONLY=1` 下，再按 `R` 定位花篮、按 `G` 放置。
- `trim_pick`、`trim_place`：协议已接收和反馈，但修根抓取位、修根放置位尚未示教，因此当前固定反馈 `retCode=1`，不会发送机械臂运动。
- MQTT 不绕过灵巧手状态检查、IK、工作空间检查、ArUco 定位或 ServoJ 到位检查。

## 2. MQTT 默认参数

| 参数 | 默认值 |
|---|---|
| Broker | `127.0.0.1:1883` |
| 下行主题（车到机械臂） | `tl/robot/command` |
| 上行主题（机械臂到车） | `tl/robot/feedback` |
| QoS | `1` |
| Client ID | `tl-tcb605-arm` |

安装依赖：

```bash
python3 -m pip install --user paho-mqtt
```

若 Jetson 不能联网，在联网电脑下载对应 wheel，复制到 Jetson 后离线安装。

## 3. 启动

第一轮只验证 MQTT 和机械臂流程，不使用夹爪或灵巧手。先启动天链驱动，再运行：

```bash
cd ~/Desktop/TL_Dobot
source /opt/ros/humble/setup.bash
source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash

TL_MQTT_HOST=192.168.1.100 \
bash scripts/run_mqtt_mouse_pick_place_test.sh
```

`192.168.1.100` 是示例，必须改为现场 Broker 的真实 IP。Broker 就在 Jetson 上时使用 `127.0.0.1`。程序显示 `MQTT connected and subscribed` 后才允许车端发布任务；10 秒内无法连接会直接报出 Broker、网络或账号错误，不再静默运行。

如需账号：

```bash
TL_MQTT_USERNAME=robot TL_MQTT_PASSWORD=your_password ...
```

## 4. `seed` 操作顺序

车端发布：

```json
{"cmd":"seed","targetId":"A1-01"}
```

机械臂端操作：

1. 终端显示 `MQTT seed accepted`。
2. 在 D435 窗口中点击种球中心。
3. 按 `C`。`TL_USE_HAND=0` 时仅模拟抓取，不访问夹爪或灵巧手。
4. 抓取完成后按 `R`，程序视觉定位花篮并移动到花篮检测位。
5. 按 `G`，程序执行放置并松手。
6. 只有完整放置完成后才发布：

```json
{"cmd":"seed","retCode":0}
```

失败返回 `1`；手已闭合或流程正忙返回 `2`。

## 5. targetId 与花篮编号

默认情况下，`targetId` 只作为任务编号，花篮仍由现有实时 ArUco 视觉流程选择。若车端编号固定映射到 1-6 号花篮，可启动时设置：

```bash
TL_MQTT_TARGET_MAP='{"A1-01":1,"A1-02":2}' \
TL_MQTT_ENABLED=1 \
TL_USE_HAND=1 \
TL_PLACE_ONLY=1 \
bash scripts/run_dynamic_aruco_basket_test.sh
```

未写入映射表的 `targetId` 继续使用实时花篮视觉选择。

## 6. 联调命令

发布任务：

```bash
mosquitto_pub -h 192.168.1.100 -t tl/robot/command -q 1 \
  -m '{"cmd":"seed","targetId":"A1-01"}'
```

监听反馈：

```bash
mosquitto_sub -h 192.168.1.100 -t tl/robot/feedback -q 1 -v
```

联调时使用专用脚本验证 MQTT、鼠标点击和花篮路径。该脚本强制 `TL_USE_HAND=0`，不会访问夹爪或灵巧手。

# 假灵巧手 TCP 测试

本测试用于在不安装傲意灵巧手的情况下，用一个刚性轻质挂件模拟灵巧手抓取中心。

## 1. 参数含义

程序新增两个参数：

```bash
--mock-grasp-offset-mm X Y Z
--mock-grasp-clearance-mm C
--grasp-rpy-rad RX RY RZ
```

`--mock-grasp-offset-mm` 是从 `tool0` 法兰中心到假挂件尖端/模拟抓取中心的偏移，单位 mm，坐标方向采用机械臂 `tool0` 坐标系。

`--mock-grasp-clearance-mm` 是假挂件尖端距离点击目标中心的安全高度。比如挂件最长 150 mm，希望尖端离目标 10 mm，则不是填 `hover=160`，而是填：

```bash
--mock-grasp-offset-mm 0 0 -150
--mock-grasp-clearance-mm 10
```

如果挂件尖端不在 tool0 轴线上，必须把横向偏移也量出来。例如尖端相对 tool0 向 tool-X 正方向偏 35 mm、向 tool-Z 负方向 150 mm：

```bash
--mock-grasp-offset-mm 35 0 -150
```

`--grasp-rpy-rad` 用于把“检测时的相机姿态”和“抓取时的末端姿态”分开。平视相机时最容易出错：相机平视用于观察，但抓取/假手尖端可能需要另一套固定姿态。如果不传该参数，程序默认使用当前检测位的 tool0 RPY。

## 2. 操作顺序

1. 固定假挂件，确认刚性足够，运动中不会晃动。
2. 用尺测量 `tool0` 法兰中心到假挂件尖端的 `X/Y/Z` 偏移。
3. 先设置较大安全距离，比如 `20 mm`。
4. 点击目标中心，按 `c` 计算，检查 `mock grasp base` 和 `tool0 target TCP`。
5. 确认路径安全后按 `m` 运动。
6. 用 `a/d/s/w` 做 1 mm 微调，让假挂件尖端对准目标中心。
7. 按 `v` 保存偏差，按 `r` 回检测位。

## 3. 推荐命令

默认假设尖端在 tool0 的 `-Z` 方向 150 mm：

```bash
cd ~/Desktop/TL_Dobot
chmod +x scripts/*.sh
MOCK_GRASP_OFFSET_MM="0 0 -150" \
MOCK_GRASP_CLEARANCE_MM=20 \
./scripts/run_mock_hand_tcp_test.sh
```

如果平视相机时检测位姿态不是你想要的抓取姿态，先记录一个能够让假手正确朝向目标的 TCP RPY，然后固定传入。例如：

```bash
MOCK_GRASP_OFFSET_MM="0 0 -150" \
MOCK_GRASP_CLEARANCE_MM=20 \
MOCK_GRASP_RPY_RAD="3.087 -0.014 1.602" \
./scripts/run_mock_hand_tcp_test.sh
```

也可以用角度：

```bash
MOCK_GRASP_RPY_DEG="176.9 -0.8 91.8" \
./scripts/run_mock_hand_tcp_test.sh
```

如果发现尖端偏高，可以把 `MOCK_GRASP_CLEARANCE_MM` 改为 `10` 或 `5`。不要设置负数。

## 4. 重要提醒

这一步只验证“视觉定位 + 假手 TCP 偏移 + 机械臂运动”是否正确，不做真实抓取。等假挂件尖端能够稳定到达目标中心附近，再把 `mock-grasp-offset-mm` 换成真实灵巧手抓取中心偏移，进入灵巧手低速闭合保护测试。

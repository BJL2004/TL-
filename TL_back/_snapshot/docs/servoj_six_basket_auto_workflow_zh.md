# ServoJ 六花篮自主识别与俯视取放

## 1. 方案结论

当前工程继续采用 ServoJ 是合理的。天链驱动的原生 MoveJ 在本机上仍存在到位不完整的问题，而 ServoJ 已经完成真机到位验证。ServoJ 必须满足以下条件：

- 只有一个程序发布 `/tl_driver/set_servoj_pos`，不得同时运行 `safety_only.launch.py`、示教程序或第二个抓取程序。
- 保持 100 Hz 发布；程序使用最小加加速度插值，避免线性插值在起止位置产生突变。
- 自由空间转运和精细接近使用不同速度，不能为了转运提速而让灵巧手高速进入物体或花篮。
- 每段运动都检查最终关节误差；没有显示 `Arrived` 就不能继续下一步。

默认速度如下：

```text
自由空间、回检测位、花篮巡检：20 deg/s
靠近物体、靠近花篮释放位： 8 deg/s
发布频率：                    100 Hz
```

`TL_SERVOJ_POINTS=0` 表示根据关节距离自动计算轨迹时间。不要再使用固定 1000 点，否则每一段都会固定运行约 10 秒。

## 2. 六个花篮的数据要求

编号 1-6 对应 ArUco ID 0-5。每个花篮必须独立采集三组数据：

1. `search_pose`：相机能稳定、完整看见该花篮码的观察位。
2. `basket_approach`：灵巧手位于篮口上方、不会碰撞架子和篮沿的接近位。
3. `basket_release`：掌心位于篮内预定释放中心的俯视放置位。

六个花篮共需要 18 个记录。数据保存在：

```text
~/tl_mock_pick_place/basket_aruco.yaml
```

运行示教：

```bash
cd ~/Desktop/TL_Dobot
source /opt/ros/humble/setup.bash
source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
bash scripts/teach_aruco_basket.sh
```

每个花篮执行 `B -> A -> P -> R`，然后手动平移到下一个花篮。终端会显示：

```text
Six-basket teach progress: complete=[...] (x/6), incomplete=[...]
```

只有显示 `complete=[1, 2, 3, 4, 5, 6] (6/6)` 才能启动无固定编号的全自动模式。

## 3. 假手完整测试

第一次测试不要启用灵巧手：

```bash
cd ~/Desktop/TL_Dobot
source /opt/ros/humble/setup.bash
source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash

TL_USE_HAND=0 \
TL_SERVOJ_SPEED_DEG_S=20 \
TL_SERVOJ_PRECISION_SPEED_DEG_S=8 \
bash scripts/run_dynamic_aruco_basket_test.sh
```

窗口操作：

1. 程序先回到固定俯视抓取检测位。
2. 点击待抓物体中心。
3. 按 `C` 计算俯视抓取接近位和抓取位，检查终端无 IK 错误。
4. 按 `G` 执行完整流程；不需要先按 `B`，也不需要输入花篮编号。
5. 程序先在六个已示教观察位中自主巡检，选择稳定可见的已示教 ArUco 码，并重新计算该花篮当前的接近位和释放位。
6. 程序回到抓取检测位，然后执行俯视接近、俯视抓取、抬升、经检测位安全转运、俯视放置和返回。
7. 按 `Q` 退出。

`B` 仅用于单独测试六篮自主定位，不执行抓取；`M` 仅用于测试点击目标的抓取位，不执行放置。数字 `1-6` 仍保留为调试时的人工覆盖，正常自动运行不使用。

## 4. 自主花篮选择规则

未设置 `TL_BASKET_NUMBER` 时，程序不会固定花篮编号。它按当前关节位置到六个示教观察位的距离排序，依次巡检；在画面中发现满足多帧稳定性要求的已示教码后，选择最靠近画面中心的码，再移动到该码自己的观察位做精定位。

这意味着六个码不需要同时出现在一张图里。单个眼在手上相机可以通过移动机械臂完成六个花篮的巡视。

ArUco 只能确定花篮编号和位姿，不能判断花篮是否为空。如果要求“只投放空篮”，还需要增加篮内占用检测；当前自动规则是选择第一个巡检到的稳定、完整、已示教花篮码。

## 5. 灵巧手上线顺序

假手至少连续完成 10 次无碰撞循环后，再安装灵巧手。安装后先保持 `TL_USE_HAND=0`，重新确认：

- 相机支架、灵巧手和 ArUco 相对位置没有变化。
- 六个 `basket_approach` 与 `basket_release` 对真实手均留有足够间隙。
- 电缆在六个观察位和转运路径中不会拉扯。
- `/tl_hand/stop`、物理急停和手部保护参数已逐项验证。

完成保护验证后才使用：

先在另一个终端启动手部保护节点，并保持该终端运行：

```bash
cd ~/Desktop/TL_Dobot
source /opt/ros/humble/setup.bash
source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
export PYTHONPATH="$PWD/src/tl_hand_deploy:${PYTHONPATH:-}"
python3 -m tl_hand_deploy.roh_hand_node \
  --ros-args \
  --params-file "$PWD/src/tl_hand_deploy/config/hand.yaml"
```

`limits_verified` 默认是 `false`。只有逐项核对真实灵巧手的张开位置、闭合位置、速度、电流和力限制后，才能在 `src/tl_hand_deploy/config/hand.yaml` 中改为 `true`。不要为了跑通流程直接绕过这一检查。

然后在测试终端运行：

```bash
TL_USE_HAND=1 \
TL_SERVOJ_SPEED_DEG_S=15 \
TL_SERVOJ_PRECISION_SPEED_DEG_S=5 \
bash scripts/run_dynamic_aruco_basket_test.sh
```

真实手首轮使用较低速度。确认 10 次稳定后，自由空间速度可逐步从 15 调到 20；精细速度建议保持 5-8，不要超过自由空间速度。

## 6. 提速参数

```bash
# 保守
TL_SERVOJ_SPEED_DEG_S=15 TL_SERVOJ_PRECISION_SPEED_DEG_S=5

# 当前推荐
TL_SERVOJ_SPEED_DEG_S=20 TL_SERVOJ_PRECISION_SPEED_DEG_S=8

# 假手验证后才可尝试，不用于真实手首轮
TL_SERVOJ_SPEED_DEG_S=25 TL_SERVOJ_PRECISION_SPEED_DEG_S=8
```

允许范围为 `(0, 45] deg/s`。出现抖动时先检查是否有多个 ServoJ 发布者，再把自由空间速度降到 15；不要通过降低 100 Hz 发布频率来解决。

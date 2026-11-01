# U 型叉花篮取放流程

## 设计

该流程不使用夹爪、灵巧手或安全代理节点，直接使用天链驱动的 ServoJ。D435 + ArUco 只负责确定花篮原位和运行时的花篮选择；U 型叉的插入、抬升和转运位由人工拖动示教。

每个花篮需要三个位姿：

- `fork_approach`：叉子在花篮外侧的安全接近位。
- `fork_insert`：叉子已经插入花篮、但还没有抬升的位姿。
- `fork_lift`：叉子抬升后花篮已经脱离支撑的承载位。

所有花篮共用一个 `fork_transfer_pose`，它是带着花篮移动到的指定转运位置。

## 示教

```bash
cd ~/Desktop/TL_Dobot
source /opt/ros/humble/setup.bash
source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
bash scripts/teach_aruco_basket.sh
```

窗口操作：

1. 把 D435 和机械臂移动到能看清一个 ArUco 的搜索位，按 `B` 选中花篮。
2. 按住拖动按钮，把 U 型叉移动到花篮外侧安全位，松开后按 `F`。
3. 拖动叉子插入花篮，保持不抬升，按 `I`。
4. 拖动叉子向上抬升，确认花篮完全脱离支撑，按 `L`。
5. 只需第一次示教：带着空花篮或等质量替代物移动到指定转运位，按 `T`。
6. 按 `R` 回到该花篮搜索位，然后平移机械臂到下一个花篮，重复 `B/F/I/L`。
7. 六个花篮和一个转运位都完成后按 `Q`。

`A/P` 仍保留给旧的普通花篮放置流程；U 型叉流程使用 `F/I/L/T`。

## 运行

```bash
bash scripts/run_fork_basket_mqtt_test.sh
```

手动测试：`B` 定位、`K` 取篮、`P` 放回、`1..6` 选择编号、`R` 回观察位、`Q` 退出。

MQTT 测试：

```json
{"cmd":"trim_pick","targetId":"1"}
```

执行顺序为：`fork_approach -> fork_insert -> fork_lift -> fork_transfer`，成功后返回 `retCode: 0`，程序等待下一条命令。

```json
{"cmd":"trim_place","targetId":"1"}
```

执行顺序为：保存的原花篮 `fork_lift -> fork_insert -> fork_approach -> observation`，花篮回到原叉取位置后返回 `retCode: 0`。

`targetId` 可以直接使用 `1..6`，也可以用 `TL_MQTT_TARGET_MAP` 映射到花篮编号。取篮成功后不重新识别花篮，因此即使花篮已被移走，也能使用取篮时保存的原始 ArUco 位姿放回原位。

## 重要限制

- U 型叉必须在 `fork_insert` 和 `fork_lift` 之间真实承载花篮，不能只用空叉示教后直接承重。
- `fork_transfer_pose` 必须留出花篮宽度、叉子长度和线缆余量；ServoJ 是关节插值，转运位之间不要穿过工作台或其他花篮。
- 当前版本没有保护节点。测试时必须降低速度、低载荷、空旷环境，并始终保持物理急停可用。
- 修改花篮安装位置、ArUco 贴纸位置、叉子安装位置或相机支架后，需要重新示教受影响的位姿。

1.启动天链机械臂驱动（上电操作）
    source /opt/ros/humble/setup.bash
    source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
    ros2 launch tl_driver tl_tcb605_driver.launch.py

2.运行简易抓取测试程序（鼠标点击抓取，带钢性假手）
    cd TL_Dobot/
    chmod +x scripts/run_mock_hand_side_grasp_test.sh
    ./scripts/run_mock_hand_side_grasp_test.sh

3.到指定俯视抓取姿态位置抓取测试程序（鼠标点击抓取，带钢性假手）
    cd ~/Desktop/TL_Dobot
    sed -i 's|exec "$project_dir/scripts/run_no_gripper_hover_test.sh"|exec bash "$project_dir/scripts/run_no_gripper_hover_test.sh"|' \
    scripts/run_mock_hand_side_grasp_test.sh
    chmod +x scripts/*.sh
    bash scripts/run_mock_hand_side_grasp_test.sh

4.跑假手测试（鼠标点击抓取，带钢性假手。鼠标点击确定抓取位置，c计算，m移动，g下降到指定高度，r返回。）
    timeout 5s ros2 topic list | grep set_servoj_pos   # 驱动起来后应能看到
    bash ~/Desktop/TL_Dobot/scripts/run_mock_hand_side_grasp_test.sh

5、完整流程（鼠标点击版本）
    机械臂上电
    source /opt/ros/humble/setup.bash
    source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
    ros2 launch tl_driver tl_tcb605_driver.launch.py
    灵巧手抓取
    source /opt/ros/humble/setup.bash
    source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
    cd ~/Desktop/TL_Dobot
    TL_USE_HAND=1 TL_PLACE_ONLY=1 bash scripts/run_dynamic_aruco_basket_test.sh

6、MQTT版本
    机械臂上电
    source /opt/ros/humble/setup.bash
    source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
    ros2 launch tl_driver tl_tcb605_driver.launch.py

    MQTT抓取
    cd ~/Desktop/TL_Dobot
    TL_MQTT_HOST=127.0.0.1 \
    TL_YOLO_MODEL=~/yolo_models/best.pt \
    TL_FIXED_PICK_BASE_MM="380.37 74.1 52.31" \
    bash scripts/run_yolo_mqtt_pick_place_test.sh

    MQTT命令测试
    mosquitto_pub -h 127.0.0.1 -t tl/robot/command -q 1 \
    -m '{"cmd":"seed","targetId":"A1-01"}'

7、MQTT版本
    # 终端 1 —— 驱动（常驻，单独起）
    source /opt/ros/humble/setup.bash
    source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
    ros2 launch tl_driver tl_tcb605_driver.launch.py

    # 终端 2 —— 控制节点（分开起）
    bash ~/Desktop/TL_Handback/scripts/run_fixed_pick_mqtt.sh
    # 看到: tl_hand package prefix: .../TL_Handback/install/tl_hand
    # 然后: MQTT connected and subscribed

    # 终端 3 —— 发命令
    python3 ~/Desktop/TL_Dobot/scripts/mqtt_seed_controller.py
    >>> A1-01

8、MQTT提速版本
    # 终端 1：驱动
    source /opt/ros/humble/setup.bash
    source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
    ros2 launch tl_driver tl_tcb605_driver.launch.py

    # 终端 2：抓放节点
    bash ~/Desktop/TL_Handback/scripts/run_fixed_pick_mqtt.sh

    # 终端 3：发命令
    python3 ~/Desktop/TL_Dobot/scripts/mqtt_seed_controller.py
    >>> A1-01
    bash ~/Desktop/TL_Dobot/scripts/run_roh_hand_test.sh

9、灵巧手控制程序
    # 驱动开着的前提下
    bash ~/Desktop/TL_Dobot/scripts/run_roh_hand_test.sh


    改好了。现在只示教了一部分也能启动——启动时对未示教的花篮只打印警告，不拦截：


    text
    WARNING: baskets [7, 8, ..., 24] are not fully taught yet;
    MQTT seed works only for the taught baskets
    已示教的花篮（1~6 号 = A1 排）可以正常测试，没示教的发命令会提示 "not fully taught" 拒绝。

    同步 + 重跑：


    bash
    scp src/tl_hand/tl_hand/mock_pick_place.py \
        nvidia@<jetson-ip>:~/Desktop/TL_Handback/src/tl_hand/tl_hand/

    bash
    cd ~/Desktop/TL_Handback && colcon build --packages-select tl_hand
    bash ~/Desktop/TL_Handback/scripts/run_fixed_pick_mqtt.sh
    启动后发 A1-01 测试完整流程。

10、直接测试。三个终端：

    终端 1：驱动（如果还开着就跳过）

    bash
    ros2 launch tl_driver tl_tcb605_driver.launch.py
    终端 2：灵巧手程序

    bash
    bash ~/Desktop/TL_Handback/scripts/run_fixed_pick_mqtt.sh
    终端 3：发指令

    bash
    python3 ~/Desktop/TL_Dobot/scripts/mqtt_seed_controller.py --target A1-01


    bash ~/Desktop/TL_Dobot/scripts/teach_aruco_basket.sh
    # 3号：B(手稳住1秒) → A → P → R
    # 4号：B(手稳住1秒) → A → P → R → Q

    mosquitto_pub -h 127.0.0.1 -t node/arm/01/down -m '{"cmd":"trim_pick","targetId":3}'mosquitto_pub -h 127.0.0.1 -t node/arm/01/down -m '{"cmd":"trim_place","targetId":3}'


11、
    叉子上没花篮（空叉）
    bash
    pkill -f fork_trim_mqtt
    rm ~/tl_fork_teach/active_transfer.yaml
    bash ~/Desktop/TL_Handback/scripts/run_fork_trim_mqtt.sh

12、发叉取命令（重点测新增的位置1/位置2）：
    bash
    mosquitto_pub -h 116.205.245.172 -u shapandevice -P 'qwer1234!!' \
    -t node/arm/01/down -m '{"cmd":"trim_pick","targetId":"A1-03"}' -q 1

    mosquitto_pub -h 116.205.245.172 -u shapandevice -P 'qwer1234!!' \
  -t node/arm/01/down -m '{"cmd":"trim_place","targetId":"A1-03"}' -q 1



新版种子节点测试流程：
    1. 确认没在跑叉子节点，然后启动（两个终端）：
    bash
    pkill -f "fork_trim_mqtt|mock_pick_place"

    bash
    # 终端 A：驱动
    ros2 launch tl_driver tl_tcb605_driver.launch.py

    bash
    # 终端 B：种子节点（灵巧手模式）
    bash ~/Desktop/TL_Handback/scripts/run_fixed_pick_mqtt.sh
    启动日志应显示：MQTT connected and subscribed: node/arm/01/down，且 broker=116.205.245.172（不是 127.0.0.1 就对了）

    4. 测试（终端 C）：


    bash
    # 订阅反馈
    mosquitto_sub -h 116.205.245.172 -u shapandevice -P 'qwer1234!!' -t node/arm/01/up -q 1 -v -C 2

    # 发放种子命令
    mosquitto_pub -h 116.205.245.172 -u shapandevice -P 'qwer1234!!' \
    -t node/arm/01/down -m '{"cmd":"seed","targetId":"A1-03"}' -q 1
    预期反馈（重点验证新版协议）：


    text
    {"cmd":"seed","targetId":"A1-03","retCode":1}   ← 先发：响应命令开始动作
    {"cmd":"seed","targetId":"A1-03","retCode":0}   ← 终态：放置完成



    # 终端 B：Ctrl+C 杀掉旧节点，重新启动
    bash ~/Desktop/TL_Handback/scripts/run_fixed_pick_mqtt_headless.sh




    bash ~/Desktop/TL/scripts/run_seed_mqtt.sh

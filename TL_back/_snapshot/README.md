1.启动天链机械臂驱动（上电操作）
    source /opt/ros/humble/setup.bash
    source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
    ros2 launch tl_driver tl_tcb605_driver.launch.py

2.运行简易抓取测试程序（鼠标点击抓取）
    cd ~/Desktop/TL_Dobot/src/tl_hand_deploy
    source /opt/ros/humble/setup.bash
    source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
    export PYTHONPATH="$PWD:${PYTHONPATH:-}"

    python3 -m tl_hand_deploy.overhead_test \
    --calibration ~/tl_handeye_data/session_20260808_022900/calibration/handeye_candidate_rejected.yaml \
    --allow-rejected \
    --hover-mm 150 \
    --interp-points 500 \
    --rate-hz 100 \
    --timeout-sec 60 \
    --execute

3.视觉精确标定（鼠标点击抓取）
    python3 -m tl_hand_deploy.overhead_test \
    --calibration ~/tl_handeye_data/session_20260808_022900/calibration/handeye_candidate_rejected.yaml \
    --allow-rejected \
    --hover-mm 150 \
    --pointer-length-mm 140 \
    --min-tip-clearance-mm 5 \
    --interp-points 500 \
    --rate-hz 100 \
    --timeout-sec 60 \
    --execute
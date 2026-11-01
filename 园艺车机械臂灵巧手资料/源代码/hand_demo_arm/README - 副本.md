 两个终端：

    终端1（tl_driver 保持运行）：
    bash
    source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
    ros2 launch tl_driver tl_driver.launch.py arm_type:=tcb605


    终端2：
    bash
    cd ~/Desktop/hand_demo_arm
    source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
    python3 vision_grasp_demo.py


    流程：z 回零 → 放物体到 D435 前 → g 抓取
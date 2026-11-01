from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os
import sys


def generate_launch_description():
    share = get_package_share_directory("tl_hand_deploy")
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    sys.executable,
                    "-m",
                    "tl_hand_deploy.safe_arm_node",
                    "--ros-args",
                    "-r",
                    "__node:=safe_arm_node",
                    "--params-file",
                    os.path.join(share, "config", "arm_safety.yaml"),
                ],
                output="screen",
            )
        ]
    )

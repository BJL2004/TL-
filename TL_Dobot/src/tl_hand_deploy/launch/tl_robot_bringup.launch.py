"""One-command robot bringup: TL driver (+ optional ROH hand guard node).

Usage:
  ros2 launch tl_hand_deploy tl_robot_bringup.launch.py
  ros2 launch tl_hand_deploy tl_robot_bringup.launch.py with_hand:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    with_hand = LaunchConfiguration("with_hand")

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("tl_driver"),
                "launch",
                "tl_tcb605_driver.launch.py",
            )
        )
    )

    hand_guard_node = Node(
        package="tl_hand_deploy",
        executable="tl_hand_guard",
        name="roh_hand_guard",
        output="screen",
        condition=IfCondition(with_hand),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "with_hand",
                default_value="false",
                description="start the ROH hand guard node (/tl_hand services)",
            ),
            driver_launch,
            hand_guard_node,
        ]
    )

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory("tl_hand_deploy")
    return LaunchDescription(
        [
            Node(
                package="tl_hand_deploy",
                executable="tl_hand_guard",
                name="roh_hand_node",
                output="screen",
                parameters=[os.path.join(share, "config", "hand.yaml")],
            )
        ]
    )

import os
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("tl_example")

    executable = "queue_trajectory_playback"
    lib_exec = os.path.join(pkg_share, "..", "..", "lib", "tl_example", executable)
    bin_exec = os.path.join(pkg_share, "..", "..", "bin", executable)

    if os.path.exists(lib_exec) or os.path.exists(bin_exec):
        node = Node(
            package="tl_example",
            executable=executable,
            name=executable,
            output="screen",
        )
        return [node]
    else:
        raise FileNotFoundError(
            f"Could not find {executable} executable. " f"Checked {lib_exec} and {bin_exec}"
        )


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])

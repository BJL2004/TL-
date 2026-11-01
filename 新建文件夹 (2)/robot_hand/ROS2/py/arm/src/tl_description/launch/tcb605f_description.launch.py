import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('tl_description')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Use joint_state_publisher_gui if true',
            choices=['true', 'false'],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'tl_description.launch.py')
            ),
            launch_arguments={
                'arm_type': 'tcb605f',
                'use_sim': LaunchConfiguration('use_sim'),
            }.items(),
        ),
    ])

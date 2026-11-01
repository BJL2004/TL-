import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('tl_driver')

    config_path = os.path.join(
        pkg_share,
        'config',
        'tl_tcb605f_config.yaml'
    )

    # Python entry_points 安装到 bin/ 而非 lib/<pkg>/，
    # 因此需要检测实际路径后选择正确的启动方式
    lib_exec = os.path.join(pkg_share, '..', '..', 'lib', 'tl_driver', 'tl_driver_node')
    bin_exec = os.path.join(pkg_share, '..', '..', 'bin', 'tl_driver_node')

    if os.path.exists(lib_exec):
        return LaunchDescription([
            Node(
                package='tl_driver',
                executable='tl_driver_node',
                name='tl_driver',
                output='screen',
                parameters=[config_path]
            )
        ])
    elif os.path.exists(bin_exec):
        return LaunchDescription([
            ExecuteProcess(
                cmd=[bin_exec, '--ros-args', '--params-file', config_path],
                output='screen'
            )
        ])
    else:
        raise FileNotFoundError(
            f"Could not find tl_driver executable. Checked {lib_exec} and {bin_exec}"
        )

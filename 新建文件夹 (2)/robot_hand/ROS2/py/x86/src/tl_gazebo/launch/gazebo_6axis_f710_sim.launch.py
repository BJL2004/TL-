"""天链机械臂 6 轴 F710 遥操作仿真 — 通用启动文件。

用法：
  ros2 launch tl_gazebo gazebo_6axis_f710_sim.launch.py arm_type:=tcb610
"""

import os
from typing import List

from launch import LaunchDescription, SomeActionsType
from launch.actions import ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory


def _launch_actions(context, *args, **kwargs):
    """在运行时解析 arm_type 并生成实际的启动动作。"""
    arm_type_str = LaunchConfiguration("arm_type").perform(context)

    package_name = "tl_gazebo"

    tl_description_share = get_package_share_directory("tl_description")

    # ---- Gazebo 模型路径设置 ----
    gazebo_model_root = os.path.expanduser("~/.gazebo/tl_robot_models")
    gazebo_tl_description_model = os.path.join(gazebo_model_root, "tl_description")
    gazebo_tl_description_meshes = os.path.join(gazebo_tl_description_model, "meshes")

    os.makedirs(gazebo_tl_description_model, exist_ok=True)

    real_meshes_dir = os.path.join(tl_description_share, "meshes")

    if os.path.lexists(gazebo_tl_description_meshes):
        if os.path.islink(gazebo_tl_description_meshes):
            if os.path.realpath(gazebo_tl_description_meshes) != os.path.realpath(real_meshes_dir):
                os.unlink(gazebo_tl_description_meshes)
        else:
            raise RuntimeError(
                f"{gazebo_tl_description_meshes} already exists and is not a symlink. "
                f"Please remove it manually."
            )
    if not os.path.lexists(gazebo_tl_description_meshes):
        os.symlink(real_meshes_dir, gazebo_tl_description_meshes)

    # model.config / dummy.sdf
    model_config_path = os.path.join(gazebo_tl_description_model, "model.config")
    dummy_sdf_path = os.path.join(gazebo_tl_description_model, "dummy.sdf")
    if not os.path.exists(model_config_path):
        with open(model_config_path, "w") as f:
            f.write(
                '<?xml version="1.0"?><model><name>tl_description</name>'
                '<version>1.0</version><sdf version="1.6">dummy.sdf</sdf>'
                "<author><name>tl</name><email>none@example.com</email></author>"
                "<description>Mesh resource package for TL robot.</description></model>"
            )
    if not os.path.exists(dummy_sdf_path):
        with open(dummy_sdf_path, "w") as f:
            f.write(
                '<?xml version="1.0"?><sdf version="1.6">'
                '<model name="tl_description"><static>true</static>'
                '<link name="dummy_link"/></model></sdf>'
            )

    # GAZEBO_MODEL_PATH
    bad_model_path = os.path.dirname(tl_description_share)
    existing = os.environ.get("GAZEBO_MODEL_PATH", "")
    system_models = "/usr/share/gazebo-11/models"
    model_paths = [gazebo_model_root]
    if os.path.exists(system_models):
        model_paths.append(system_models)
    for p in existing.split(":"):
        if not p:
            continue
        if os.path.realpath(p) == os.path.realpath(bad_model_path):
            continue
        if p not in model_paths:
            model_paths.append(p)

    set_gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH", value=":".join(model_paths)
    )
    set_gazebo_model_database_uri = SetEnvironmentVariable(
        name="GAZEBO_MODEL_DATABASE_URI", value=""
    )

    # ---- 构建 robot_description：读取 URDF + 附加 Gazebo 配置 ----
    tl_description_share_path = get_package_share_directory("tl_description")
    urdf_path = os.path.join(tl_description_share_path, "urdf", f"{arm_type_str}.urdf")
    with open(urdf_path, "r") as f:
        urdf_content = f.read()
    # 去掉原 <robot> 包装，插入 Gazebo 特定内容
    urdf_inner = urdf_content.split("<robot", 1)[1].split(">", 1)[1]
    urdf_inner = urdf_inner.rsplit("</robot>", 1)[0]

    controller_yaml = os.path.join(
        get_package_share_directory(package_name), "config", "ros2_controllers_f710_sim_6axis.yaml"
    )

    robot_description_xml = f"""<?xml version="1.0"?>
<robot name="tl_{arm_type_str}_f710_sim">
{urdf_inner}
  <link name="world"/>
  <joint name="fixed" type="fixed">
    <parent link="world"/>
    <child link="link0"/>
  </joint>
  <gazebo reference="link0">
    <material>Gazebo/White</material>
    <self_collide>false</self_collide>
    <gravity>false</gravity>
  </gazebo>
  <gazebo reference="link1">
    <material>Gazebo/White</material>
    <self_collide>false</self_collide>
    <gravity>false</gravity>
  </gazebo>
  <gazebo reference="link2">
    <material>Gazebo/White</material>
    <gravity>false</gravity>
  </gazebo>
  <gazebo reference="link3">
    <material>Gazebo/White</material>
    <gravity>false</gravity>
  </gazebo>
  <gazebo reference="link4">
    <material>Gazebo/White</material>
    <gravity>false</gravity>
  </gazebo>
  <gazebo reference="link5">
    <material>Gazebo/White</material>
    <gravity>false</gravity>
  </gazebo>
  <gazebo reference="link6">
    <material>Gazebo/White</material>
    <gravity>false</gravity>
  </gazebo>
  <gazebo>
    <is_static>true</is_static>
    <self_collide>true</self_collide>
  </gazebo>
  <ros2_control name="GazeboSystem" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint3">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint4">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint5">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint6">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>{controller_yaml}</parameters>
      <robot_param>robot_description</robot_param>
      <robot_param_node>robot_state_publisher</robot_param_node>
    </plugin>
  </gazebo>
</robot>"""

    params = {"robot_description": robot_description_xml}

    # ---- 启动动作 ----
    actions: List[SomeActionsType] = []
    actions.append(set_gazebo_model_database_uri)
    actions.append(set_gazebo_model_path)

    gazebo_default_world = "/usr/share/gazebo-11/worlds/empty.world"
    if not os.path.exists(gazebo_default_world):
        raise RuntimeError(f"Gazebo default world not found: {gazebo_default_world}")

    actions.append(
        ExecuteProcess(
            cmd=[
                "gazebo",
                "--verbose",
                gazebo_default_world,
                "-s",
                "libgazebo_ros_init.so",
                "-s",
                "libgazebo_ros_factory.so",
            ],
            output="screen",
        )
    )

    actions.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"use_sim_time": True}, params, {"publish_frequency": 15.0}],
            output="screen",
        )
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", f"tl_{arm_type_str}"],
        output="screen",
    )
    actions.append(spawn_entity)

    load_jsc = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )
    load_jpc = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "tcb_group_position_controller",
        ],
        output="screen",
    )

    actions.append(
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=spawn_entity, on_exit=[load_jsc])
        )
    )
    actions.append(
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=load_jsc, on_exit=[load_jpc])
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "arm_type",
                default_value="tcb605",
                description="机械臂型号（6 轴），如 tcb605、tcb610",
            ),
            OpaqueFunction(function=_launch_actions),
        ]
    )

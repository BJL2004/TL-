"""TL hand pick-place launch: starts the TL driver + MQTT/YOLO pick-place node.

Usage:
  ros2 launch tl_hand tl_hand_pick_place.launch.py
  ros2 launch tl_hand tl_hand_pick_place.launch.py use_hand:=true
  ros2 launch tl_hand tl_hand_pick_place.launch.py mqtt_host:=192.168.1.100
  ros2 launch tl_hand tl_hand_pick_place.launch.py yolo_model:=/home/nvidia/yolo_models/best.pt
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_pick_place_node(context, *args, **kwargs):
    use_hand = LaunchConfiguration("use_hand").perform(context)
    mqtt_host = LaunchConfiguration("mqtt_host").perform(context)
    mqtt_port = LaunchConfiguration("mqtt_port").perform(context)

    yolo_conf = LaunchConfiguration("yolo_conf").perform(context)
    yolo_imgsz = LaunchConfiguration("yolo_imgsz").perform(context)
    calibration = os.path.expanduser(LaunchConfiguration("calibration").perform(context))
    place_file = os.path.expanduser(LaunchConfiguration("place_file").perform(context))
    basket_aruco_file = os.path.expanduser(LaunchConfiguration("basket_aruco_file").perform(context))
    yolo_model = os.path.expanduser(LaunchConfiguration("yolo_model").perform(context))
    fixed_pick_base_mm = LaunchConfiguration("fixed_pick_base_mm").perform(context)
    observation_joints = LaunchConfiguration("observation_joints").perform(context)
    home_joint_deg = LaunchConfiguration("home_joint_deg").perform(context)

    arguments = [
        "--calibration", calibration,
        "--place-file", place_file,
        "--basket-aruco-file", basket_aruco_file,
        "--base-xy-bias-mm", "-53.2", "-34.2",
        "--grasp-rpy-rad", "-2.043", "1.471", "-1.025",
        "--mock-grasp-offset-mm", "-6.3", "34.0", "42.5",
        "--pick-approach-clearance-mm", "130",
        "--pick-grasp-clearance-mm", "50",
        "--final-grasp-z-mm", "79.62",
        "--basket-motion-mode", "translation",
        "--max-basket-shift-mm", "200",
        "--interp-points", "0",
        "--rate-hz", "100",
        "--servoj-speed-deg-s", "20",
        "--servoj-precision-speed-deg-s", "8",
        "--servoj-min-duration-sec", "1.0",
        "--auto-lift-to-reachable",
        "--max-auto-lift-mm", "200",
        "--mqtt-enabled",
        "--mqtt-host", mqtt_host,
        "--mqtt-port", mqtt_port,
        "--mqtt-command-topic", "tl/robot/command",
        "--mqtt-feedback-topic", "tl/robot/feedback",
        "--mqtt-client-id", "tl-tcb605-arm",
        "--mqtt-keepalive", "60",
        "--yolo-model", yolo_model,
        "--yolo-conf", yolo_conf,
        "--yolo-imgsz", yolo_imgsz,
        "--timeout-sec", "90",
        "--max-joint-delta-deg", "220",
        "--allow-rejected",
        "--execute",
    ]

    if fixed_pick_base_mm.strip():
        arguments += ["--fixed-pick-base-mm"] + fixed_pick_base_mm.split()

    if observation_joints.strip():
        arguments += ["--observation-joints"] + observation_joints.split()

    if home_joint_deg.strip():
        arguments += ["--home-joint-deg"] + home_joint_deg.split()

    if use_hand.lower() in ("true", "1", "yes"):
        arguments += [
            "--use-hand",
            "--hand-grasp-positions", "35000", "35000", "35000", "35000", "35000", "25000",
            "--hand-open-positions", "0", "0", "0", "0", "0", "0",
            "--hand-level5-ratio", "0.85",
        ]

    return [
        Node(
            package="tl_hand",
            executable="tl_mock_pick_place",
            name="tl_pick_place",
            output="screen",
            arguments=arguments,
        )
    ]


def generate_launch_description():
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("tl_driver"),
                "launch",
                "tl_tcb605_driver.launch.py",
            )
        )
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(name="RCUTILS_LOGGING_MIN_SEVERITY", value="WARN"),
            DeclareLaunchArgument(
                "use_hand",
                default_value="false",
                description="enable the ROH-LiteS001 hand (true/false)",
            ),
            DeclareLaunchArgument(
                "mqtt_host",
                default_value="127.0.0.1",
                description="MQTT broker host",
            ),
            DeclareLaunchArgument(
                "mqtt_port",
                default_value="1883",
                description="MQTT broker port",
            ),
            DeclareLaunchArgument(
                "yolo_model",
                default_value="~/yolo_models/best.pt",
                description="YOLO model weights path",
            ),
            DeclareLaunchArgument(
                "yolo_conf",
                default_value="0.5",
                description="YOLO confidence threshold",
            ),
            DeclareLaunchArgument(
                "yolo_imgsz",
                default_value="640",
                description="YOLO inference image size",
            ),
            DeclareLaunchArgument(
                "calibration",
                default_value="~/tl_handeye_data/best/handeye_best.yaml",
                description="hand-eye calibration YAML path",
            ),
            DeclareLaunchArgument(
                "place_file",
                default_value="~/tl_mock_pick_place/place_pose.yaml",
                description="taught place pose YAML path",
            ),
            DeclareLaunchArgument(
                "basket_aruco_file",
                default_value="~/tl_mock_pick_place/basket_aruco.yaml",
                description="taught basket ArUco YAML path",
            ),
            DeclareLaunchArgument(
                "fixed_pick_base_mm",
                default_value="380.37 74.1 52.31",
                description="fixed pick point in base frame [x y z] mm",
            ),
            DeclareLaunchArgument(
                "observation_joints",
                default_value="-83.668 67.725 3.956 -2.101 -5.484 -1.431",
                description="observation pose joints [J1..J6] in degrees",
            ),
            DeclareLaunchArgument(
                "home_joint_deg",
                default_value="-1.521 -1.225 35.678 3.856 31.471 -3.533",
                description="home pose joints [J1..J6] in degrees",
            ),
            driver_launch,
            OpaqueFunction(function=_make_pick_place_node),
        ]
    )
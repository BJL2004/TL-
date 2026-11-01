"""TL hand pick-place launch: starts ONLY the MQTT fixed-point pick-place node.

No YOLO: the seed is picked at the configured fixed base point on MQTT command.
Start the TL driver separately in another terminal first:
  ros2 launch tl_driver tl_tcb605_driver.launch.py

Usage:
  ros2 launch tl_hand tl_hand_pick_place.launch.py use_hand:=true
  ros2 launch tl_hand tl_hand_pick_place.launch.py mqtt_host:=127.0.0.1
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_pick_place_node(context, *args, **kwargs):
    use_hand = LaunchConfiguration("use_hand").perform(context)
    mqtt_host = LaunchConfiguration("mqtt_host").perform(context)
    mqtt_port = LaunchConfiguration("mqtt_port").perform(context)
    mqtt_username = LaunchConfiguration("mqtt_username").perform(context)
    mqtt_password = LaunchConfiguration("mqtt_password").perform(context)

    calibration = os.path.expanduser(LaunchConfiguration("calibration").perform(context))
    place_file = os.path.expanduser(LaunchConfiguration("place_file").perform(context))
    basket_aruco_file = os.path.expanduser(LaunchConfiguration("basket_aruco_file").perform(context))
    fixed_pick_base_mm = LaunchConfiguration("fixed_pick_base_mm").perform(context)
    observation_joints = LaunchConfiguration("observation_joints").perform(context)
    home_joint_deg = LaunchConfiguration("home_joint_deg").perform(context)

    arguments = [
        "--calibration", calibration,
        "--place-file", place_file,
        "--basket-aruco-file", basket_aruco_file,
        "--base-xy-bias-mm", "-53.2", "-34.2",
        "--grasp-rpy-rad", "1.607", "1.337", "1.553",
        "--mock-grasp-offset-mm", "-6.3", "34.0", "42.5",
        "--pick-approach-clearance-mm", "130",
        "--pick-grasp-clearance-mm", "50",
        "--final-grasp-z-mm", "218.880",
        "--basket-motion-mode", "translation",
        "--max-basket-shift-mm", "200",
        "--max-place-reachability-shift-mm", "80",
        "--marker-sample-spread-mm", "60",
        "--interp-points", "0",
        "--rate-hz", "100",
        "--servoj-speed-deg-s", "40",
        "--servoj-precision-speed-deg-s", "20",
        "--servoj-min-duration-sec", "0.3",
        "--hand-settle-sec", "1.0",
        "--auto-lift-to-reachable",
        "--max-auto-lift-mm", "200",
        "--mqtt-enabled",
        "--mqtt-host", mqtt_host,
        "--mqtt-port", mqtt_port,
        "--mqtt-username", mqtt_username,
        "--mqtt-password", mqtt_password,
        "--mqtt-command-topic", "node/arm/01/down",
        "--mqtt-feedback-topic", "node/arm/01/up",
        "--mqtt-target-map", "",
        "--mqtt-client-id", "tl-tcb605-arm",
        "--mqtt-keepalive", "60",
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
            "--hand-hold-positions", "8000", "8000", "8000", "8000", "8000", "5000",
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
                default_value="116.205.245.172",
                description="MQTT broker host",
            ),
            DeclareLaunchArgument(
                "mqtt_port",
                default_value="1883",
                description="MQTT broker port",
            ),
            DeclareLaunchArgument(
                "mqtt_username",
                default_value="shapandevice",
                description="MQTT broker username",
            ),
            DeclareLaunchArgument(
                "mqtt_password",
                default_value="qwer1234!!",
                description="MQTT broker password",
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
                default_value="398.53 -193.69 211.63",
                description="fixed pick point in base frame [x y z] mm",
            ),
            DeclareLaunchArgument(
                "observation_joints",
                default_value="-23.246 -14.220 53.738 4.389 26.746 -3.532",
                description="observation pose joints [J1..J6] in degrees",
            ),
            DeclareLaunchArgument(
                "home_joint_deg",
                default_value="-23.246 -14.220 53.738 4.389 26.746 -3.532",
                description="home pose joints [J1..J6] in degrees",
            ),
            OpaqueFunction(function=_make_pick_place_node),
        ]
    )
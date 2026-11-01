"""Headless MQTT seed pick-place: same behavior as the windowed node, but
without any OpenCV window, mouse, or keyboard interaction.

Everything is driven by MQTT commands only:
  {"cmd":"seed","targetId":"A1-01"} -> pick at the fixed point, localize the
  target basket, place the seed, and return to the observation pose.

Run it with scripts/run_fixed_pick_mqtt_headless.sh (do NOT touch
run_fixed_pick_mqtt.sh, the windowed launcher).
"""

import sys

import numpy as np
import rclpy

from .mock_pick_place import MockPickPlaceNode, parse_args


class HeadlessPickPlaceNode(MockPickPlaceNode):
    """Pick-place node without a preview window or keyboard/mouse handling.

    Reuses all pick, basket-localization, placement, hand, and MQTT logic
    from the windowed node; only the run loop is replaced with a pure
    MQTT-driven loop.
    """

    def __init__(self, arguments, calibration_path, calibration, tool_camera):
        super().__init__(arguments, calibration_path, calibration, tool_camera)
        # No preview window: basket scan/localize previews are skipped because
        # every imshow call in the base class is guarded by self.window_name.
        self.window_name = None

    def run(self) -> None:
        self.wait_for_robot()
        if self.arguments.observation_joints is not None:
            self.detect_joint = self._pad_joint(self.arguments.observation_joints)
        self.move_to_configured_observation()
        self.start_camera()
        if self.mqtt_client is not None:
            self.mqtt_client.start()
        if self.arguments.use_hand:
            # Protective initial state: level-1 hold, same as the windowed node.
            try:
                self._hand_write_positions(
                    "hold level-1", self.arguments.hand_hold_positions
                )
            except RuntimeError as error:
                print(
                    f"WARNING: initial level-1 hold failed ({error}); "
                    "initialize the hand before starting a pick"
                )
        print(f"calibration={self.calibration_path}")
        print(
            f"base XY bias[mm]={np.round(self.arguments.base_xy_bias_mm, 2).tolist()}"
        )
        print("HEADLESS MQTT pick-place: no preview window; MQTT commands only.")
        print("Send: {\"cmd\":\"seed\",\"targetId\":\"A1-01\"} to node/arm/01/down")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            self._process_mqtt_commands()
            if self.mqtt_auto_pick_pending:
                self.mqtt_auto_pick_pending = False
                try:
                    self._auto_seed_pick_place()
                except RuntimeError as error:
                    print(f"Seed cycle failed: {error}")
                    self._mqtt_fail_pending(error)


def main(argv=None) -> int:
    arguments = parse_args(argv)
    from .overhead_test import load_calibration

    calibration_path, calibration, tool_camera = load_calibration(
        arguments.calibration, arguments.allow_rejected
    )
    rclpy.init(args=[value for value in argv or [] if value != "--ros-args"])
    node = HeadlessPickPlaceNode(
        arguments, calibration_path, calibration, tool_camera
    )
    try:
        node.run()
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

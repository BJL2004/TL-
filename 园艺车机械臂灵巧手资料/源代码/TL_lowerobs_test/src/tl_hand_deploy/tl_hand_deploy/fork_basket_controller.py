"""Direct ServoJ U-fork basket pickup, transfer, and return controller."""

from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
import yaml

from .basket_aruco import camera_parameters, detect_marker_transforms, draw_marker_overlay
from .mock_pick_place import MockPickPlaceNode, parse_args
from .mqtt_protocol import RET_FAILED, RET_OCCUPIED, RET_SUCCESS, MqttCommand
from .overhead_test import load_calibration


class ForkBasketNode(MockPickPlaceNode):
    """Reuse the existing ArUco localization, with fork-specific motion states."""

    @staticmethod
    def _load_basket_aruco(path: Path, basket_number: int | None) -> dict:
        config_path = path.expanduser().resolve()
        if not config_path.exists():
            raise RuntimeError(f"fork basket ArUco file is missing: {config_path}")
        data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
        records = data.get("markers", {}) or {}
        required = ("search_pose", "fork_approach", "fork_insert", "fork_lift")
        usable = []
        incomplete = []
        for marker_id in range(6):
            record = records.get(str(marker_id), records.get(marker_id, {})) or {}
            if all(name in record for name in required):
                usable.append(marker_id)
            else:
                incomplete.append(marker_id + 1)
        if not usable:
            raise RuntimeError(
                f"{config_path} has no complete fork record; teach F, I, L after B"
            )
        transfer = data.get("fork_transfer_pose") or {}
        if "joint" not in transfer or "tcp" not in transfer:
            raise RuntimeError(
                f"{config_path} has no fork_transfer_pose; teach the global T pose"
            )
        selected_id = None if basket_number is None else int(basket_number) - 1
        if selected_id is not None and selected_id not in usable:
            raise RuntimeError(
                f"basket number {basket_number} has no complete fork record; "
                f"available={[value + 1 for value in usable]}"
            )
        if selected_id is None and incomplete:
            raise RuntimeError(
                "automatic fork selection needs all six baskets taught with F/I/L; "
                f"incomplete numbers={incomplete}"
            )
        data["_path"] = str(config_path)
        data["_usable_ids"] = usable
        data["_configured_id"] = selected_id
        data["_selected_id"] = selected_id
        print(f"fork basket ArUco file={config_path}")
        print(f"fork-taught marker numbers={[value + 1 for value in usable]}")
        return data

    def __init__(self, arguments, calibration_path, calibration, tool_camera):
        if arguments.use_hand:
            raise RuntimeError("fork controller never uses a hand or protection node")
        super().__init__(arguments, calibration_path, calibration, tool_camera)
        self.held = False
        self.held_marker_id = None
        self.held_search_joint = None
        self.held_fork_joints = None
        self.transfer_joint = self._pad_joint(
            self.basket_aruco["fork_transfer_pose"]["joint"]
        )

    def _dynamic_pose_names(self) -> list[str]:
        return ["fork_approach", "fork_insert", "fork_lift"]

    def _resolve_command_basket(self, command: MqttCommand) -> None:
        basket_number = self.mqtt_target_map.get(command.target_id)
        if basket_number is None:
            try:
                parsed = int(command.target_id)
            except ValueError:
                parsed = None
            if parsed in range(1, 7):
                basket_number = parsed
        if basket_number is not None:
            self.select_basket_number(basket_number)

    def _pick_basket(self) -> None:
        if self.held:
            raise RuntimeError("a basket is already held; wait for trim_place")
        if not self.dynamic_place_joints:
            self.localize_basket()
        selected_id = self.basket_aruco["_selected_id"]
        record = self._basket_record(selected_id)
        self.held_marker_id = int(selected_id)
        self.held_search_joint = self._pad_joint(record["search_pose"]["joint"])
        self.held_fork_joints = {
            name: self._pad_joint(self.dynamic_place_joints[name])
            for name in self._dynamic_pose_names()
        }
        self._validate_joint(self.held_fork_joints["fork_approach"])
        self._validate_joint(self.held_fork_joints["fork_insert"])
        self._validate_joint(self.held_fork_joints["fork_lift"])
        self._validate_joint(self.transfer_joint)
        print("Fork pick 1/4: move to approach")
        self.move_joint(self.held_fork_joints["fork_approach"])
        print("Fork pick 2/4: insert the U-fork")
        self.move_joint(
            self.held_fork_joints["fork_insert"],
            speed_deg_s=self.arguments.servoj_precision_speed_deg_s,
        )
        print("Fork pick 3/4: lift until the basket clears the support")
        self.move_joint(
            self.held_fork_joints["fork_lift"],
            speed_deg_s=self.arguments.servoj_precision_speed_deg_s,
        )
        print("Fork pick 4/4: move to the taught transfer pose")
        self.move_joint(self.transfer_joint)
        self.held = True
        print(
            f"Basket number={self.held_marker_id + 1} is held at the transfer pose. "
            "Wait for MQTT trim_place."
        )

    def _place_basket(self) -> None:
        if not self.held or self.held_fork_joints is None:
            raise RuntimeError("no basket is currently held")
        print("Fork place 1/4: return to the original basket lift pose")
        self.move_joint(self.held_fork_joints["fork_lift"])
        print("Fork place 2/4: lower the basket onto its support")
        self.move_joint(
            self.held_fork_joints["fork_insert"],
            speed_deg_s=self.arguments.servoj_precision_speed_deg_s,
        )
        print("Fork place 3/4: retract the U-fork")
        self.move_joint(
            self.held_fork_joints["fork_approach"],
            speed_deg_s=self.arguments.servoj_precision_speed_deg_s,
        )
        print("Fork place 4/4: return to the configured observation pose")
        self.return_observation()
        self.held = False
        self.held_marker_id = None
        self.held_search_joint = None
        self.held_fork_joints = None
        self.dynamic_place_joints = {}
        self.dynamic_place_poses = {}
        self.basket_aruco["_selected_id"] = None
        print("Basket returned to its original fork position.")

    def _handle_mqtt_command(self, command: MqttCommand) -> None:
        if self.mqtt_client is None:
            return
        if command.name not in ("trim_pick", "trim_place"):
            self.mqtt_client.publish_feedback(command.name, RET_FAILED)
            print(f"Fork controller rejects unsupported command: {command.name}")
            return
        if self.mqtt_pending_command is not None:
            self.mqtt_client.publish_feedback(command.name, RET_OCCUPIED)
            print("Fork controller is busy with another MQTT command")
            return
        if command.name == "trim_pick" and self.held:
            self.mqtt_client.publish_feedback(command.name, RET_OCCUPIED)
            print("trim_pick rejected: a basket is already held")
            return
        if command.name == "trim_place" and not self.held:
            self.mqtt_client.publish_feedback(command.name, RET_FAILED)
            print("trim_place rejected: no basket is held")
            return
        self.mqtt_pending_command = command
        try:
            if command.name == "trim_pick":
                self._resolve_command_basket(command)
                self._pick_basket()
            else:
                self._place_basket()
        except RuntimeError as error:
            print(f"Fork command failed: {error}")
            self._mqtt_publish_pending(RET_FAILED)
            return
        self._mqtt_publish_pending(RET_SUCCESS)

    def _draw_preview(self, image, camera_matrix, distortion):
        marker_length_m = float(self.basket_aruco.get("marker_length_m", 0.06))
        transforms, corners, ids, _ = detect_marker_transforms(
            image,
            camera_matrix,
            distortion,
            marker_length_m,
            self.basket_aruco["_usable_ids"],
        )
        preview = draw_marker_overlay(
            image,
            corners,
            ids,
            transforms,
            camera_matrix,
            distortion,
            marker_length_m * 0.5,
        )
        visible = [marker_id + 1 for marker_id in sorted(transforms)]
        selected = self.basket_aruco["_selected_id"]
        state = "HELD" if self.held else "EMPTY"
        cv2.putText(
            preview,
            f"fork state={state} visible={visible} selected="
            f"{None if selected is None else selected + 1}",
            (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 2,
        )
        cv2.putText(
            preview,
            "B:localize K:pick P:place 1..6:select R:return Q:quit",
            (8, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2,
        )
        return preview

    def run(self) -> None:
        self.wait_for_robot()
        if self.arguments.observation_joints is not None:
            self.detect_joint = self._pad_joint(self.arguments.observation_joints)
        self.move_to_configured_observation()
        self.start_camera()
        if self.mqtt_client is not None:
            self.mqtt_client.start()
        camera_matrix, distortion = camera_parameters(self.color_intrinsics)
        print("Direct U-fork mode: no safety proxy, no hand interface, direct ServoJ.")
        print("Manual test: B localize, K pick, P place. MQTT: trim_pick / trim_place.")
        window_name = "TL U-fork basket controller"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            self._process_mqtt_commands()
            try:
                frames = self.align.process(self.pipeline.wait_for_frames(timeout_ms=2000))
            except RuntimeError as error:
                self.get_logger().warning(f"D435 frame timeout: {error}")
                continue
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            image = np.asanyarray(color_frame.get_data()).copy()
            cv2.imshow(window_name, self._draw_preview(image, camera_matrix, distortion))
            key = cv2.waitKey(1) & 0xFF
            try:
                if key in (ord("q"), 27):
                    break
                if key == ord("b"):
                    self.localize_basket()
                elif key == ord("k"):
                    self._pick_basket()
                elif key == ord("p"):
                    self._place_basket()
                elif key == ord("r"):
                    self.return_observation()
                elif ord("1") <= key <= ord("6"):
                    self.select_basket_number(key - ord("0"))
                elif key == ord("0"):
                    self.reset_live_basket_selection()
            except RuntimeError as error:
                print(f"Command rejected/stopped: {error}")


def main(argv=None):
    arguments = parse_args(argv)
    calibration_path, calibration, tool_camera = load_calibration(
        arguments.calibration, arguments.allow_rejected
    )
    rclpy.init()
    node = ForkBasketNode(arguments, calibration_path, calibration, tool_camera)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

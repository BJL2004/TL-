"""Standalone U-fork basket pick, carry, and return executor.

The fork keeps the basket on the entire time it is being carried. The only
release point is the original support position during the return sequence.
This module does not use the hand, gripper, MQTT, YOLO, or safety proxy.
"""

from datetime import datetime, timezone
import argparse
import math
from pathlib import Path
import tempfile
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from tl_ros2_interface.srv import CoordTransform

from .fork_basket_teach import (
    ForkBasketTeachNode,
    load_calibration,
)

# 修剪作业新增位姿（示教器读数：关节坐标 J 表 / 大地坐标 XYZABC 表）：
#   位置1：到达 T 之后的高位过渡点（绕 J1 回转，叉保持水平，花篮安全）
#   位置2：修剪作业低位（花篮放低供车端剪根；J5 大幅翻转，必须低速过渡）
TRIM_HOLD_1_JOINT_DEG = [-3.164, 5.379, -1.368, -2.153, -92.470, -1.274]
TRIM_HOLD_2_JOINT_DEG = [-3.051, -57.827, -34.822, 1.369, -178.869, 4.907]
TRIM_HOLD_1_TCP = [486.657, -48.233, 415.655, 0.440, 1.445, 2.637]
TRIM_HOLD_2_TCP = [495.720, -50.561, -8.761, 2.279, 1.469, 0.949]


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Standalone U-fork basket pick, temporary carry, and return"
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        default=Path("~/tl_handeye_data/best/handeye_best.yaml"),
    )
    parser.add_argument(
        "--teach-file",
        "--output",
        dest="teach_file",
        type=Path,
        default=Path("~/tl_fork_teach/fork_basket_teach.yaml"),
        help="teaching file; --output is kept as a compatibility alias",
    )
    parser.add_argument(
        "--state-file",
        type=Path,
        default=Path("~/tl_fork_teach/active_transfer.yaml"),
    )
    parser.add_argument("--marker-length-m", type=float, default=0.058)
    parser.add_argument("--marker-ids", type=int, nargs="+", default=list(range(192)))
    parser.add_argument("--sample-window", type=int, default=30)
    parser.add_argument("--minimum-samples", type=int, default=8)
    parser.add_argument("--camera-serial", default="")
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--servoj-speed-deg-s", type=float, default=8.0)
    parser.add_argument("--servoj-rate-hz", type=float, default=100.0)
    parser.add_argument("--servoj-min-duration-sec", type=float, default=1.0)
    parser.add_argument(
        "--trim-hold-speed-deg-s",
        type=float,
        default=12.0,
        help="speed for the trim hold legs (position 1 <-> position 2); lower "
        "keeps the basket from sliding off while J5 rotates",
    )
    parser.add_argument(
        "--trim-hold-segments",
        type=int,
        default=6,
        help="interpolated joint segments for the position 1 -> position 2 leg",
    )
    parser.add_argument(
        "--trim-lower-clearance-mm",
        type=float,
        default=150.0,
        help="Z clearance of the intermediate pose above trim position 2: the "
        "arm first reaches position 2's XY at this height, then lowers down",
    )
    parser.add_argument("--arrival-tolerance-deg", type=float, default=1.0)
    parser.add_argument("--timeout-sec", type=float, default=120.0)
    parser.add_argument("--max-joint-delta-deg", type=float, default=220.0)
    parser.add_argument("--cartesian-step-mm", type=float, default=15.0)
    parser.add_argument(
        "--carry-clearance-mm",
        type=float,
        default=80.0,
        help="extra Z clearance above the higher of lift and temporary poses",
    )
    parser.add_argument(
        "--attitude-step-deg",
        type=float,
        default=5.0,
        help="maximum attitude change per Cartesian waypoint while carrying "
        "the basket (slow rotation at high Z)",
    )
    parser.add_argument(
        "--max-basket-shift-mm",
        type=float,
        default=200.0,
        help="maximum allowed marker translation from the taught search pose",
    )
    parser.add_argument(
        "--carry-extra-mm",
        type=float,
        default=10.0,
        help="minimum Z clearance above max(lift, temporary) for the carry "
        "horizontal travel (the fork holds the basket suspended, so a "
        "modest value is safe; raise it if your basket is very tall)",
    )
    parser.add_argument(
        "--observation-joints",
        type=float,
        nargs=6,
        default=None,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="fixed observation (standby) joints; overrides the YAML "
        "observation_pose so a stray O keypress cannot poison the pose",
    )
    parser.add_argument("--allow-rejected", action="store_true")
    parser.add_argument(
        "--execute",
        action="store_true",
        help="required to send robot motion; without it the program refuses P/R",
    )
    # MQTT options used by the trim command node (fork_trim_mqtt); the
    # keyboard cycle ignores them.
    parser.add_argument("--mqtt-enabled", action="store_true")
    parser.add_argument("--mqtt-host", default="127.0.0.1")
    parser.add_argument("--mqtt-port", type=int, default=1883)
    parser.add_argument("--mqtt-command-topic", default="node/arm/01/down")
    parser.add_argument("--mqtt-feedback-topic", default="node/arm/01/up")
    parser.add_argument("--mqtt-client-id", default="tl-tcb605-fork")
    parser.add_argument("--mqtt-username", default="")
    parser.add_argument("--mqtt-password", default="")
    parser.add_argument("--mqtt-keepalive", type=int, default=60)
    arguments = parser.parse_args(argv)
    arguments.output = arguments.teach_file
    if arguments.marker_length_m <= 0.0:
        parser.error("--marker-length-m must be positive")
    if arguments.sample_window < arguments.minimum_samples:
        parser.error("--sample-window must be >= --minimum-samples")
    if arguments.cartesian_step_mm <= 0.0:
        parser.error("--cartesian-step-mm must be positive")
    if arguments.carry_clearance_mm < 0.0:
        parser.error("--carry-clearance-mm must not be negative")
    if arguments.attitude_step_deg <= 0.0:
        parser.error("--attitude-step-deg must be positive")
    if arguments.servoj_speed_deg_s <= 0.0 or arguments.servoj_speed_deg_s > 45.0:
        parser.error("--servoj-speed-deg-s must be in (0, 45]")
    if arguments.trim_hold_speed_deg_s <= 0.0 or arguments.trim_hold_speed_deg_s > 45.0:
        parser.error("--trim-hold-speed-deg-s must be in (0, 45]")
    if arguments.trim_hold_segments < 1:
        parser.error("--trim-hold-segments must be at least 1")
    if arguments.trim_lower_clearance_mm < 0.0:
        parser.error("--trim-lower-clearance-mm must not be negative")
    if arguments.servoj_rate_hz <= 0.0 or arguments.servoj_rate_hz > 200.0:
        parser.error("--servoj-rate-hz must be in (0, 200]")
    return arguments


class ForkBasketCycleNode(ForkBasketTeachNode):
    def __init__(self, arguments, calibration_path, calibration, tool_camera):
        super().__init__(arguments, calibration_path, calibration, tool_camera)
        self.ik_client = self.create_client(CoordTransform, "/tl_driver/coord_transform")
        self.state_path = arguments.state_file.expanduser().resolve()
        self.active_transfer = self._load_state()
        self.requested_marker_id = None
        self._marker_number_buffer = ""
        if self.active_transfer is not None:
            print(
                "Active fork transfer state found: "
                f"basket number={int(self.active_transfer['marker_id']) + 1}, "
                f"phase={self.active_transfer.get('phase', 'unknown')}. "
                "Press R only if the arm is physically at the saved phase."
            )

    def _teach_data(self):
        data = self._load_output()
        if not data.get("markers"):
            raise RuntimeError(f"teach file has no basket records: {self.output_path}")
        transfer = data.get("fork_transfer_pose")
        if not transfer or "tcp" not in transfer:
            raise RuntimeError(
                "teach file has no fixed fork_transfer_pose; press T in the teach program"
            )
        waypoint = data.get("carry_waypoint")
        if not waypoint or "joint" not in waypoint:
            raise RuntimeError(
                "teach file has no loaded carry_waypoint; re-teach L, W and T"
            )
        return data

    def _marker_candidates(self):
        data = self._teach_data()
        records = data.get("markers", {})
        visible = {
            marker_id: center
            for marker_id, center in self.latest_marker_centers.items()
            if str(marker_id) in records
        }
        if self.requested_marker_id is not None:
            marker_id = int(self.requested_marker_id)
            if str(marker_id) not in records:
                raise RuntimeError(f"basket number {marker_id + 1} is not taught")
            return marker_id
        if not visible:
            raise RuntimeError(
                "no taught ArUco marker is visible; press 1..6 to select a basket"
            )
        image_center = np.asarray(
            [self.arguments.camera_width / 2.0, self.arguments.camera_height / 2.0]
        )
        return min(
            visible,
            key=lambda marker_id: float(
                np.linalg.norm(np.asarray(visible[marker_id]) - image_center)
            ),
        )

    def _localize_marker(self, marker_id):
        # After a ServoJ stream the driver may pause /tcp_pose publishing and
        # the arm may still be settling; sampling with a stale or moving tool
        # pose projects the wrong pose through the marker transform and shifts
        # the basket by 100+ mm. Wait for a fresh AND stable pose first.
        settle_deadline = time.monotonic() + 5.0
        last_tcp = None
        stable_ticks = 0
        settled = False
        while rclpy.ok() and time.monotonic() < settle_deadline:
            self._check_estop()
            rclpy.spin_once(self, timeout_sec=0.02)
            if time.monotonic() - self.latest_tcp_at > 0.25:
                stable_ticks = 0
                continue
            tcp = np.asarray(self.latest_tcp[:6], dtype=float)
            if last_tcp is not None:
                if float(np.max(np.abs(tcp - last_tcp))) < 0.5:
                    stable_ticks += 1
                    if stable_ticks >= 8:
                        settled = True
                        break
                else:
                    stable_ticks = 0
            last_tcp = tcp
        if not settled:
            raise RuntimeError(
                "/tcp_pose did not settle after reaching the search pose; "
                "check the driver and the arm"
            )
        samples = []
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and len(samples) < 20 and time.monotonic() < deadline:
            self._check_estop()
            rclpy.spin_once(self, timeout_sec=0.0)
            try:
                frames = self.align.process(
                    self.pipeline.wait_for_frames(timeout_ms=2000)
                )
            except RuntimeError:
                continue
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if (
                not color_frame
                or not depth_frame
                or self.latest_tcp is None
                or time.monotonic() - self.latest_tcp_at > 0.3
            ):
                continue
            image = np.asanyarray(color_frame.get_data())
            camera_markers, _, _, _ = self._detect_markers(image, depth_frame)
            if marker_id not in camera_markers:
                continue
            base_markers = self._base_marker_transforms(camera_markers)
            if marker_id in base_markers:
                samples.append(base_markers[marker_id])
        if len(samples) < self.arguments.minimum_samples:
            raise RuntimeError(
                f"marker number {marker_id + 1} was not stable at the search pose; "
                f"got {len(samples)} samples"
            )
        from .fork_basket_teach import average_transforms

        return average_transforms(samples)

    @staticmethod
    def _matrix_to_tcp(transform, attitude=None):
        from scipy.spatial.transform import Rotation

        matrix = np.asarray(transform, dtype=float).reshape(4, 4)
        rpy = Rotation.from_matrix(matrix[:3, :3]).as_euler("XYZ")
        if attitude is not None:
            rpy = np.asarray(attitude, dtype=float)
        return [
            float(matrix[0, 3] * 1000.0),
            float(matrix[1, 3] * 1000.0),
            float(matrix[2, 3] * 1000.0),
            float(rpy[0]),
            float(rpy[1]),
            float(rpy[2]),
        ]

    @staticmethod
    def _locked_attitude(tcp, attitude):
        return [float(tcp[0]), float(tcp[1]), float(tcp[2])] + [
            float(value) for value in attitude
        ]

    def _solve_ik(self, tcp, reference_joint=None):
        if not self.ik_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("/tl_driver/coord_transform is unavailable")
        pose = [float(value) for value in tcp]
        reference_tcp = self.latest_tcp
        if reference_tcp is not None:
            for index in range(3, 6):
                difference = pose[index] - float(reference_tcp[index])
                pose[index] = float(reference_tcp[index]) + math.atan2(
                    math.sin(difference), math.cos(difference)
                )
        request = CoordTransform.Request()
        request.origin_coord = 1
        request.target_coord = 0
        request.form = 0
        request.origin_pos = pose + [0.0]
        reference = (
            list(reference_joint[:7])
            if reference_joint is not None
            else list(self.latest_joint[:7])
        )
        request.reference_pos = (reference + [0.0] * 7)[:7]
        response = self._call(self.ik_client, request, 5.0)
        if not getattr(response, "success", False):
            raise RuntimeError(f"IK failed for TCP={np.round(pose, 2).tolist()}")
        return self._pad_joint(response.target_pos)

    def _move_to_tcp_by_joint(
        self,
        tcp,
        label,
        reference_joint=None,
        fallback_joint=None,
        fallback_tcp=None,
        fallback_limit_mm=15.0,
    ):
        """Solve one endpoint, then stream a joint trajectory to it.

        A valid endpoint can be rejected when an interpolated Cartesian
        waypoint crosses an IK branch boundary. For the approach and carry
        segments, solve the endpoint with its taught joint family and let the
        controller follow one continuous joint trajectory.
        """
        try:
            target_joint = self._solve_ik(tcp, reference_joint)
            print(f"{label}: endpoint IK solved; using direct ServoJ joint path")
        except RuntimeError as error:
            if fallback_joint is None or fallback_tcp is None:
                raise
            mismatch_mm = float(
                np.linalg.norm(
                    np.asarray(tcp[:3], dtype=float)
                    - np.asarray(fallback_tcp[:3], dtype=float)
                )
            )
            if mismatch_mm > float(fallback_limit_mm):
                raise RuntimeError(
                    f"{error}; taught fallback differs by {mismatch_mm:.1f} mm "
                    f"which exceeds {float(fallback_limit_mm):.1f} mm"
                ) from error
            target_joint = self._pad_joint(fallback_joint)
            print(
                f"{label}: endpoint IK rejected; using taught joint pose "
                f"(planned XYZ mismatch={mismatch_mm:.1f} mm)"
            )
        self._stream_joint_path([target_joint], label)
        return target_joint

    def _move_cartesian_with_endpoint_fallback(
        self,
        start_tcp,
        target_tcp,
        label,
        reference_joint=None,
        fallback_joint=None,
        fallback_tcp=None,
    ):
        try:
            self._move_cartesian_line(start_tcp, target_tcp, label)
        except RuntimeError as error:
            print(f"{label}: Cartesian waypoint plan failed ({error}); trying endpoint IK")
            self._move_to_tcp_by_joint(
                target_tcp,
                f"{label}: endpoint fallback",
                reference_joint,
                fallback_joint,
                fallback_tcp,
            )
        return target_tcp

    def _stream_joint_path(self, targets, label, speed_deg_s=None):
        if not targets:
            return
        if not self.arguments.execute:
            raise RuntimeError("runtime motion requires --execute")
        start = self._pad_joint(self.latest_joint)
        self._open_servoj()
        current = np.asarray(start, dtype=float)
        speed = float(
            self.arguments.servoj_speed_deg_s
            if speed_deg_s is None
            else speed_deg_s
        )
        rate = float(self.arguments.servoj_rate_hz)
        for target in targets:
            destination = np.asarray(self._pad_joint(target), dtype=float)
            maximum_delta = float(np.max(np.abs(destination - current)))
            if maximum_delta > self.arguments.max_joint_delta_deg:
                raise RuntimeError(
                    f"{label} joint step {maximum_delta:.1f} deg exceeds "
                    f"{self.arguments.max_joint_delta_deg:.1f} deg"
                )
            count = max(2, int(math.ceil(maximum_delta / speed * rate)))
            for ratio in np.linspace(0.0, 1.0, count + 1)[1:]:
                self._check_estop()
                waypoint = current + (destination - current) * float(ratio)
                message = Float64MultiArray()
                message.data = [float(value) for value in waypoint]
                self.servoj_publisher.publish(message)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(1.0 / rate)
            current = destination
        deadline = time.monotonic() + self.arguments.timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            self._check_estop()
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_joint is None:
                continue
            error = float(
                np.max(np.abs(np.asarray(self.latest_joint[:6]) - current[:6]))
            )
            if error <= self.arguments.arrival_tolerance_deg:
                print(f"{label}: arrived, max joint error={error:.3f} deg")
                return
        raise RuntimeError(f"{label}: ServoJ arrival timeout")

    @staticmethod
    def _interpolate_joint_path(start, end, segments):
        """Evenly spaced joint-space points from start to end (inclusive of end)."""
        start = np.asarray(start, dtype=float)
        end = np.asarray(end, dtype=float)
        segments = max(1, int(segments))
        return [
            start + (end - start) * (index / float(segments))
            for index in range(1, segments + 1)
        ]

    def _move_cartesian_line(self, start_tcp, target_tcp, label, speed_deg_s=None):
        start = np.asarray(start_tcp, dtype=float)
        target = np.asarray(target_tcp, dtype=float)
        distance = float(np.linalg.norm(target[:3] - start[:3]))
        count = max(1, int(math.ceil(distance / self.arguments.cartesian_step_mm)))
        # Extra segments for large attitude changes while carrying the basket:
        # keep each waypoint's rotation small so the basket never tilts fast.
        from scipy.spatial.transform import Rotation

        start_rotation = Rotation.from_euler("XYZ", start[3:6], degrees=False)
        target_rotation = Rotation.from_euler("XYZ", target[3:6], degrees=False)
        attitude_change_deg = math.degrees(
            (start_rotation.inv() * target_rotation).magnitude()
        )
        attitude_segments = int(
            math.ceil(attitude_change_deg / float(self.arguments.attitude_step_deg))
        )
        count = max(count, attitude_segments)
        reference = self._pad_joint(self.latest_joint)
        joint_targets = []
        for index in range(1, count + 1):
            ratio = index / float(count)
            pose = start + (target - start) * ratio
            for axis in range(3, 6):
                difference = target[axis] - start[axis]
                pose[axis] = start[axis] + math.atan2(
                    math.sin(difference), math.cos(difference)
                ) * ratio
            reference = self._solve_ik(pose.tolist(), reference)
            joint_targets.append(reference)
        print(
            f"{label}: Cartesian line, {count} waypoints "
            f"(attitude change {attitude_change_deg:.1f} deg)"
        )
        self._stream_joint_path(joint_targets, label, speed_deg_s=speed_deg_s)

    def _prepare_targets(self, base_marker, record, transfer):
        """Build pick targets in translation-only mode.

        The taught ATTITUDES are kept fixed (marker rotation estimates are
        noisy and drift run to run, which flips reachability near
        singularities); only the marker TRANSLATION from the taught search
        pose is applied to the taught positions.
        """
        search = record.get("search_pose")
        if not search or "T_base_marker" not in search:
            raise RuntimeError("basket record has no taught T_base_marker; re-teach B")
        teach_marker = np.asarray(search["T_base_marker"], dtype=float).reshape(4, 4)
        shift = np.asarray(base_marker, dtype=float)[:3, 3] - teach_marker[:3, 3]
        shift_mm = shift * 1000.0
        print(
            "marker shift[mm]="
            f"{np.round(shift_mm, 1).tolist()}, "
            f"distance={np.linalg.norm(shift_mm):.1f} mm"
        )
        if float(np.linalg.norm(shift_mm)) > float(self.arguments.max_basket_shift_mm):
            raise RuntimeError(
                f"basket shifted {np.linalg.norm(shift_mm):.1f} mm from teach; "
                f"limit is {self.arguments.max_basket_shift_mm:.0f} mm"
            )

        def translated(pose_name):
            taught = record[pose_name]["taught_tcp"]
            tcp = [float(value) for value in taught]
            tcp[0] += float(shift_mm[0])
            tcp[1] += float(shift_mm[1])
            tcp[2] += float(shift_mm[2])
            return tcp

        lift_tcp = translated("fork_lift")
        return {
            "approach": translated("fork_approach"),
            "insert": translated("fork_insert"),
            "lift": lift_tcp,
            # The fork keeps the basket level while carrying it. The taught
            # temporary position contributes only XYZ; its recorded attitude
            # is deliberately ignored.
            "temporary": self._locked_attitude(transfer["tcp"], lift_tcp[3:6]),
            "attitude": lift_tcp[3:6],
        }

    def _safe_carry_z(self, targets):
        return max(float(targets["lift"][2]), float(targets["temporary"][2])) + float(
            self.arguments.carry_clearance_mm
        )

    def _rotate_at_pose(self, start_tcp, target_attitude, label):
        """Rotate in place to target_attitude at the current XY/Z.

        If that pose is unreachable, lower 60 mm (freer joints), rotate
        there, then rise back. Always executed before any basket is carried.
        """
        start = [float(value) for value in start_tcp]
        target = self._locked_attitude(start, target_attitude)
        try:
            self._move_cartesian_line(start, target, label)
            return target
        except RuntimeError as error:
            print(f"{label}: hover rotate unreachable ({error}); trying 60 mm lower")
            low = self._locked_attitude(start, start[3:6])
            low[2] = float(start[2]) - 60.0
            self._move_cartesian_line(start, low, f"{label}: lower")
            low_target = self._locked_attitude(low, target_attitude)
            self._move_cartesian_line(low, low_target, f"{label}: rotate low")
            high_target = self._locked_attitude(
                [low_target[0], low_target[1], float(start[2])], target_attitude
            )
            self._move_cartesian_line(low_target, high_target, f"{label}: rise back")
            return high_target

    def _find_hover_z(
        self, x, y, attitude, start_z, min_z, label, reference_joint=None
    ):
        """Highest reachable Z for a horizontal hover at (x, y).

        Tries both the taught reference joints and the current joints at
        each height, stepping down 20 mm until min_z. This removes the
        reference-mismatch failures seen before.
        """
        z = float(start_z)
        while z >= float(min_z) - 1e-6:
            for reference in (reference_joint, None):
                try:
                    self._solve_ik(
                        [float(x), float(y), z, attitude[0], attitude[1], attitude[2]],
                        reference,
                    )
                    print(
                        f"{label}: hover Z={z:.0f} mm reachable "
                        f"(ref={'taught' if reference is not None else 'current'})"
                    )
                    return z
                except RuntimeError:
                    continue
            print(f"{label}: Z={z:.0f} mm unreachable with all references")
            z -= 20.0
        raise RuntimeError(
            f"{label}: no reachable hover Z in "
            f"[{float(min_z):.0f}, {float(start_z):.0f}] mm"
        )

    def _carry_translate_segmented(self, start_high, target_high, z, label):
        """Travel in 3 short segments so each endpoint IK uses a nearby
        reference (a single long jump often fails the numerical IK)."""
        segments = 3
        attitude = target_high[3:6]
        start_xy = [float(start_high[0]), float(start_high[1])]
        target_xy = [float(target_high[0]), float(target_high[1])]
        for index in range(1, segments + 1):
            ratio = index / float(segments)
            waypoint = self._locked_attitude(
                [
                    start_xy[0] + (target_xy[0] - start_xy[0]) * ratio,
                    start_xy[1] + (target_xy[1] - start_xy[1]) * ratio,
                    z,
                ],
                attitude,
            )
            self._move_to_tcp_by_joint(
                waypoint, f"{label}: translate seg {index}/{segments}"
            )

    def _carry_transfer(self, start_high, target_high, targets, label, reference_joint=None):
        """High-Z horizontal travel while carrying the basket.

        1. rotate to the target attitude IN PLACE at the start XY,
        2. translate with the target attitude locked, in short segments,
           stepping the height down towards the temporary XY until the
           endpoint becomes reachable.
        """
        rotate_target = self._locked_attitude(
            [start_high[0], start_high[1], start_high[2]], target_high[3:6]
        )
        self._move_to_tcp_by_joint(rotate_target, f"{label}: rotate in place")
        min_z = max(float(targets["lift"][2]), float(targets["temporary"][2])) + float(
            self.arguments.carry_extra_mm
        )
        z = float(start_high[2])
        while z >= float(min_z) - 1e-6:
            try:
                self._carry_translate_segmented(start_high, target_high, z, label)
                return
            except RuntimeError as error:
                print(f"{label}: translate Z={z:.0f} mm failed; retrying lower")
                z -= 20.0
        raise RuntimeError(
            f"{label}: temporary XY has no reachable travel Z above "
            f"{float(min_z):.0f} mm. The temporary position lies in a "
            "low-workspace dead zone. RE-TEACH T: pick a temporary position "
            "where the arm can still lift 150 mm+ (centre of the workspace), "
            "not at the far edge."
        )

    def _move_to_search(self, record):
        search = record.get("search_pose")
        if not search or "joint" not in search or "tcp" not in search:
            raise RuntimeError("selected basket is missing search_pose")
        self._move_joint(search["joint"])
        return [float(value) for value in search["tcp"]]

    def _move_to_observation(self, data):
        """Return to the observation (standby) pose.

        Command-line --observation-joints wins over the YAML pose, so a
        stray O keypress during teaching cannot poison the standby pose.
        """
        observation = None
        if self.arguments.observation_joints is not None:
            observation = {
                "joint": list(self.arguments.observation_joints) + [0.0]
            }
            print("Observation pose: command-line joints.")
        elif data and "observation_pose" in data:
            observation = data["observation_pose"]
            print("Observation pose: YAML observation_pose.")
        if not observation or "joint" not in observation:
            return None
        print("Returning to the observation (standby) pose...")
        self._move_joint(observation["joint"])
        return [float(value) for value in observation.get("tcp", [0.0] * 6)]

    def _pick_to_temporary(self):
        if self.active_transfer is not None:
            raise RuntimeError("a basket is already held; press R to return it first")
        data = self._teach_data()
        self._move_to_observation(data)
        marker_id = self._marker_candidates()
        record = data["markers"].get(str(marker_id))
        for name in ("fork_approach", "fork_insert", "fork_lift"):
            if not record.get(name):
                raise RuntimeError(f"basket number {marker_id + 1} is missing {name}")
        search_tcp = self._move_to_search(record)
        print(f"Basket number {marker_id + 1}: localizing marker at search pose...")
        base_marker = self._localize_marker(marker_id)
        targets = self._prepare_targets(
            base_marker, record, data["fork_transfer_pose"]
        )
        current_tcp = list(search_tcp)
        # The teach program stores the reference joints under "taught_joint";
        # accept the legacy "joint" key as a fallback.
        approach_reference = record["fork_approach"].get("taught_joint") or record[
            "fork_approach"
        ].get("joint")
        insert_reference = record["fork_insert"].get("taught_joint") or record[
            "fork_insert"
        ].get("joint")
        lift_reference = record["fork_lift"].get("taught_joint") or record[
            "fork_lift"
        ].get("joint")
        transfer_reference = data["fork_transfer_pose"].get("joint")
        try:
            # The approach travel happens BEFORE the basket is lifted, so the
            # floor must clear approach/insert heights, NOT the lift pose
            # (using lift_z made the floor exceed the search-pose height and
            # the probe failed instantly).
            approach_min_z = max(
                float(targets["approach"][2]), float(targets["insert"][2])
            ) + float(self.arguments.carry_clearance_mm)
            carry_z = self._safe_carry_z(targets)
            # The hover height is probed with the LIFT attitude and the
            # CURRENT joints as reference (a low-altitude taught joint
            # reference cannot solve high-Z targets).
            # The hover above the approach point uses the APPROACH attitude:
            # that is the pose the fork actually reaches at this XY (the
            # carry/lift attitude is frequently IK-dead at the approach XY,
            # which made the hover search fail with "no reachable hover Z").
            # If no hover Z is reachable at all (the approach XY sits at the
            # workspace edge), fall back to moving straight from the search
            # pose to the approach pose: that leg uses the taught-joint
            # endpoint fallback, which is guaranteed by the drag teaching.
            try:
                hover_z = self._find_hover_z(
                    targets["approach"][0],
                    targets["approach"][1],
                    targets["approach"][3:6],
                    current_tcp[2],
                    approach_min_z,
                    "approach",
                    approach_reference,
                )
                search_high = self._locked_attitude(
                    [current_tcp[0], current_tcp[1], hover_z], current_tcp[3:6]
                )
                self._move_cartesian_line(current_tcp, search_high, "approach: lift")
                # Solve the high approach endpoint with the current joints as
                # reference (same attitude), so the IK matches.
                approach_high = self._locked_attitude(
                    [targets["approach"][0], targets["approach"][1], hover_z],
                    targets["approach"][3:6],
                )
                self._move_to_tcp_by_joint(
                    approach_high, "approach: high transfer", None
                )
            except RuntimeError as hover_error:
                print(
                    f"WARNING: {hover_error}; moving directly from the search "
                    "pose to the approach pose (taught-joint fallback)"
                )
                approach_high = list(current_tcp)
            self._move_cartesian_with_endpoint_fallback(
                approach_high,
                targets["approach"],
                "approach: lower",
                approach_reference,
                approach_reference,
                record["fork_approach"]["taught_tcp"],
            )
            if self.latest_joint is None:
                raise RuntimeError("joint feedback is not ready after approach")
            approach_reached_joint = self._pad_joint(self.latest_joint)
            self._move_cartesian_with_endpoint_fallback(
                targets["approach"],
                targets["insert"],
                "insert: horizontal",
                insert_reference,
                insert_reference,
                record["fork_insert"]["taught_tcp"],
            )
            if self.latest_joint is None:
                raise RuntimeError("joint feedback is not ready after insert")
            insert_reached_joint = self._pad_joint(self.latest_joint)
            self.active_transfer = {
                "marker_id": marker_id,
                "base_marker": np.asarray(base_marker, dtype=float).tolist(),
                "targets": targets,
                "search_joint": [float(value) for value in record["search_pose"]["joint"]],
                "search_tcp": search_tcp,
                "carry_z": float(carry_z),
                "phase": "inserted",
                "approach_reached_joint": approach_reached_joint,
                "insert_reached_joint": insert_reached_joint,
                "joint_refs": {
                    "approach": approach_reference,
                    "insert": insert_reference,
                    "lift": lift_reference,
                    "transfer": transfer_reference,
                },
                "started_utc": datetime.now(timezone.utc).isoformat(),
            }
            self._write_state()
            self._move_cartesian_with_endpoint_fallback(
                targets["insert"],
                targets["lift"],
                "lift: vertical",
                lift_reference,
                lift_reference,
                record["fork_lift"]["taught_tcp"],
            )
            if self.latest_joint is None:
                raise RuntimeError("joint feedback is not ready after lift")
            self.active_transfer["lift_reached_joint"] = self._pad_joint(self.latest_joint)
            self.active_transfer["phase"] = "lifted"
            self._write_state()
            # No separate carry-lift: the basket is already suspended on the
            # fork at the lift pose, and the transfer is a direct joint drive
            # through the taught waypoint (always reachable). Trying to lift
            # to carry_z (lift_z + clearance) can exceed the workspace when
            # the lift pose is high.
            # Direct joint drive through the taught high-altitude carry
            # waypoint to the taught temporary pose: the arm has physically
            # been at every one of these poses during teaching, so the whole
            # carry path is always reachable.
            waypoint = data["carry_waypoint"]
            transfer_joint = self._pad_joint(data["fork_transfer_pose"]["joint"])
            drive_stops = [self._pad_joint(waypoint["joint"]), transfer_joint]
            for stop in drive_stops:
                print(
                    "carry: joint drive to "
                    f"joints={np.round(stop[:6], 3).tolist()}"
                )
                self._stream_joint_path([stop], "carry: joint drive")
            self.active_transfer["phase"] = "temporary"
            self._write_state()
            # 修剪作业流程：T -> 位置1（高位过渡，绕 J1 回转叉保持水平）
            #               -> 位置2（修剪低位，J5 大幅翻转，分段低速防花篮滑落）
            hold1 = self._pad_joint(TRIM_HOLD_1_JOINT_DEG)
            hold2 = self._pad_joint(TRIM_HOLD_2_JOINT_DEG)
            print(
                "carry: joint drive to trim position 1 "
                f"joints={np.round(hold1[:6], 3).tolist()}"
            )
            self._stream_joint_path([hold1], "carry: T->P1")
            self.active_transfer["phase"] = "hold1"
            self._write_state()
            # 位置1 -> 位置2：先在位置2 的 XY 上方以较高高度平移（叉保持
            # 位置1 的水平姿态，花篮稳），到 XY 之后低速垂直下降+翻转到位。
            hold1_tcp = TRIM_HOLD_1_TCP
            hold2_tcp = TRIM_HOLD_2_TCP
            mid_tcp = [
                hold2_tcp[0],
                hold2_tcp[1],
                hold2_tcp[2] + self.arguments.trim_lower_clearance_mm,
            ] + list(hold1_tcp[3:6])
            try:
                self._move_cartesian_line(
                    hold1_tcp, mid_tcp, "carry: P1->P2 high translate"
                )
                self._move_cartesian_line(
                    mid_tcp,
                    hold2_tcp,
                    "carry: P2 lower: vertical",
                    speed_deg_s=self.arguments.trim_hold_speed_deg_s,
                )
            except RuntimeError as cartesian_error:
                print(
                    f"WARNING: {cartesian_error}; falling back to the joint "
                    "interpolation path for trim position 2"
                )
                hold2_path = self._interpolate_joint_path(
                    hold1, hold2, self.arguments.trim_hold_segments
                )
                self._stream_joint_path(
                    hold2_path,
                    "carry: P1->P2 (fallback)",
                    speed_deg_s=self.arguments.trim_hold_speed_deg_s,
                )
            self.active_transfer["phase"] = "hold2"
            self._write_state()
            print(
                f"Basket number {marker_id + 1} is held at trim position 2. "
                "The fork has not released it; press R to return it."
            )
        finally:
            self._close_servoj()

    def _return_to_original(self):
        if self.active_transfer is None:
            raise RuntimeError("no active held basket; press P first")
        active = self.active_transfer
        phase = str(active.get("phase", "temporary"))
        data = self._load_output()

        def reached_joint(name):
            value = active.get(f"{name}_reached_joint")
            if value is None:
                raise RuntimeError(
                    f"active state lacks {name}_reached_joint; "
                    "restart the cycle from an empty fork instead of guessing "
                    "the loaded return path"
                )
            return self._pad_joint(value)

        try:
            if phase == "hold2":
                hold1 = self._pad_joint(TRIM_HOLD_1_JOINT_DEG)
                hold2 = self._pad_joint(TRIM_HOLD_2_JOINT_DEG)
                # 返回对称路径：先低速垂直提起+回翻（位置2 -> 中间高），
                # 再以水平姿态平移回位置1。
                hold1_tcp = TRIM_HOLD_1_TCP
                hold2_tcp = TRIM_HOLD_2_TCP
                mid_tcp = [
                    hold2_tcp[0],
                    hold2_tcp[1],
                    hold2_tcp[2] + self.arguments.trim_lower_clearance_mm,
                ] + list(hold1_tcp[3:6])
                try:
                    self._move_cartesian_line(
                        hold2_tcp,
                        mid_tcp,
                        "return: loaded P2 lift: vertical",
                        speed_deg_s=self.arguments.trim_hold_speed_deg_s,
                    )
                    self._move_cartesian_line(
                        mid_tcp, hold1_tcp, "return: loaded P2->P1 high translate"
                    )
                except RuntimeError as cartesian_error:
                    print(
                        f"WARNING: {cartesian_error}; falling back to the joint "
                        "interpolation path for the P2->P1 return"
                    )
                    hold1_path = self._interpolate_joint_path(
                        hold2, hold1, self.arguments.trim_hold_segments
                    )
                    self._stream_joint_path(
                        hold1_path,
                        "return: loaded P2->P1 (fallback)",
                        speed_deg_s=self.arguments.trim_hold_speed_deg_s,
                    )
                phase = "hold1"
                active["phase"] = phase
                self._write_state()

            if phase == "hold1":
                transfer_joint = self._pad_joint(data["fork_transfer_pose"]["joint"])
                print(
                    "return: loaded trim position 1 -> temporary "
                    f"joints={np.round(transfer_joint[:6], 3).tolist()}"
                )
                self._stream_joint_path([transfer_joint], "return: loaded P1->T")
                phase = "temporary"
                active["phase"] = phase
                self._write_state()

            if phase == "temporary":
                waypoint = data["carry_waypoint"]
                print(
                    "return: loaded temporary -> carry waypoint "
                    f"joints={np.round(self._pad_joint(waypoint['joint'])[:6], 3).tolist()}"
                )
                self._stream_joint_path(
                    [self._pad_joint(waypoint["joint"])],
                    "return: loaded T->W",
                )

                phase = "waypoint"
                active["phase"] = phase
                self._write_state()

            if phase == "waypoint":
                lift_joint = reached_joint("lift")
                print(
                    "return: loaded carry waypoint -> lift "
                    f"joints={np.round(lift_joint[:6], 3).tolist()}"
                )
                self._stream_joint_path([lift_joint], "return: loaded W->L")
                phase = "lifted"
                active["phase"] = phase
                self._write_state()

            if phase == "lifted":
                insert_joint = reached_joint("insert")
                print(
                    "return: loaded lift -> insert support "
                    f"joints={np.round(insert_joint[:6], 3).tolist()}"
                )
                self._stream_joint_path([insert_joint], "return: loaded L->I")
                phase = "inserted"
                active["phase"] = phase
                self._write_state()

            if phase == "inserted":
                approach_joint = reached_joint("approach")
                print(
                    "return: loaded insert -> approach retract "
                    f"joints={np.round(approach_joint[:6], 3).tolist()}"
                )
                self._stream_joint_path([approach_joint], "return: loaded I->F")
                phase = "retracted"
                active["phase"] = phase
                self._write_state()

            if phase != "retracted":
                raise RuntimeError(f"unsupported active fork phase: {phase}")

            search_joint = self._pad_joint(active["search_joint"])
            print(
                "return: empty fork approach -> search pose "
                f"joints={np.round(search_joint[:6], 3).tolist()}"
            )
            self._stream_joint_path([search_joint], "return: empty F->search")
            self._clear_state()
            print(
                "Basket returned to its original support; fork retracted; "
                "search pose reached."
            )
            self._move_to_observation(data)
            print("Cycle complete at the observation (standby) pose.")
        finally:
            self._close_servoj()

    def _write_state(self):
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        payload = yaml.safe_dump(
            {"active": True, **self.active_transfer},
            allow_unicode=True,
            sort_keys=False,
        )
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=self.state_path.parent,
            prefix=f".{self.state_path.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            stream.write(payload)
            temporary_path = Path(stream.name)
        temporary_path.replace(self.state_path)

    def _load_state(self):
        if not self.state_path.exists():
            return None
        try:
            payload = yaml.safe_load(self.state_path.read_text(encoding="utf-8")) or {}
        except (OSError, yaml.YAMLError) as error:
            print(f"WARNING: cannot read active fork state: {error}")
            return None
        if not payload.get("active"):
            return None
        required = ("marker_id", "targets", "search_tcp", "search_joint", "carry_z")
        if any(key not in payload for key in required):
            print("WARNING: active fork state is incomplete; ignoring it")
            return None
        return payload

    def _clear_state(self):
        self.active_transfer = None
        if self.state_path.exists():
            self.state_path.unlink()

    def _draw_cycle_preview(self, image, corners, ids):
        preview = image.copy()
        if ids is not None and len(ids):
            cv2.aruco.drawDetectedMarkers(preview, corners, ids)
        visible = sorted(marker_id + 1 for marker_id in self.latest_marker_centers)
        selected = (
            "auto"
            if self.requested_marker_id is None
            else str(self.requested_marker_id + 1)
        )
        cv2.putText(
            preview,
            f"visible={visible} selected={selected} active={self.active_transfer is not None}",
            (8, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (0, 255, 255),
            2,
        )
        cv2.putText(
            preview,
            "1..6 select  P pick-to-temporary  R return-to-original  Q quit",
            (8, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (0, 255, 0),
            2,
        )
        return preview

    def run(self):
        if not self.arguments.execute:
            raise RuntimeError("start the executor with --execute; it must send robot motion")
        self._wait_feedback()
        self._start_camera()
        data = self._teach_data()
        print("Standalone U-fork basket cycle.")
        print("No hand, gripper, MQTT, YOLO, or safety proxy is used.")
        print("P: search marker -> insert -> lift -> fixed temporary hold.")
        print("R: lift -> return above original -> lower -> retract -> search pose.")
        print(f"Teach file: {self.output_path}")
        print(f"Active state file: {self.state_path}")
        print(
            "Taught baskets: "
            + str(sorted(int(key) + 1 for key in data.get("markers", {})))
        )
        window_name = "TL standalone U-fork basket cycle"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            try:
                frames = self.align.process(self.pipeline.wait_for_frames(timeout_ms=2000))
            except RuntimeError as error:
                self.get_logger().warning(f"D435 frame timeout: {error}")
                continue
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame or self.latest_tcp is None:
                continue
            image = np.asanyarray(color_frame.get_data()).copy()
            camera_markers, centers, corners, ids = self._detect_markers(image, depth_frame)
            self.latest_marker_centers = centers
            self.latest_base_markers = self._base_marker_transforms(camera_markers)
            cv2.imshow(window_name, self._draw_cycle_preview(image, corners, ids))
            key = cv2.waitKey(1) & 0xFF
            try:
                if key in (ord("q"), 27):
                    break
                if ord("0") <= key <= ord("9"):
                    digit = key - ord("0")
                    if len(self._marker_number_buffer) < 3:
                        self._marker_number_buffer += str(digit)
                    print(
                        f"Basket number buffer: {self._marker_number_buffer or '(empty)'}; "
                        "press P to pick, c to clear"
                    )
                elif key == ord("c"):
                    self._marker_number_buffer = ""
                    print("Basket number buffer cleared.")
                elif key == ord("p"):
                    self._commit_marker_number()
                    self._pick_to_temporary()
                elif key == ord("r"):
                    self._return_to_original()
            except (RuntimeError, ValueError) as error:
                print(f"Command rejected/stopped: {error}")

    def stop(self):
        try:
            self._close_servoj()
        except Exception as error:
            print(f"ServoJ close warning: {error}")
        super().stop()


def main(argv=None):
    arguments = parse_args(argv)
    calibration_path, calibration, tool_camera = load_calibration(
        arguments.calibration, arguments.allow_rejected
    )
    print(
        f"calibration={calibration_path}\n"
        f"accepted={bool(calibration.get('accepted', False))}, "
        f"convention={calibration.get('rpy_convention', 'intrinsic_xyz')}"
    )
    rclpy.init(args=[])
    node = ForkBasketCycleNode(arguments, calibration_path, calibration, tool_camera)
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

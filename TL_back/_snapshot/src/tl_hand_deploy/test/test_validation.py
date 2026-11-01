import unittest

import numpy as np

from tl_hand_deploy.reference_compat import (
    approach_point_above_reachable_grasp,
    build_hover_pose,
    camera_point_to_base,
    format_joint_errors,
    joint_error_degrees,
    linear_joint_trajectory,
    nearest_equivalent_joint_angles,
    summarize_validation_records,
    tool_pose_for_virtual_tcp,
    transform_delta,
    vendor_movej_target,
    wrap_angle_rad,
    xy_probe_pose,
)
from tl_hand_deploy.geometry import pose_to_transform
from tl_hand_deploy.validation import (
    validate_cartesian_target,
    validate_joint_target,
    validate_motion_scalars,
)


class ValidationTest(unittest.TestCase):
    def test_joint_step_is_rejected(self):
        result = validate_joint_target(
            [30, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [-170] * 6,
            [170] * 6,
            20,
        )
        self.assertFalse(result.accepted)

    def test_cartesian_workspace_is_rejected(self):
        result = validate_cartesian_target(
            [0, 0, 10, 0, 0, 0],
            [0, 0, 100, 0, 0, 0],
            [-650, -650, 60],
            [650, 650, 850],
            100,
            0.5,
        )
        self.assertFalse(result.accepted)

    def test_vendor_movel_scalars_are_accepted(self):
        result = validate_motion_scalars(20.0, 20.0, 20.0, [20.0, 20.0, 20.0])
        self.assertTrue(result.accepted)

    def test_reference_coordinate_chain(self):
        point = camera_point_to_base(
            [100.0, 200.0, 300.0, 0.0, 0.0, 0.0],
            np.eye(4),
            [0.01, 0.02, 0.5],
        )
        np.testing.assert_allclose(point, [110.0, 220.0, 800.0])

    def test_reference_hover_uses_current_orientation(self):
        pose = build_hover_pose([10.0, 20.0, 30.0], [0, 0, 0, 1, 2, 3], 100.0)
        self.assertEqual(pose, [10.0, 20.0, 130.0, 1.0, 2.0, 3.0])

    def test_reference_hover_wraps_unbounded_rpy(self):
        pose = build_hover_pose(
            [10.0, 20.0, 30.0], [0, 0, 0, -0.041, -9.166, 0.06], 150.0
        )
        self.assertAlmostEqual(pose[3], -0.041)
        self.assertAlmostEqual(pose[4], wrap_angle_rad(-9.166))
        self.assertAlmostEqual(pose[5], 0.06)

    def test_ik_joint_wrap_uses_nearest_equivalent(self):
        normalized = nearest_equivalent_joint_angles(
            [252.0, 359.0],
            [-90.0, -1.0],
            [-170.0, -360.0],
            [170.0, 360.0],
        )
        np.testing.assert_allclose(normalized, [-108.0, -1.0])

    def test_reference_servoj_trajectory_reaches_target(self):
        trajectory = linear_joint_trajectory([0.0] * 7, [10.0] * 7, 500)
        self.assertEqual(len(trajectory), 500)
        np.testing.assert_allclose(trajectory[-1], [10.0] * 7)

    def test_pick_approach_stays_above_auto_lifted_grasp(self):
        grasp = [320.0, -150.0, 90.0]
        approach = approach_point_above_reachable_grasp(grasp, 130.0, 50.0)
        np.testing.assert_allclose(approach, [320.0, -150.0, 170.0])

    def test_vendor_movej_target_has_fourteen_values(self):
        target = vendor_movej_target([1, 2, 3, 4, 5, 6, 0])
        self.assertEqual(target, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0] + [0.0] * 8)

    def test_movej_joint_error_reports_target_minus_current(self):
        errors = joint_error_degrees([1, 2, 3, 4, 5, 6], [3, 1, 3, 8, 0, 7])
        np.testing.assert_allclose(errors, [2, -1, 0, 4, -5, 1])
        self.assertEqual(
            format_joint_errors(errors),
            "[J1=+2.000, J2=-1.000, J3=+0.000, J4=+4.000, J5=-5.000, J6=+1.000]",
        )

    def test_transform_delta_reports_translation(self):
        first = np.eye(4)
        second = np.eye(4)
        second[0, 3] = 0.052
        translation_mm, rotation_deg = transform_delta(first, second)
        self.assertAlmostEqual(translation_mm, 52.0)
        self.assertAlmostEqual(rotation_deg, 0.0)

    def test_validation_summary_detects_fixed_offset(self):
        records = []
        for x_value, y_value in ((0, 0), (100, 0), (200, 0), (0, 100), (100, 100), (200, 100)):
            records.append(
                {
                    "estimated_object_base_mm": [x_value, y_value, 0.0],
                    "correction_base_mm": [5.0, -3.0, 0.0],
                }
            )
        summary = summarize_validation_records(records)
        self.assertEqual(summary["judgment"], "predominantly fixed offset")
        self.assertAlmostEqual(summary["after_fixed_bias_xy_rms_mm"], 0.0)

    def test_validation_summary_detects_spatial_error(self):
        records = []
        for x_value, y_value in ((0, 0), (100, 0), (200, 0), (0, 100), (100, 100), (200, 100)):
            records.append(
                {
                    "estimated_object_base_mm": [x_value, y_value, 0.0],
                    "correction_base_mm": [x_value / 10.0, y_value / 10.0, 0.0],
                }
            )
        summary = summarize_validation_records(records)
        self.assertEqual(
            summary["judgment"], "position-dependent systematic error"
        )

    def test_xy_probe_pose_moves_target_toward_current_tcp(self):
        target = [-10.0, -586.0, -35.0, 3.1, 0.0, 1.56]
        current = [14.0, -369.0, 189.0, 3.1, 0.0, 1.56]
        pose, shift = xy_probe_pose(target, current, 50.0, 100.0)
        self.assertGreater(shift[1], 0.0)
        self.assertLess(abs(np.linalg.norm(shift) - 50.0), 1e-6)
        self.assertAlmostEqual(pose[0], target[0] + shift[0])
        self.assertAlmostEqual(pose[1], target[1] + shift[1])
        self.assertAlmostEqual(pose[2], target[2] + 100.0)

    def test_tool_pose_for_virtual_tcp_hits_desired_point(self):
        desired_tip_mm = [30.0, -420.0, 80.0]
        current_tool_pose = [10.0, -300.0, 200.0, 0.0, 0.0, 1.57]
        offset_tool_mm = [0.0, 0.0, -150.0]
        tool_pose = tool_pose_for_virtual_tcp(
            desired_tip_mm, current_tool_pose, offset_tool_mm
        )
        base_tool = pose_to_transform(
            np.asarray(tool_pose[:3]) / 1000.0, tool_pose[3:]
        )
        actual_tip_mm = (
            base_tool
            @ np.r_[np.asarray(offset_tool_mm, dtype=float) / 1000.0, 1.0]
        )[:3] * 1000.0
        np.testing.assert_allclose(actual_tip_mm, desired_tip_mm, atol=1e-6)

    def test_tool_pose_for_virtual_tcp_accepts_fixed_grasp_rpy(self):
        desired_tip_mm = [30.0, -420.0, 80.0]
        current_tool_pose = [10.0, -300.0, 200.0, 0.3, -0.2, 0.1]
        fixed_rpy = [3.087, -0.014, 1.602]
        offset_tool_mm = [20.0, 0.0, -150.0]
        tool_pose = tool_pose_for_virtual_tcp(
            desired_tip_mm, current_tool_pose, offset_tool_mm, target_tool_rpy_rad=fixed_rpy
        )
        np.testing.assert_allclose(tool_pose[3:], fixed_rpy, atol=1e-12)
        base_tool = pose_to_transform(
            np.asarray(tool_pose[:3]) / 1000.0, tool_pose[3:]
        )
        actual_tip_mm = (
            base_tool
            @ np.r_[np.asarray(offset_tool_mm, dtype=float) / 1000.0, 1.0]
        )[:3] * 1000.0
        np.testing.assert_allclose(actual_tip_mm, desired_tip_mm, atol=1e-6)


if __name__ == "__main__":
    unittest.main()

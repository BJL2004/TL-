import math
import unittest
from types import SimpleNamespace

import cv2
import numpy as np

from tl_hand_deploy.geometry import make_transform, pose_to_transform, rotation_distance_rad
from tl_hand_deploy.handeye_calibrate import (
    _image_coverage,
    _leave_one_out_stability,
    _motion_diversity,
    _robust_handeye_solve,
    _robust_upper_limit,
    _solve_candidates,
)


class HandEyeCalibrateTest(unittest.TestCase):
    @staticmethod
    def _synthetic_dataset():
        samples = []
        gripper_base = []
        for index in range(24):
            row = index // 6
            column = index % 6
            translation_mm = [
                -140.0 + column * 55.0,
                -100.0 + row * 65.0,
                280.0 + ((index * 37) % 7) * 24.0,
            ]
            rpy = [
                math.radians(-35.0 + column * 14.0),
                math.radians(-28.0 + row * 18.0),
                math.radians(-25.0 + (index % 5) * 13.0),
            ]
            samples.append(
                {
                    "index": index + 1,
                    "tcp_pose_mm_rad": translation_mm + rpy,
                    "checkerboard_quality": {
                        "center_normalized": [
                            0.15 + 0.14 * (column % 5),
                            0.15 + 0.22 * (row % 4),
                        ],
                        "area_ratio": 0.08,
                    },
                }
            )
            gripper_base.append(
                pose_to_transform(
                    np.asarray(translation_mm) / 1000.0,
                    rpy,
                    "intrinsic_xyz",
                )
            )

        tool_camera = pose_to_transform(
            [0.065, -0.028, 0.042],
            [math.radians(4.0), math.radians(-7.0), math.radians(91.0)],
            "intrinsic_xyz",
        )
        base_board = pose_to_transform(
            [0.42, -0.18, 0.03],
            [math.radians(2.0), math.radians(-3.0), math.radians(12.0)],
            "intrinsic_xyz",
        )
        rvecs = []
        tvecs = []
        for transform in gripper_base:
            camera_board = np.linalg.inv(tool_camera) @ np.linalg.inv(transform) @ base_board
            rvecs.append(cv2.Rodrigues(camera_board[:3, :3])[0])
            tvecs.append(camera_board[:3, 3].reshape(3, 1))
        return samples, rvecs, tvecs, tool_camera

    def test_default_candidates_use_only_intrinsic_xyz(self):
        samples, rvecs, tvecs, expected = self._synthetic_dataset()
        candidates = _solve_candidates(samples, rvecs, tvecs)
        self.assertTrue(candidates)
        self.assertEqual({item["rpy_convention"] for item in candidates}, {"intrinsic_xyz"})
        selected = candidates[0]["T_tool0_camera"]
        self.assertLess(np.linalg.norm(selected[:3, 3] - expected[:3, 3]) * 1000.0, 0.1)
        self.assertLess(math.degrees(rotation_distance_rad(selected, expected)), 0.01)

    def test_leave_one_out_is_stable_for_well_conditioned_data(self):
        samples, rvecs, tvecs, _ = self._synthetic_dataset()
        selected = _solve_candidates(samples, rvecs, tvecs)[0]
        metrics = _leave_one_out_stability(samples, rvecs, tvecs, selected)
        self.assertEqual(metrics["leave_one_out_failed_sample_indices"], [])
        self.assertLess(metrics["leave_one_out_translation_max_mm"], 0.1)
        self.assertLess(metrics["leave_one_out_rotation_max_deg"], 0.01)

    def test_robust_limit_does_not_force_a_fixed_rejection_fraction(self):
        normal = [1.0, 1.1, 0.9, 1.05, 0.95]
        self.assertEqual(_robust_upper_limit(normal, 8.0, 3.5), 8.0)
        self.assertLess(_robust_upper_limit(normal + [30.0], 8.0, 3.5), 30.0)

    def test_robust_solver_removes_a_single_corrupted_pose(self):
        samples, rvecs, tvecs, expected = self._synthetic_dataset()
        corrupted_tvecs = [value.copy() for value in tvecs]
        corrupted_tvecs[7] = corrupted_tvecs[7] + np.array([[0.12], [-0.08], [0.06]])
        arguments = SimpleNamespace(
            disable_outlier_rejection=False,
            min_valid_samples=20,
            outlier_translation_floor_mm=8.0,
            outlier_rotation_floor_deg=1.0,
            outlier_sigma=3.5,
        )
        _, _, _, _, selected, rejected = _robust_handeye_solve(
            samples,
            rvecs,
            corrupted_tvecs,
            "intrinsic_xyz",
            arguments,
            rejection_budget=3,
        )
        self.assertEqual([item["sample_index"] for item in rejected], [8])
        estimate = selected["T_tool0_camera"]
        self.assertLess(np.linalg.norm(estimate[:3, 3] - expected[:3, 3]) * 1000.0, 0.1)
        self.assertLess(math.degrees(rotation_distance_rad(estimate, expected)), 0.01)

    def test_image_coverage_uses_three_by_three_bins(self):
        samples, _, _, _ = self._synthetic_dataset()
        coverage = _image_coverage(samples)
        self.assertGreaterEqual(coverage["image_coverage_bins"], 6)
        self.assertAlmostEqual(coverage["board_area_ratio_min"], 0.08)

    def test_correlated_rotation_is_not_two_independent_axes(self):
        transforms = []
        axis = np.asarray([1.0, 1.0, 0.0]) / math.sqrt(2.0)
        for angle_deg in np.linspace(-35.0, 35.0, 15):
            rotation, _ = cv2.Rodrigues(axis * math.radians(angle_deg))
            transforms.append(make_transform(rotation, [0.0, 0.0, 0.0]))
        metrics = _motion_diversity(transforms)
        component_spans = metrics["motion_rotation_axis_span_deg"]
        principal_spreads = metrics["motion_rotation_principal_spread_deg"]
        self.assertGreater(component_spans[0], 25.0)
        self.assertGreater(component_spans[1], 25.0)
        self.assertLess(principal_spreads[1], 0.01)


if __name__ == "__main__":
    unittest.main()

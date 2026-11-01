import math
import unittest

import numpy as np

from tl_hand_deploy.basket_aruco import (
    basket_teach_status,
    fuse_taught_target,
    marker_transforms_with_depth_translation,
    select_stable_marker_nearest_center,
    translated_taught_target,
)
from tl_hand_deploy.geometry import pose_to_transform, transform_to_pose


class BasketArucoTest(unittest.TestCase):
    def test_intrinsic_xyz_pose_round_trip(self):
        pose = np.asarray([0.31, -0.12, 0.42, -2.043, 1.471, -1.025])
        transform = pose_to_transform(pose[:3], pose[3:], "intrinsic_xyz")
        recovered = transform_to_pose(transform, "intrinsic_xyz")
        self.assertTrue(np.allclose(recovered, pose, atol=1.0e-8))

    def test_two_markers_fuse_same_taught_target(self):
        target = pose_to_transform(
            [0.40, 0.02, 0.18], [-2.043, 1.471, -1.025], "intrinsic_xyz"
        )
        base_markers = {
            0: pose_to_transform([0.35, 0.10, 0.12], [0.0, 0.0, 0.0]),
            1: pose_to_transform([0.43, -0.08, 0.15], [0.0, 0.0, math.pi / 2.0]),
        }
        records = {}
        for marker_id, base_marker in base_markers.items():
            records[str(marker_id)] = {
                "basket_release": {
                    "T_marker_tool0": (np.linalg.inv(base_marker) @ target).tolist()
                }
            }
        fused, used_ids, spread_mm = fuse_taught_target(
            base_markers, records, "basket_release", 5.0
        )
        self.assertEqual(used_ids, [0, 1])
        self.assertLess(spread_mm, 1.0e-6)
        self.assertTrue(np.allclose(fused, target, atol=1.0e-8))

    def test_selects_stable_marker_nearest_image_center(self):
        samples = [
            {1: [318.0, 238.0], 5: [510.0, 330.0]},
            {1: [321.0, 241.0], 5: [511.0, 331.0]},
            {1: [320.0, 240.0], 5: [509.0, 329.0]},
        ]
        selected_id, metrics = select_stable_marker_nearest_center(
            samples, minimum_samples=3, image_width=640, image_height=480
        )
        self.assertEqual(selected_id, 1)
        self.assertEqual(metrics[1]["samples"], 3.0)
        self.assertLess(
            metrics[1]["center_distance_px"], metrics[5]["center_distance_px"]
        )

    def test_ignores_marker_without_enough_samples(self):
        samples = [
            {0: [320.0, 240.0], 3: [500.0, 240.0]},
            {3: [500.0, 240.0]},
            {3: [500.0, 240.0]},
        ]
        selected_id, _ = select_stable_marker_nearest_center(
            samples, minimum_samples=2, image_width=640, image_height=480
        )
        self.assertEqual(selected_id, 3)

    def test_reports_all_six_independent_basket_teach_records(self):
        data = {
            "markers": {
                str(marker_id): {
                    "search_pose": {
                        "T_base_marker": np.eye(4).tolist(),
                        "marker_length_m": 0.058,
                    },
                    "basket_approach": {},
                    "basket_release": {},
                }
                for marker_id in range(6)
            }
        }
        status = basket_teach_status(data)
        self.assertEqual(sorted(status), list(range(6)))
        self.assertTrue(all(record["complete"] for record in status.values()))

    def test_legacy_teach_without_dynamic_reference_is_incomplete(self):
        status = basket_teach_status(
            {
                "markers": {
                    "0": {
                        "search_pose": {},
                        "basket_approach": {},
                        "basket_release": {},
                    }
                }
            },
            marker_ids=[0],
        )
        self.assertFalse(status[0]["dynamic_reference"])
        self.assertFalse(status[0]["complete"])

    def test_depth_translation_replaces_pnp_translation_only(self):
        marker = pose_to_transform([0.1, 0.2, 0.8], [0.1, -0.2, 0.3])
        corners = [np.asarray([[[10.0, 20.0], [30.0, 20.0], [30.0, 40.0], [10.0, 40.0]]])]
        transforms, depths = marker_transforms_with_depth_translation(
            {2: marker}, corners, np.asarray([[2]]), lambda center: [0.02, -0.03, 0.5]
        )
        self.assertTrue(np.allclose(transforms[2][:3, :3], marker[:3, :3]))
        self.assertTrue(np.allclose(transforms[2][:3, 3], [0.02, -0.03, 0.5]))
        self.assertEqual(depths[2], 0.5)

    def test_translation_mode_preserves_taught_pose_orientation(self):
        taught_marker = pose_to_transform([0.3, -0.1, 0.2], [0.0, 0.0, 0.4])
        taught_tcp = [410.0, 20.0, 180.0, -2.043, 1.471, -1.025]
        live_marker = taught_marker.copy()
        live_marker[:3, 3] += [0.04, -0.025, 0.01]
        live_marker[:3, :3] = pose_to_transform(
            [0.0, 0.0, 0.0], [0.3, -0.2, 1.0]
        )[:3, :3]
        record = {
            "search_pose": {
                "T_base_marker": taught_marker.tolist(),
                "marker_length_m": 0.058,
            },
            "basket_release": {"taught_tcp": taught_tcp},
        }
        target, shift = translated_taught_target(
            live_marker, record, "basket_release"
        )
        expected = pose_to_transform(
            np.asarray(taught_tcp[:3]) / 1000.0,
            taught_tcp[3:],
            "intrinsic_xyz",
        )
        self.assertTrue(np.allclose(shift, [0.04, -0.025, 0.01]))
        self.assertTrue(np.allclose(target[:3, 3], expected[:3, 3] + shift))
        self.assertTrue(np.allclose(target[:3, :3], expected[:3, :3]))


if __name__ == "__main__":
    unittest.main()

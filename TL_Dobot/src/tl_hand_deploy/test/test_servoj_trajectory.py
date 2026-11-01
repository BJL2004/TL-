import unittest

import numpy as np

from tl_hand_deploy.reference_compat import (
    minimum_jerk_joint_trajectory,
    servoj_trajectory_point_count,
)


class ServoJTrajectoryTest(unittest.TestCase):
    def test_minimum_jerk_trajectory_reaches_target(self):
        trajectory = minimum_jerk_joint_trajectory([0.0, 0.0], [10.0, -5.0], 100)
        self.assertEqual(len(trajectory), 100)
        self.assertTrue(np.allclose(trajectory[-1], [10.0, -5.0]))
        self.assertLess(abs(trajectory[0][0]), 0.01)

    def test_point_count_scales_with_joint_distance(self):
        short_points, short_duration = servoj_trajectory_point_count(5.0, 100.0, 20.0, 1.0)
        long_points, long_duration = servoj_trajectory_point_count(80.0, 100.0, 20.0, 1.0)
        self.assertEqual(short_points, 100)
        self.assertEqual(short_duration, 1.0)
        self.assertGreater(long_points, short_points)
        self.assertGreater(long_duration, short_duration)


if __name__ == "__main__":
    unittest.main()

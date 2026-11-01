import unittest

from tl_hand_deploy.roh_protocol import classify_operation, release_toward_open


class RohProtocolTest(unittest.TestCase):
    def test_force_reached_is_contact(self):
        self.assertEqual(classify_operation([4, 2, 2, 2, 2, 2], "grasp"), "contact")

    def test_stuck_is_fault(self):
        self.assertEqual(classify_operation([5, 2, 2, 2, 2, 2], "grasp"), "fault")

    def test_release_moves_toward_open(self):
        self.assertEqual(release_toward_open([1000] * 6, [0] * 6, 0.1), [900] * 6)


if __name__ == "__main__":
    unittest.main()

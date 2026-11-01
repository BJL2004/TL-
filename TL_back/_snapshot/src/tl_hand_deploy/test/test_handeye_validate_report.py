import unittest
from types import SimpleNamespace

from tl_hand_deploy.handeye_validate_report import evaluate_records


class HandEyeValidateReportTest(unittest.TestCase):
    @staticmethod
    def _arguments():
        return SimpleNamespace(
            min_samples=9,
            min_workspace_span_mm=100.0,
            max_xy_rms_mm=5.0,
            max_xy_error_mm=8.0,
            max_z_rms_mm=5.0,
            max_z_error_mm=8.0,
        )

    @staticmethod
    def _records(correction):
        records = []
        for index, (x_value, y_value) in enumerate(
            (
                (0, 0),
                (60, 0),
                (120, 0),
                (0, 60),
                (60, 60),
                (120, 60),
                (0, 120),
                (60, 120),
                (120, 120),
            )
        ):
            records.append(
                {
                    "sample": index + 1,
                    "calibration": "/tmp/handeye_accepted.yaml",
                    "base_xy_bias_mm": [0.0, 0.0],
                    "base_z_bias_mm": 0.0,
                    "estimated_object_base_mm": [x_value, y_value, 0.0],
                    "correction_base_mm": list(correction),
                }
            )
        return records

    def test_accepts_small_uncompensated_errors(self):
        result = evaluate_records(self._records([2.0, -1.0, 1.5]), self._arguments())
        self.assertTrue(result["accepted"])

    def test_rejects_large_fixed_bias_instead_of_hiding_it(self):
        result = evaluate_records(self._records([20.0, -10.0, 1.0]), self._arguments())
        self.assertFalse(result["accepted"])
        self.assertFalse(result["quality_checks"]["xy_rms"])

    def test_rejects_records_collected_with_runtime_bias(self):
        records = self._records([1.0, 1.0, 1.0])
        records[0]["base_xy_bias_mm"] = [-20.0, 5.0]
        result = evaluate_records(records, self._arguments())
        self.assertFalse(result["quality_checks"]["uncompensated_measurements"])

    def test_rejects_legacy_records_without_z_bias_provenance(self):
        records = self._records([1.0, 1.0, 1.0])
        records[0].pop("base_z_bias_mm")
        result = evaluate_records(records, self._arguments())
        self.assertFalse(result["quality_checks"]["uncompensated_measurements"])


if __name__ == "__main__":
    unittest.main()

import os
import sys
import tempfile
import unittest


PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)
CALIBRATION_DIR = os.path.join(PROJECT_DIR, "calibration")
if CALIBRATION_DIR not in sys.path:
    sys.path.insert(0, CALIBRATION_DIR)

from analyze_calibration import build_suggestions  # noqa: E402
from calibration_store import CalibrationLog, load_records  # noqa: E402


class CalibrationLogTests(unittest.TestCase):
    def test_jsonl_log_survives_incomplete_last_line(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "data.jsonl")
            log = CalibrationLog("unit_test", path, batch_size=3)
            log.write("sample", value=1)
            log.write("pulses_summary", suggested_pulses_per_meter=[1, 2, 3])
            log.close()
            with open(path, "a", encoding="utf-8") as destination:
                destination.write('{"type":"incomplete"')

            records = load_records(path)
            self.assertTrue(
                any(record.get("type") == "pulses_summary" for record in records)
            )
            self.assertFalse(
                any(record.get("type") == "incomplete" for record in records)
            )

    def test_analyzer_extracts_latest_suggestions(self):
        records = [
            {
                "type": "pulses_summary",
                "suggested_pulses_per_meter": [6001, 6002, 6003],
            },
            {
                "type": "open_loop_summary",
                "suggested_stiction_duty": [300, 310, 320],
                "suggested_feedforward": [5.0, 5.1, 5.2],
                "max_measured_wheel_speed_cm_s": [90, 91, 92],
            },
        ]
        result = build_suggestions(records)
        self.assertEqual(result["pulses_per_meter"], [6001, 6002, 6003])
        self.assertEqual(result["stiction_duty"], [300, 310, 320])
        self.assertEqual(result["feedforward"], [5.0, 5.1, 5.2])


if __name__ == "__main__":
    unittest.main()

import os
import sys
import unittest


PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from approach import (  # noqa: E402
    _legacy_config,
    calc_tof_approach_vy,
    calc_visual_approach_vy,
)
from main_config import ApproachConfig  # noqa: E402


class TennisApproachProfileTests(unittest.TestCase):
    def setUp(self):
        self.config = _legacy_config(ApproachConfig)

    def test_tennis_uses_15_cm_s_minimum_in_visual_and_tof_profiles(self):
        visual_vy = calc_visual_approach_vy(
            0.0,
            ApproachConfig.STOP_Y_THRESHOLD_PX,
            self.config,
            obj_class=3,
        )
        tof_vy = calc_tof_approach_vy(
            True,
            ApproachConfig.TENNIS_STOP_DISTANCE_MM + 1.0,
            self.config,
            ApproachConfig.TENNIS_STOP_DISTANCE_MM,
            obj_class=3,
        )
        ordinary_visual_vy = calc_visual_approach_vy(
            0.0,
            ApproachConfig.STOP_Y_THRESHOLD_PX,
            self.config,
            obj_class=1,
        )

        self.assertEqual(visual_vy, 15.0)
        self.assertEqual(
            ordinary_visual_vy,
            ApproachConfig.MIN_APPROACH_SPEED_CM_S,
        )
        self.assertGreater(tof_vy, 15.0)
        self.assertLess(tof_vy, 16.0)

    def test_tennis_visual_slow_start_uses_its_own_config_value(self):
        self.config["tennis_approach_y_slow_start"] = 80.0

        tennis_vy = calc_visual_approach_vy(
            0.0, 60.0, self.config, obj_class=3
        )
        ordinary_vy = calc_visual_approach_vy(
            0.0, 60.0, self.config, obj_class=1
        )

        self.assertEqual(tennis_vy, ApproachConfig.APPROACH_SPEED_CM_S)
        self.assertLess(ordinary_vy, tennis_vy)


if __name__ == "__main__":
    unittest.main()

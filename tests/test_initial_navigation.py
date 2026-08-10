import math
import os
import sys
import unittest


PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from main import MainTaskController, MainTaskState  # noqa: E402
from main_config import (  # noqa: E402
    INITIAL_X_CM,
    INITIAL_Y_CM,
    MissionConfig,
)


class InitialNavigationTests(unittest.TestCase):
    def test_initial_waypoint_keeps_90_degree_heading_and_blind_thresholds(self):
        controller = MainTaskController()

        self.assertEqual((INITIAL_X_CM, INITIAL_Y_CM), (25.0, -5.0))
        self.assertEqual(
            MissionConfig.INITIAL_WAYPOINT,
            (160.0, 70.0, 90.0),
        )
        self.assertAlmostEqual(
            controller.nav_turn.target_heading_rad,
            math.pi / 2.0,
        )
        self.assertEqual(MissionConfig.VISUAL_ENABLE_MIN_X_CM, 50.0)
        self.assertEqual(MissionConfig.VISUAL_ENABLE_MIN_Y_CM, 50.0)

    def test_initial_waypoint_transition_uses_hard_stop(self):
        controller = MainTaskController()
        target_x, target_y = MissionConfig.INITIAL_WAYPOINT[:2]
        heading = math.atan2(target_y, target_x)
        controller.state = MainTaskState.NAVIGATE

        result = controller.step(None, (target_x, target_y, heading))

        self.assertEqual(controller.state, MainTaskState.WAIT_TARGET)
        self.assertEqual(result.reason, "waypoint_reached_waiting_for_target")
        self.assertTrue(result.debug["hard_stop"])


if __name__ == "__main__":
    unittest.main()

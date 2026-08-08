import math
import os
import sys
import unittest


PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from main import MainTaskController, MainTaskState  # noqa: E402
from main_config import MissionConfig  # noqa: E402


class InitialNavigationTests(unittest.TestCase):
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

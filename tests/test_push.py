import math
import os
import sys
import unittest


PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from main_config import PushConfig  # noqa: E402
from push import PushController  # noqa: E402


class FastPushConfig(PushConfig):
    PUSH_SETTLE_S = 0.0
    PUSH_RAMP_S = 0.1
    PUSH_DURATION_S = 1.0
    YELLOW_STOP_DELAY_S = 0.1
    TARGET_LOSS_TIMEOUT_S = 0.2


def target(x=160.0, y=170.0, found=True):
    return {"found": found, "x": x, "y": y, "class_id": 1}


class PushControllerTests(unittest.TestCase):
    def _started(self):
        controller = PushController(FastPushConfig)
        controller.start(0.0, class_id=1)
        return controller

    def test_pushes_forward_and_keeps_locked_heading(self):
        controller = self._started()
        result = controller.step(target(), 30.0, math.radians(-10.0), dt=0.1)
        self.assertGreater(result.command[1], 0.0)
        self.assertGreater(result.command[2], 0.0)

    def test_visual_target_error_corrects_lateral_axis(self):
        controller = self._started()
        result = controller.step(target(x=200.0), None, 0.0, dt=0.1)
        self.assertGreater(result.command[0], 0.0)

    def test_obstacle_triggers_avoidance(self):
        controller = self._started()
        result = controller.step(
            target(), None, 0.0,
            hazard={"found": True, "type": 1, "x": 100.0, "y": 90.0},
            dt=0.1,
        )
        self.assertEqual(result.reason, "push_running_avoid_track")
        self.assertFalse(result.done)
        self.assertFalse(result.failed)
        self.assertTrue(controller.active)

    def test_yellow_line_completes_after_delay(self):
        controller = self._started()
        controller.step(
            target(), None, 0.0,
            hazard={"found": True, "type": 2, "x": 160.0, "y": 100.0},
            dt=0.1,
        )
        result = controller.step(target(), None, 0.0, dt=0.1)
        self.assertTrue(result.done)
        self.assertEqual(result.reason, "push_yellow_line")

    def test_lost_target_stops_safely_after_timeout(self):
        controller = self._started()
        controller.step(target(found=False), None, 0.0, dt=0.1)
        controller.step(target(found=False), None, 0.0, dt=0.1)
        result = controller.step(target(found=False), None, 0.0, dt=0.1)
        self.assertTrue(result.failed)
        self.assertEqual(result.reason, "push_target_lost")

    def test_push_duration_completes(self):
        controller = self._started()
        result = None
        for _ in range(11):
            result = controller.step(target(), None, 0.0, dt=0.1)
        self.assertTrue(result.done)
        self.assertEqual(result.reason, "push_duration_complete")


if __name__ == "__main__":
    unittest.main()

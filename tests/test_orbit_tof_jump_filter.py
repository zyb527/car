import math
import os
import sys
import unittest


PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from main_config import OrbitConfig  # noqa: E402
from orbit import OrbitController  # noqa: E402


def target():
    return {"found": True, "x": 160.0, "y": 140.0, "class_id": 1}


class OrbitToFJumpFilterTests(unittest.TestCase):
    def _controller(self):
        controller = OrbitController(OrbitConfig)
        controller.start_absolute(
            0.0,
            math.pi / 2.0,
            direction=-1.0,
            orbit_radius_mm=200.0,
        )
        return controller

    def test_jump_over_90_mm_reuses_previous_valid_tof_frame(self):
        controller = self._controller()
        controller._filtered_tof(200.0)

        found, distance, rejected = controller._filtered_tof(291.0)

        self.assertTrue(found)
        self.assertTrue(rejected)
        self.assertEqual(distance, 200.0)
        self.assertEqual(controller.last_valid_tof_mm, 200.0)

    def test_change_of_90_mm_or_less_is_accepted(self):
        controller = self._controller()
        controller._filtered_tof(200.0)

        found, distance, rejected = controller._filtered_tof(290.0)

        self.assertTrue(found)
        self.assertFalse(rejected)
        self.assertEqual(distance, 290.0)
        self.assertEqual(controller.last_valid_tof_mm, 290.0)

    def test_running_debug_only_keeps_control_required_fields(self):
        controller = self._controller()

        result = controller.step(target(), 200.0, 0.0)

        self.assertEqual(
            set(result.debug),
            {"phase", "tof_jump_rejected", "immediate_command"},
        )


if __name__ == "__main__":
    unittest.main()

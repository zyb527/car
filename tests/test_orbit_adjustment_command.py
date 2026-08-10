import math
import os
import sys
import unittest


PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from main_config import OrbitConfig  # noqa: E402
from orbit import OrbitController  # noqa: E402


def target(x, y):
    return {"found": True, "x": float(x), "y": float(y), "class_id": 1}


class OrbitAdjustmentCommandTests(unittest.TestCase):
    def _controller_at_final_heading(self):
        controller = OrbitController(OrbitConfig)
        controller.start_absolute(
            math.pi / 2.0,
            math.pi / 2.0,
            class_id=1,
        )
        return controller

    def test_adjustment_tolerances_are_15_pixels(self):
        self.assertEqual(OrbitConfig.ORBIT_STOP_X_ERROR_PX, 15.0)
        self.assertEqual(OrbitConfig.ORBIT_FINAL_ALIGN_X_ERROR_PX, 15.0)
        self.assertEqual(OrbitConfig.ORBIT_FINAL_ALIGN_Y_ERROR_PX, 15.0)
        self.assertAlmostEqual(
            OrbitConfig.ORBIT_ALIGN_MIN_W_ERROR_RAD,
            math.radians(4.0),
        )

    def test_align_and_close_in_use_immediate_commands(self):
        controller = self._controller_at_final_heading()
        entered_align = controller.step(target(50.0, 166.0), 150.0, math.pi / 2.0)
        self.assertTrue(entered_align.debug["immediate_command"])

        aligning = controller.step(target(70.0, 166.0), 150.0, math.pi / 2.0)
        self.assertTrue(aligning.debug["immediate_command"])

        entered_close_in = controller.step(target(100.0, 166.0), 150.0, math.pi / 2.0)
        self.assertTrue(entered_close_in.debug["immediate_command"])

        closing = controller.step(target(70.0, 150.0), 150.0, math.pi / 2.0)
        self.assertTrue(closing.debug["immediate_command"])


if __name__ == "__main__":
    unittest.main()

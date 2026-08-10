import math
import os
import sys
import unittest


PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from main_config import OrbitConfig  # noqa: E402
from orbit import OrbitController  # noqa: E402


class OrbitClassTargetTests(unittest.TestCase):
    def test_align_and_close_in_targets_follow_object_class(self):
        expected = {
            1: (100.0, (55.0, 145.0)),
            2: (100.0, (70.0, 145.0)),
            3: (90.0, (70.0, 140.0)),
            4: (100.0, (70.0, 142.0)),
            5: (100.0, (70.0, 142.0)),
        }
        for class_id, (align_x, close_target) in expected.items():
            with self.subTest(class_id=class_id):
                controller = OrbitController(OrbitConfig)
                controller.start_absolute(
                    0.0,
                    math.pi / 2.0,
                    class_id=class_id,
                )
                self.assertEqual(
                    (controller.rod_target_x_px, controller.rod_target_y_px),
                    close_target,
                )
                self.assertEqual(controller.align_target_x_px, align_x)


if __name__ == "__main__":
    unittest.main()

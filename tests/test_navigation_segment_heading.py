import math
import os
import sys
import unittest


PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from main_config import NavigationConfig  # noqa: E402
from navigation import CoordinatePatrolController  # noqa: E402


class NavigationSegmentHeadingTests(unittest.TestCase):
    def test_two_coordinate_waypoint_holds_segment_heading_near_endpoint(self):
        controller = CoordinatePatrolController([(100.0, 70.0)], NavigationConfig)
        controller.reset(0.0, 0.0)
        segment_heading = math.atan2(70.0, 100.0)

        # Six centimetres to the left of the endpoint is outside both arrival
        # tolerances. Pointing at the endpoint from here would incorrectly ask
        # for a roughly 90-degree terminal turn.
        normal_x = -math.sin(segment_heading)
        normal_y = math.cos(segment_heading)
        pose = (
            100.0 + 6.0 * normal_x,
            70.0 + 6.0 * normal_y,
            segment_heading,
        )
        result = controller.step(pose)

        self.assertFalse(result.done)
        self.assertAlmostEqual(controller.target_heading_rad(pose), segment_heading)
        self.assertEqual(result.command[2], 0.0)

    def test_explicit_waypoint_heading_still_overrides_segment_heading(self):
        controller = CoordinatePatrolController(
            [(100.0, 70.0, 90.0)], NavigationConfig
        )
        controller.reset(0.0, 0.0)

        self.assertAlmostEqual(
            controller.target_heading_rad((99.0, 65.0, 0.0)),
            math.pi / 2.0,
        )


if __name__ == "__main__":
    unittest.main()

import math
import os
import sys
import unittest


CAR_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if CAR_DIR not in sys.path:
    sys.path.insert(0, CAR_DIR)

from main_config import NavigationConfig  # noqa: E402
from navigation import (  # noqa: E402
    ClockwiseTurnController,
    CounterclockwiseTurnController,
    HeadingTurnController,
)


class FastTimeoutNavigationConfig(NavigationConfig):
    TURN_MAX_TIME_S = 0.05


class NavigationTurnTimeoutTests(unittest.TestCase):
    def test_absolute_heading_turn_stops_after_maximum_time(self):
        controller = HeadingTurnController(FastTimeoutNavigationConfig)
        controller.start(0.0)

        self.assertFalse(controller.step(math.pi, 0.0, 0.02).failed)
        result = controller.step(math.pi, 0.0, 0.03)

        self.assertTrue(result.failed)
        self.assertEqual(result.reason, "heading_turn_timeout")
        self.assertEqual(result.command, (0.0, 0.0, 0.0))

    def test_clockwise_turn_stops_after_maximum_time(self):
        controller = ClockwiseTurnController(FastTimeoutNavigationConfig)
        controller.start(0.0)

        self.assertFalse(controller.step(0.0, 0.0, 0.02).failed)
        result = controller.step(0.0, 0.0, 0.03)

        self.assertTrue(result.failed)
        self.assertEqual(result.reason, "clockwise_turn_timeout")
        self.assertEqual(result.command, (0.0, 0.0, 0.0))

    def test_counterclockwise_turn_stops_after_maximum_time(self):
        controller = CounterclockwiseTurnController(FastTimeoutNavigationConfig)
        controller.start(0.0)

        self.assertFalse(controller.step(0.0, 0.0, 0.02).failed)
        result = controller.step(0.0, 0.0, 0.03)

        self.assertTrue(result.failed)
        self.assertEqual(result.reason, "counterclockwise_turn_timeout")
        self.assertEqual(result.command, (0.0, 0.0, 0.0))

    def test_success_at_limit_wins_over_timeout(self):
        controller = HeadingTurnController(FastTimeoutNavigationConfig)
        controller.start(0.0)

        result = controller.step(0.0, 0.0, 0.05)

        self.assertTrue(result.done)
        self.assertFalse(result.failed)
        self.assertEqual(result.reason, "heading_reached")


if __name__ == "__main__":
    unittest.main()

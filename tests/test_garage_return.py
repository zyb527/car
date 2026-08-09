import math
import os
import sys
import unittest


PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from garage import GarageController, GarageState  # noqa: E402
from main import MainTaskController, MainTaskState  # noqa: E402
from main_config import GarageConfig  # noqa: E402


class FakeVision:
    def __init__(self):
        self.yellow_events = []

    def set_yellow_line(self, enabled):
        self.yellow_events.append(bool(enabled))


class GarageReturnTests(unittest.TestCase):
    def test_bear_turn_to_180_is_always_counterclockwise(self):
        for class_id, heading in ((4, 0.01), (5, -0.01)):
            with self.subTest(class_id=class_id, heading=heading):
                garage = GarageController(GarageConfig)
                garage.start(FakeVision())
                garage.last_pushed_class_id = class_id

                result = garage.step(
                    (160.0, 200.0, heading), None, 0.0, 0.02
                )

                self.assertEqual(garage.state, GarageState.TURN_180)
                self.assertEqual(result.reason, "turning_180")
                self.assertGreater(result.command[2], 0.0)

    def test_tennis_shifts_until_world_y_decreases_50_cm(self):
        vision = FakeVision()
        garage = GarageController(GarageConfig)
        garage.start(vision)
        garage.last_pushed_class_id = 3

        turned = garage.step(
            (160.0, 200.0, math.pi), None, 0.0, 0.02
        )
        self.assertEqual(turned.reason, "turn_complete")
        self.assertEqual(garage.state, GarageState.TENNIS_LEFT_SHIFT)
        self.assertEqual(vision.yellow_events, [False])

        shifting = garage.step(
            (160.0, 151.0, math.pi), None, 0.0, 0.02
        )
        self.assertEqual(shifting.reason, "tennis_left_shifting")
        self.assertLess(shifting.command[0], 0.0)
        self.assertEqual(shifting.command[1:], (0.0, 0.0))

        completed = garage.step(
            (160.0, 150.0, math.pi), None, 0.0, 0.02
        )
        self.assertEqual(completed.reason, "tennis_left_shift_complete")
        self.assertEqual(garage.state, GarageState.FORWARD_FIND_YELLOW)
        self.assertEqual(vision.yellow_events, [False, True])

    def test_counterclockwise_turn_stops_after_small_overshoot(self):
        garage = GarageController(GarageConfig)
        garage.start(FakeVision())
        garage.last_pushed_class_id = 4
        garage.step((160.0, 200.0, 0.0), None, 0.0, 0.02)

        braking = garage.step(
            (160.0, 200.0, math.pi - 0.02), None, 0.5, 0.02
        )
        self.assertEqual(braking.command[2], 0.0)
        self.assertEqual(garage.state, GarageState.TURN_180)

        completed = garage.step(
            (160.0, 200.0, -math.pi + 0.03), None, 0.0, 0.02
        )
        self.assertEqual(completed.reason, "turn_complete")
        self.assertNotEqual(garage.state, GarageState.TURN_180)

    def test_main_passes_the_last_pushed_class_to_garage(self):
        class CaptureGarage:
            def __init__(self):
                self.started_with = None

            def start(self, vision_receiver):
                self.started_with = vision_receiver

        vision = FakeVision()
        mission = MainTaskController(vision)
        mission.locked_class_id = 3
        mission._finish_push_round()
        mission.garage = CaptureGarage()

        mission._start_garage()

        self.assertEqual(mission.state, MainTaskState.GARAGE)
        self.assertEqual(mission.garage.started_with, vision)
        self.assertEqual(mission.garage.last_pushed_class_id, 3)


if __name__ == "__main__":
    unittest.main()

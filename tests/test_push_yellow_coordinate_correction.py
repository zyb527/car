import os
import sys
import unittest


PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from control import MotionStep  # noqa: E402
from main import (  # noqa: E402
    MainTaskController,
    MainTaskState,
    _apply_push_coordinate_correction,
)
from main_config import PushConfig  # noqa: E402
from push import PushController  # noqa: E402


class FastYellowConfig(PushConfig):
    PUSH_DURATION_S = 5.0
    YELLOW_STOP_DELAY_S = 0.1


def target():
    return {
        "found": True,
        "x": PushConfig.TARGET_CENTER_X_PX,
        "y": PushConfig.TARGET_Y_PX,
        "class_id": 1,
    }


def yellow(y):
    return {"found": True, "hazard_type": 6, "x": 160.0, "y": y}


class FakeOdometry:
    def __init__(self):
        self.reset_position_calls = []

    def reset_position(self, x_cm, y_cm):
        self.reset_position_calls.append((x_cm, y_cm))


class FakeVision:
    def __init__(self):
        self.yellow_events = []

    def set_yellow_line(self, enabled):
        self.yellow_events.append(bool(enabled))

    def unlock_target(self):
        pass


class EventPush:
    def step(self, *args, **kwargs):
        return MotionStep(
            (0.0, 1.0, 0.0),
            debug={
                "yellow_coordinate_correction": {
                    "phase": "first",
                    "axis": "x",
                    "value_cm": 213.0,
                    "disable_yellow_line": True,
                }
            },
        )


class PushYellowCoordinateCorrectionTests(unittest.TestCase):
    def _started(self, class_id):
        controller = PushController(FastYellowConfig)
        controller.start(0.0, class_id=class_id)
        return controller

    def test_first_correction_matches_each_object_type(self):
        cases = (
            (1, "x", 27.0, False),
            (3, "y", 213.0, True),
            (4, "x", 293.0, False),
        )
        for class_id, axis, value_cm, disable_yellow in cases:
            with self.subTest(class_id=class_id):
                controller = self._started(class_id)
                result = controller.step(
                    target(), None, 0.0, hazard=yellow(101.0), dt=0.01
                )
                correction = result.debug["yellow_coordinate_correction"]
                self.assertEqual(correction["phase"], "first")
                self.assertEqual(correction["axis"], axis)
                self.assertEqual(correction["value_cm"], value_cm)
                self.assertEqual(
                    correction["disable_yellow_line"], disable_yellow
                )

    def test_first_frame_above_second_threshold_only_corrects_once(self):
        controller = self._started(1)
        first = controller.step(
            target(), None, 0.0, hazard=yellow(211.0), dt=0.01
        )
        self.assertEqual(
            first.debug["yellow_coordinate_correction"]["phase"], "first"
        )
        self.assertFalse(controller.yellow_second_correction_done)

        second = controller.step(
            target(), None, 0.0, hazard=yellow(211.0), dt=0.01
        )
        self.assertEqual(
            second.debug["yellow_coordinate_correction"]["phase"], "second"
        )

    def test_second_correction_for_sandbag_and_bear(self):
        cases = ((1, 5.0), (4, 315.0))
        for class_id, value_cm in cases:
            with self.subTest(class_id=class_id):
                controller = self._started(class_id)
                controller.step(
                    target(), None, 0.0, hazard=yellow(101.0), dt=0.01
                )
                result = controller.step(
                    target(), None, 0.0, hazard=yellow(211.0), dt=0.01
                )
                correction = result.debug["yellow_coordinate_correction"]
                self.assertEqual(correction["phase"], "second")
                self.assertEqual(correction["axis"], "x")
                self.assertEqual(correction["value_cm"], value_cm)
                self.assertTrue(correction["disable_yellow_line"])

    def test_tennis_disables_after_first_and_never_gets_second_correction(self):
        controller = self._started(3)
        first = controller.step(
            target(), None, 0.0, hazard=yellow(101.0), dt=0.01
        )
        self.assertTrue(
            first.debug["yellow_coordinate_correction"]["disable_yellow_line"]
        )
        second = controller.step(
            target(), None, 0.0, hazard=yellow(211.0), dt=0.01
        )
        self.assertNotIn("yellow_coordinate_correction", second.debug)
        self.assertFalse(controller.yellow_second_correction_done)

    def test_second_correction_does_not_restart_first_delay(self):
        controller = self._started(1)
        controller.step(
            target(), None, 0.0, hazard=yellow(101.0), dt=0.05
        )
        second = controller.step(
            target(), None, 0.0, hazard=yellow(211.0), dt=0.06
        )
        self.assertFalse(second.done)
        self.assertEqual(
            second.debug["yellow_coordinate_correction"]["phase"], "second"
        )
        stopped = controller.step(target(), None, 0.0, dt=0.05)
        self.assertTrue(stopped.done)
        self.assertEqual(stopped.reason, "push_yellow_line_hard_stop")

    def test_missing_second_correction_does_not_block_push_completion(self):
        controller = self._started(1)
        controller.step(
            target(), None, 0.0, hazard=yellow(101.0), dt=0.01
        )
        stopped = controller.step(target(), None, 0.0, dt=0.11)
        self.assertTrue(stopped.done)
        self.assertEqual(stopped.reason, "push_yellow_line_hard_stop")
        self.assertFalse(controller.yellow_second_correction_done)

    def test_coordinate_application_preserves_other_axis_and_heading(self):
        odometry = FakeOdometry()
        corrected = _apply_push_coordinate_correction(
            odometry,
            (100.0, 200.0, 0.7),
            {"axis": "x", "value_cm": 213.0},
        )
        self.assertEqual(corrected, (213.0, 200.0, 0.7))
        self.assertEqual(odometry.reset_position_calls, [(213.0, 200.0)])

    def test_tennis_event_closes_camera_yellow_detection_in_main_controller(self):
        vision = FakeVision()
        controller = MainTaskController(vision)
        controller.state = MainTaskState.PUSH
        controller.push = EventPush()
        controller.step(target(), (0.0, 0.0, 0.0), tof_distance_mm=30.0)
        self.assertEqual(vision.yellow_events[-1], False)


if __name__ == "__main__":
    unittest.main()

import os
import sys
import unittest


PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from control import MotionStep  # noqa: E402
from main import MainTaskController, MainTaskState  # noqa: E402
from main_config import MissionConfig, PushConfig  # noqa: E402
from push import PushController, State  # noqa: E402


class FakeVision:
    def __init__(self):
        self.unlock_count = 0
        self.yellow_events = []

    def set_yellow_line(self, enabled):
        self.yellow_events.append(bool(enabled))

    def unlock_target(self):
        self.unlock_count += 1

    def target_event(self, target, allowed_class_ids, locked_class_id=0):
        if not target or not target.get("found", False):
            return None
        class_id = int(target["class_id"])
        if class_id not in allowed_class_ids:
            return None
        if locked_class_id and class_id != locked_class_id:
            return None
        return {
            "x": float(target["x"]),
            "y": float(target["y"]),
            "class_id": class_id,
        }


class DoneOrbit:
    active = True

    def step(self, *args, **kwargs):
        return MotionStep.stop("orbit_complete", done=True)


class DonePush:
    def step(self, *args, **kwargs):
        return MotionStep.stop("push_duration_complete", done=True)

    def reset(self):
        pass


def target():
    return {
        "found": True,
        "x": PushConfig.TARGET_CENTER_X_PX,
        "y": PushConfig.TARGET_Y_PX,
        "class_id": 1,
    }


class PushCameraFilterTests(unittest.TestCase):
    def _controller(self):
        vision = FakeVision()
        controller = MainTaskController(vision)
        vision.unlock_count = 0
        vision.yellow_events = []
        controller.visual_target_gate_open = True
        controller.locked_class_id = 1
        controller.target_heading_rad = 0.0
        return controller, vision

    def test_push_reference_and_avoidance_center_are_independent(self):
        self.assertEqual(
            (PushConfig.TARGET_CENTER_X_PX, PushConfig.TARGET_Y_PX),
            (30.0, 75.0),
        )
        self.assertEqual(PushConfig.AVOID_CENTER_X_PX, 65.0)

    def test_orbit_to_push_keeps_target_filter_locked(self):
        controller, vision = self._controller()
        controller.state = MainTaskState.ORBIT
        controller.orbit = DoneOrbit()

        result = controller.step(target(), (0.0, 0.0, 0.0), 30.0)

        self.assertEqual(result.reason, "orbit_to_push")
        self.assertEqual(controller.state, MainTaskState.PUSH)
        self.assertEqual(vision.unlock_count, 0)
        self.assertEqual(vision.yellow_events, [True])

    def test_completed_push_unlocks_filter(self):
        controller, vision = self._controller()
        controller.state = MainTaskState.PUSH
        controller.push = DonePush()

        controller.step(target(), (0.0, 0.0, 0.0), 30.0)

        self.assertEqual(vision.unlock_count, 1)
        self.assertEqual(vision.yellow_events, [False])

    def test_yellow_stop_delay_ignores_later_obstacle_frames(self):
        controller = PushController(PushConfig)
        controller.start(0.0, class_id=1)
        yellow = {
            "found": True,
            "hazard_type": PushConfig.HAZARD_YELLOW,
            "x": 65.0,
            "y": 101.0,
            "frame_sequence": 1,
            "frame_ms": 0,
        }
        obstacle = {
            "found": True,
            "hazard_type": PushConfig.HAZARD_OBSTACLE,
            "x": 65.0,
            "y": 90.0,
            "frame_sequence": 2,
            "frame_ms": 50,
        }

        controller.step(target(), 30.0, 0.0, hazard=yellow, dt=0.05)
        during_delay = controller.step(
            target(), 30.0, 0.0, hazard=obstacle, dt=0.10
        )

        self.assertEqual(controller.state, State.YELLOW_DELAY)
        self.assertEqual(during_delay.reason, "push_running_yellow_delay")
        self.assertEqual(during_delay.debug["avoid_gear"], 0)

        controller.step(target(), 30.0, 0.0, hazard=obstacle, dt=0.10)
        stopped = controller.step(
            target(), 30.0, 0.0, hazard=obstacle, dt=0.11
        )
        self.assertTrue(stopped.done)
        self.assertEqual(stopped.reason, "push_yellow_line_hard_stop")


if __name__ == "__main__":
    unittest.main()

import math
import os
import sys
import unittest


PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from control import MotionStep  # noqa: E402
from main import MainTaskController, MainTaskState  # noqa: E402
from main_config import MissionConfig  # noqa: E402


class FakeVision:
    def __init__(self):
        self.unlocked = 0
        self.locked_ids = []

    def set_yellow_line(self, enabled):
        pass

    def unlock_target(self):
        self.unlocked += 1

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

    def lock_target(self, target, allowed_class_ids):
        event = self.target_event(target, allowed_class_ids)
        if event is not None:
            self.locked_ids.append(event["class_id"])
        return event


class SearchThenRunApproach:
    def __init__(self):
        self.search_emitted = False
        self.last_target = None

    def reset(self):
        pass

    def step(self, target, tof_distance_mm, dt=0.02):
        self.last_target = target
        if not self.search_emitted:
            self.search_emitted = True
            return MotionStep.stop("spin_search", failed=True)
        return MotionStep(reason="approach_reacquired")


class ApproachReacquireAnyIdTests(unittest.TestCase):
    def test_lost_id_can_be_replaced_by_another_valid_id(self):
        vision = FakeVision()
        mission = MainTaskController(vision)
        mission.state = MainTaskState.APPROACH
        mission.locked_class_id = 1
        mission.target_heading_rad = math.radians(
            MissionConfig.CLASS_HEADING_DEG[1]
        )
        mission.approach = SearchThenRunApproach()
        pose = (0.0, 0.0, 0.0)

        searching = mission.step(None, pose, tof_distance_mm=300.0)

        self.assertEqual(searching.reason, "approach_spin_search")
        self.assertGreater(searching.command[2], 0.0)
        self.assertEqual(vision.unlocked, 1)

        new_target = {
            "found": True,
            "x": 160.0,
            "y": 100.0,
            "class_id": 4,
        }
        reacquired = mission.step(new_target, pose, tof_distance_mm=300.0)

        self.assertEqual(reacquired.reason, "approach_reacquired")
        self.assertEqual(mission.locked_class_id, 4)
        self.assertAlmostEqual(
            mission.target_heading_rad,
            math.radians(MissionConfig.CLASS_HEADING_DEG[4]),
        )
        self.assertEqual(vision.locked_ids, [4])
        self.assertIsNone(mission.target_search_state)
        self.assertIs(mission.approach.last_target, new_target)


if __name__ == "__main__":
    unittest.main()

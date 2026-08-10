import math
import os
import sys
import unittest


PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from control import MotionStep  # noqa: E402
from main import MainTaskController, MainTaskState  # noqa: E402
from main_config import MissionConfig, NavigationConfig  # noqa: E402
from navigation import (  # noqa: E402
    CoordinatePatrolController,
    PostPushPointSearchController,
    PostPushPointSearchState,
)


class FakeVision:
    def __init__(self):
        self.locked_ids = []

    def set_yellow_line(self, enabled):
        pass

    def unlock_target(self):
        pass

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


class DonePatrol:
    def step(self, pose, yaw_rate_rad_s):
        return MotionStep.stop("waypoint_reached", done=True)


class PostPushPointSearchTests(unittest.TestCase):
    def _controller(self):
        return PostPushPointSearchController(
            NavigationConfig,
            MissionConfig.POST_PUSH_POINT_WAIT_S,
            MissionConfig.POST_PUSH_FORWARD_SPEED_CM_S,
            MissionConfig.POST_PUSH_FORWARD_MAX_DISTANCE_CM,
        )

    def test_aligns_waits_then_drives_forward_at_configured_speed(self):
        controller = self._controller()
        pose = (160.0, 180.0, 0.0)
        controller.start(pose, -90.0)

        turning = controller.step(pose, 0.0, 0.02)
        self.assertEqual(turning.command[:2], (0.0, 0.0))
        self.assertLess(turning.command[2], 0.0)

        controller.step((160.0, 180.0, -math.pi / 2.0), 0.0, 0.02)
        self.assertEqual(controller.state, PostPushPointSearchState.WAIT)

        controller.step((160.0, 180.0, -math.pi / 2.0), 0.0, 0.5)
        wait_done = controller.step(
            (160.0, 180.0, -math.pi / 2.0), 0.0, 0.5
        )
        self.assertEqual(
            wait_done.reason, "post_push_point_wait_complete_starting_forward"
        )

        forward = controller.step(
            (160.0, 180.0, -math.pi / 2.0), 0.0, 0.02
        )
        self.assertEqual(
            forward.command, (0.0, MissionConfig.POST_PUSH_FORWARD_SPEED_CM_S, 0.0)
        )

    def test_stops_after_exact_forward_distance_limit(self):
        controller = self._controller()
        pose = (160.0, 180.0, -math.pi / 2.0)
        controller.start(pose, -90.0)
        controller.step(pose, 0.0, 0.02)
        controller.step(pose, 0.0, 1.0)
        controller.step(pose, 0.0, 0.02)

        done = controller.step((160.0, 80.0, -math.pi / 2.0), 0.0, 0.02)

        self.assertTrue(done.done)
        self.assertEqual(done.reason, "post_push_point_forward_distance_complete")
        self.assertAlmostEqual(done.debug["forward_progress_cm"], 100.0)

    def test_main_selects_heading_from_the_just_pushed_class(self):
        mission = MainTaskController(FakeVision())
        mission.state = MainTaskState.POST_PUSH_NAVIGATE
        mission.post_push_class_id = 3
        mission.patrol = DonePatrol()
        mission.visual_target_gate_open = True
        mission.post_push_visual_gate = None

        result = mission.step(
            None,
            (160.0, 180.0, 0.0),
            tof_distance_mm=300.0,
        )

        self.assertEqual(
            result.reason, "post_push_waypoint_reached_starting_point_search"
        )
        self.assertEqual(mission.state, MainTaskState.POST_PUSH_POINT_SEARCH)
        self.assertAlmostEqual(
            mission.post_push_search.target_heading_rad, -math.pi / 2.0
        )

    def test_post_push_return_waypoints_include_class_specific_heading(self):
        expected = {
            1: (100.0, 120.0, 0.0),
            2: (100.0, 120.0, 0.0),
            3: (190.0, 170.0, -90.0),
            4: (220.0, 120.0, 180.0),
            5: (220.0, 120.0, 180.0),
        }

        self.assertEqual(MissionConfig.POST_PUSH_WAYPOINT_BY_CLASS, expected)
        for class_id, waypoint in expected.items():
            with self.subTest(class_id=class_id):
                patrol = CoordinatePatrolController((waypoint,), NavigationConfig)
                patrol.reset(0.0, 0.0)
                self.assertAlmostEqual(
                    patrol.target_heading_rad((0.0, 0.0, 0.0)),
                    math.radians(waypoint[2]),
                )

    def test_any_target_interrupts_post_push_point_search(self):
        vision = FakeVision()
        mission = MainTaskController(vision)
        pose = (160.0, 180.0, -math.pi / 2.0)
        mission.state = MainTaskState.POST_PUSH_POINT_SEARCH
        mission.visual_target_gate_open = True
        mission.post_push_search.start(pose, -90.0)
        target = {"found": True, "x": 150.0, "y": 90.0, "class_id": 4}

        mission.step(target, pose, tof_distance_mm=300.0)

        self.assertEqual(mission.state, MainTaskState.APPROACH)
        self.assertEqual(mission.locked_class_id, 4)
        self.assertEqual(vision.locked_ids, [4])


if __name__ == "__main__":
    unittest.main()

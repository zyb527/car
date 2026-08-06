import math
import importlib.util
import os
import sys
import unittest


PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from control import MotionStep  # noqa: E402
from garage import (  # noqa: E402
    GarageController,
    PHASE_ENTER,
    PHASE_FINAL_LATERAL,
    PHASE_FIND_MARKER,
    PHASE_FIND_YELLOW,
    PHASE_PRE_LATERAL,
)
from main_config import (  # noqa: E402
    GarageConfig,
    INITIAL_X_CM,
    INITIAL_Y_CM,
    MissionConfig,
)


MAIN_PATH = os.path.join(PROJECT_DIR, "main.py")
MAIN_SPEC = importlib.util.spec_from_file_location(
    "primary_main_state_machine",
    MAIN_PATH,
)
main_module = importlib.util.module_from_spec(MAIN_SPEC)
MAIN_SPEC.loader.exec_module(main_module)
MainTaskController = main_module.MainTaskController
TaskState = main_module.TaskState


class FakeVision:
    def __init__(self):
        self.filters = []

    def set_target_filter(self, class_id):
        self.filters.append(int(class_id))


def task_target(class_id=1, x=160.0, y=100.0):
    return {
        "found": True,
        "x": x,
        "y": y,
        "class_id": class_id,
    }


class FastTimeoutMissionConfig(MissionConfig):
    GLOBAL_GARAGE_TIMEOUT_S = 1.0


class DonePush:
    def step(self, *args, **kwargs):
        return MotionStep.stop("push_duration_complete", done=True)


class MainStateMachineTests(unittest.TestCase):
    def setUp(self):
        self.vision = FakeVision()
        self.pose = (23.0, -45.0, math.pi / 2.0)
        self.mission = MainTaskController(vision_receiver=self.vision)
        self.mission.reset(self.pose, 100)

    def test_confirmed_initial_pose_and_empty_path_are_safe(self):
        self.assertEqual((INITIAL_X_CM, INITIAL_Y_CM), (23.0, -45.0))
        self.assertEqual(self.mission.state, TaskState.SEARCH)

        result = self.mission.step(
            None,
            None,
            None,
            self.pose,
            vision_online=False,
            now_ms=120,
        )

        self.assertEqual(result.command, (0.0, 0.0, 0.0))
        self.assertTrue(result.debug["hard_stop"])
        self.assertEqual(self.mission.state, TaskState.SEARCH)

    def test_online_search_rotates_and_target_enters_approach(self):
        searching = self.mission.step(
            None,
            {"found": False},
            None,
            self.pose,
            vision_online=True,
            now_ms=120,
        )
        self.assertGreater(searching.command[2], 0.0)

        approaching = self.mission.step(
            task_target(),
            {"found": False},
            400.0,
            self.pose,
            vision_online=True,
            now_ms=140,
        )
        self.assertEqual(self.mission.state, TaskState.APPROACH)
        self.assertGreater(approaching.command[1], 0.0)
        self.assertEqual(self.vision.filters[-1], 1)

    def test_approach_completion_starts_orbit(self):
        self.mission.step(
            task_target(),
            {"found": False},
            400.0,
            self.pose,
            vision_online=True,
            now_ms=120,
        )

        result = self.mission.step(
            task_target(y=185.0),
            {"found": False},
            180.0,
            self.pose,
            vision_online=True,
            now_ms=140,
        )

        self.assertEqual(self.mission.state, TaskState.ORBIT)
        self.assertEqual(result.reason, "approach_to_orbit")
        self.assertTrue(self.mission.orbit.active)

    def test_third_completed_push_enters_garage_and_hard_stops(self):
        self.mission.state = TaskState.PUSH
        self.mission.active_class_id = 1
        self.mission.pushed_objects_count = 2
        self.mission.push = DonePush()

        result = self.mission.step(
            task_target(),
            {"found": False},
            100.0,
            self.pose,
            vision_online=True,
            now_ms=200,
        )

        self.assertEqual(self.mission.state, TaskState.GARAGE)
        self.assertEqual(self.mission.pushed_objects_count, 3)
        self.assertTrue(result.debug["hard_stop"])
        self.assertEqual(self.vision.filters[-1], 5)

    def test_global_timeout_enters_garage(self):
        mission = MainTaskController(
            vision_receiver=self.vision,
            mission_config=FastTimeoutMissionConfig,
        )
        mission.reset(self.pose, 0)

        mission.step(
            None,
            {"found": False},
            None,
            self.pose,
            vision_online=True,
            now_ms=1000,
        )

        self.assertEqual(mission.state, TaskState.GARAGE)

    def test_active_action_camera_loss_enters_fault(self):
        self.mission.state = TaskState.APPROACH

        result = self.mission.step(
            None,
            None,
            None,
            self.pose,
            vision_online=False,
            now_ms=200,
        )

        self.assertTrue(result.failed)
        self.assertEqual(self.mission.state, TaskState.FAULT)


class GarageControllerTests(unittest.TestCase):
    def setUp(self):
        self.garage = GarageController(GarageConfig)
        self.pose = (160.0, 0.0, -math.pi / 2.0)
        self.garage.start(self.pose, 0)

    def _settle_heading(self):
        result = None
        for index in range(7):
            result = self.garage.step(
                self.pose,
                None,
                None,
                dt=0.02,
                now_ms=(index + 1) * 20,
            )
        return result

    def test_center_route_runs_complete_eight_field_garage_sequence(self):
        self._settle_heading()
        self.assertEqual(self.garage.phase, PHASE_FIND_YELLOW)

        self.garage.step(
            self.pose,
            None,
            {"hazard_found": True, "hazard_type": 2, "y": 80.0},
            now_ms=180,
        )
        self.assertEqual(self.garage.phase, PHASE_FIND_MARKER)

        self.garage.step(
            self.pose,
            task_target(class_id=5),
            {"found": False},
            now_ms=200,
        )
        self.assertEqual(self.garage.phase, PHASE_FINAL_LATERAL)

        moved_pose = (185.0, 0.0, -math.pi / 2.0)
        self.garage.step(
            moved_pose,
            task_target(class_id=5),
            {"found": False},
            now_ms=220,
        )
        self.assertEqual(self.garage.phase, PHASE_ENTER)

        completed = self.garage.step(
            moved_pose,
            task_target(class_id=5),
            {"found": False},
            now_ms=2721,
        )
        self.assertTrue(completed.done)
        self.assertTrue(completed.debug["hard_stop"])

    def test_outer_route_uses_pre_lateral_phase(self):
        pose = (80.0, 0.0, -math.pi / 2.0)
        garage = GarageController(GarageConfig)
        garage.start(pose, 0)
        for index in range(7):
            garage.step(
                pose,
                None,
                None,
                dt=0.02,
                now_ms=(index + 1) * 20,
            )

        self.assertEqual(garage.phase, PHASE_PRE_LATERAL)
        command = garage.step(pose, None, None, now_ms=160).command
        self.assertLess(command[0], 0.0)

    def test_missing_yellow_fails_closed_after_timeout(self):
        self._settle_heading()
        failed = self.garage.step(
            self.pose,
            None,
            {"found": False},
            now_ms=8200,
        )

        self.assertTrue(failed.failed)
        self.assertEqual(failed.reason, "garage_yellow_timeout")


if __name__ == "__main__":
    unittest.main()

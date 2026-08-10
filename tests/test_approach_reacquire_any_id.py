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
from navigation import ApproachLossSearchState  # noqa: E402


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


class SearchThenLoseOrbit:
    def __init__(self):
        self.active = True

    def reset(self):
        self.active = False

    def step(
        self, target, tof_distance_mm, heading_rad, yaw_rate_rad_s=0.0, dt=0.02
    ):
        return MotionStep.stop("spin_search", failed=True)


class ApproachReacquireAnyIdTests(unittest.TestCase):
    def test_search_waypoints_match_confirmed_order_and_headings(self):
        self.assertEqual(
            MissionConfig.APPROACH_LOSS_SEARCH_WAYPOINTS,
            (
                (100.0, 160.0, 0.0),
                (100.0, 80.0, 0.0),
                (120.0, 60.0, 90.0),
                (200.0, 60.0, 90.0),
                (220.0, 80.0, 180.0),
                (220.0, 160.0, 180.0),
                (120.0, 180.0, -90.0),
                (200.0, 180.0, -90.0),
            ),
        )

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

    def test_orbit_loss_searches_all_ids_then_restarts_approach(self):
        vision = FakeVision()
        mission = MainTaskController(vision)
        mission.state = MainTaskState.ORBIT
        mission.locked_class_id = 1
        mission.target_heading_rad = math.radians(
            MissionConfig.CLASS_HEADING_DEG[1]
        )
        mission.orbit = SearchThenLoseOrbit()
        mission.approach = SearchThenRunApproach()
        mission.approach.search_emitted = True
        pose = (0.0, 0.0, 0.0)

        searching = mission.step(None, pose, tof_distance_mm=300.0)

        self.assertEqual(searching.reason, "approach_spin_search")
        self.assertEqual(mission.state, MainTaskState.APPROACH_SEARCH)
        self.assertEqual(mission.locked_class_id, 0)
        self.assertIsNone(mission.target_heading_rad)
        self.assertEqual(vision.unlocked, 1)

        new_target = {
            "found": True,
            "x": 160.0,
            "y": 100.0,
            "class_id": 4,
        }
        reacquired = mission.step(new_target, pose, tof_distance_mm=300.0)

        self.assertEqual(reacquired.reason, "approach_reacquired")
        self.assertEqual(mission.state, MainTaskState.APPROACH)
        self.assertEqual(mission.locked_class_id, 4)
        self.assertEqual(vision.locked_ids, [4])

    def _mission_in_approach(self, pose=(0.0, 0.0, 0.0)):
        vision = FakeVision()
        mission = MainTaskController(vision)
        mission.state = MainTaskState.APPROACH
        mission.locked_class_id = 1
        mission.target_heading_rad = math.radians(
            MissionConfig.CLASS_HEADING_DEG[1]
        )
        mission.approach = SearchThenRunApproach()
        first = mission.step(None, pose, tof_distance_mm=300.0)
        self.assertEqual(first.reason, "approach_spin_search")
        return mission, vision

    def _finish_counterclockwise_search_turn(self, mission, x, y):
        headings = (
            0.5,
            1.0,
            1.5,
            2.0,
            2.5,
            3.0,
            -2.7831853071795862,
            -2.2831853071795862,
            -1.7831853071795862,
            -1.2831853071795862,
            -0.7831853071795862,
            -0.28318530717958623,
            0.0,
        )
        result = None
        for heading in headings:
            result = mission.step(
                None,
                (x, y, heading),
                tof_distance_mm=300.0,
                yaw_rate_rad_s=0.0,
            )
        return result

    def test_full_turn_without_target_selects_nearest_search_waypoint(self):
        pose = (205.0, 65.0, 0.0)
        mission, vision = self._mission_in_approach(pose)

        halfway = mission.step(
            None,
            (pose[0], pose[1], math.pi),
            tof_distance_mm=300.0,
        )

        self.assertEqual(mission.state, MainTaskState.APPROACH_SEARCH)
        self.assertEqual(
            mission.approach_search.state, ApproachLossSearchState.TURN
        )
        self.assertEqual(halfway.reason, "approach_spin_search")
        self.assertAlmostEqual(
            halfway.command[2], MissionConfig.SEARCH_W_RAD_S
        )

        # Restart at a clean zero heading so the helper supplies a continuous
        # positive 2*pi trace after the explicit halfway assertion above.
        mission.approach_search.turn.start(0.0)
        completed = self._finish_counterclockwise_search_turn(
            mission, pose[0], pose[1]
        )

        self.assertEqual(
            completed.reason,
            "approach_search_full_turn_complete_starting_nearest_waypoint_turn",
        )
        self.assertEqual(mission.state, MainTaskState.APPROACH_SEARCH)
        self.assertEqual(
            mission.approach_search.state, ApproachLossSearchState.PRETURN
        )
        self.assertEqual(mission.approach_search.patrol.index, 3)
        self.assertEqual(
            mission.approach_search.patrol.current_waypoint(),
            (200.0, 60.0, 90.0),
        )
        self.assertAlmostEqual(
            mission.approach_search.nav_turn.target_heading_rad,
            math.pi / 2.0,
        )
        self.assertEqual(vision.unlocked, 1)

    def test_search_patrol_turns_before_translation_and_wraps_in_order(self):
        pose = (205.0, 65.0, 0.0)
        mission, _ = self._mission_in_approach(pose)
        self._finish_counterclockwise_search_turn(mission, pose[0], pose[1])

        turning_to_fourth = mission.step(
            None, pose, tof_distance_mm=300.0, yaw_rate_rad_s=0.0
        )
        self.assertEqual(turning_to_fourth.command[:2], (0.0, 0.0))
        self.assertGreater(turning_to_fourth.command[2], 0.0)

        mission.step(
            None,
            (pose[0], pose[1], math.pi / 2.0),
            tof_distance_mm=300.0,
            yaw_rate_rad_s=0.0,
        )
        self.assertEqual(
            mission.approach_search.state, ApproachLossSearchState.NAVIGATE
        )

        moving = mission.step(
            None,
            (205.0, 65.0, math.pi / 2.0),
            tof_distance_mm=300.0,
        )
        self.assertNotEqual(moving.command[:2], (0.0, 0.0))
        self.assertAlmostEqual(moving.command[2], 0.0)

        mission.step(
            None,
            (200.0, 60.0, math.pi / 2.0),
            tof_distance_mm=300.0,
        )
        self.assertEqual(mission.approach_search.patrol.index, 4)
        self.assertEqual(
            mission.approach_search.patrol.current_waypoint(),
            (220.0, 80.0, 180.0),
        )

        # Advance through points 5 to 8; the cyclic navigator must wrap to 1.
        mission.approach_search.patrol.advance(220.0, 80.0)
        self.assertEqual(mission.approach_search.patrol.index, 5)
        mission.approach_search.patrol.advance(220.0, 160.0)
        self.assertEqual(mission.approach_search.patrol.index, 6)
        self.assertEqual(
            mission.approach_search.patrol.current_waypoint(),
            (120.0, 180.0, -90.0),
        )
        mission.approach_search.patrol.advance(120.0, 180.0)
        self.assertEqual(mission.approach_search.patrol.index, 7)
        self.assertEqual(
            mission.approach_search.patrol.current_waypoint(),
            (200.0, 180.0, -90.0),
        )
        mission.approach_search.patrol.advance(200.0, 180.0)
        self.assertEqual(mission.approach_search.patrol.index, 0)
        self.assertEqual(
            mission.approach_search.patrol.current_waypoint(),
            (100.0, 160.0, 0.0),
        )

    def test_any_class_can_interrupt_waypoint_search(self):
        pose = (205.0, 65.0, 0.0)
        mission, vision = self._mission_in_approach(pose)
        self._finish_counterclockwise_search_turn(mission, pose[0], pose[1])
        target = {
            "found": True,
            "x": 150.0,
            "y": 90.0,
            "class_id": 5,
        }

        reacquired = mission.step(target, pose, tof_distance_mm=300.0)

        self.assertEqual(reacquired.reason, "approach_reacquired")
        self.assertEqual(mission.state, MainTaskState.APPROACH)
        self.assertEqual(mission.locked_class_id, 5)
        self.assertEqual(vision.locked_ids, [5])


if __name__ == "__main__":
    unittest.main()

import math
import os
import sys
import unittest


PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from approach import ApproachController  # noqa: E402
from main_config import (  # noqa: E402
    ApproachConfig,
    NavigationConfig,
    OrbitConfig,
)
from navigation import (  # noqa: E402
    CoordinatePatrolController,
    HeadingTurnController,
    body_to_world,
    world_to_body,
)
from orbit import OrbitController  # noqa: E402


def target(x=160.0, y=170.0, found=True):
    return {"found": found, "x": x, "y": y, "class_id": 1}


class CoordinateTransformTests(unittest.TestCase):
    def test_world_body_round_trip(self):
        for heading in (0.0, math.pi / 2.0, -1.2, 2.7):
            body = world_to_body(30.0, -12.0, heading)
            world = body_to_world(body[0], body[1], heading)
            self.assertAlmostEqual(world[0], 30.0)
            self.assertAlmostEqual(world[1], -12.0)

    def test_world_positive_x_is_forward_at_zero_heading(self):
        self.assertEqual(world_to_body(20.0, 0.0, 0.0), (0.0, 20.0))

    def test_world_positive_x_is_right_at_ninety_degrees(self):
        body = world_to_body(20.0, 0.0, math.pi / 2.0)
        self.assertAlmostEqual(body[0], 20.0)
        self.assertAlmostEqual(body[1], 0.0, places=7)


class PatrolTests(unittest.TestCase):
    def test_patrol_generates_world_line_velocity(self):
        controller = CoordinatePatrolController(
            [{"x": 100.0, "y": 0.0, "heading_deg": 0.0}],
            NavigationConfig,
        )
        controller.reset(0.0, 0.0)
        result = controller.step((0.0, 0.0, 0.0))

        self.assertFalse(result.done)
        self.assertEqual(result.debug["profile"], "fast")
        self.assertAlmostEqual(result.command[0], 0.0)
        self.assertGreater(result.command[1], 0.0)
        self.assertEqual(result.command[2], 0.0)

    def test_cross_track_error_is_corrected_back_to_line(self):
        controller = CoordinatePatrolController(
            [{"x": 100.0, "y": 0.0, "heading_deg": 0.0}],
            NavigationConfig,
        )
        controller.reset(0.0, 0.0)
        result = controller.step((20.0, 10.0, 0.0))
        world = body_to_world(
            result.command[0], result.command[1], 0.0
        )

        self.assertLess(world[1], 0.0)
        self.assertGreater(world[0], 0.0)

    def test_waypoint_requires_explicit_advance(self):
        waypoints = (
            {"x": 10.0, "y": 0.0, "heading_deg": 0.0},
            {"x": 10.0, "y": 20.0, "heading_deg": 90.0},
        )
        controller = CoordinatePatrolController(
            waypoints, NavigationConfig
        )
        controller.reset(0.0, 0.0)
        result = controller.step((9.0, 0.0, 0.0))
        self.assertTrue(result.done)
        self.assertEqual(controller.index, 0)

        controller.advance()
        self.assertEqual(controller.index, 1)
        self.assertEqual(controller.segment_start, (10.0, 0.0))


class HeadingTurnTests(unittest.TestCase):
    def test_turn_uses_shortest_signed_error(self):
        controller = HeadingTurnController(NavigationConfig)
        controller.start(math.radians(-170.0))
        result = controller.step(math.radians(170.0), 0.0, 0.02)
        self.assertGreater(result.command[2], 0.0)

    def test_turn_completes_immediately_when_angle_and_rate_are_in_tolerance(self):
        controller = HeadingTurnController(NavigationConfig)
        controller.start(1.0)
        result = controller.step(1.0, 0.0, 0.02)
        self.assertTrue(result.done)
        self.assertEqual(result.command, (0.0, 0.0, 0.0))

    def test_turn_waits_while_yaw_rate_is_above_tolerance(self):
        controller = HeadingTurnController(NavigationConfig)
        controller.start(1.0)
        result = controller.step(
            1.0,
            NavigationConfig.TURN_YAW_RATE_TOLERANCE_RAD_S + 0.01,
            0.02,
        )
        self.assertFalse(result.done)

    def test_slow_turn_suppresses_wireless_angular_feedforward(self):
        controller = HeadingTurnController(NavigationConfig)
        controller.start(math.radians(4.5))

        result = controller.step(0.0, 0.0, 0.02)

        self.assertEqual(result.debug.get("profile"), "slow")
        self.assertNotEqual(result.command[2], 0.0)
        self.assertTrue(result.debug.get("suppress_feedforward_w"))

    def test_slow_turn_speed_is_proportional_to_heading_error(self):
        controller = HeadingTurnController(NavigationConfig)
        controller.start(math.radians(4.5))

        result = controller.step(0.0, 0.0, 0.02)

        self.assertAlmostEqual(
            result.command[2],
            NavigationConfig.TURN_SLOW_KP * math.radians(4.5),
        )

    def test_mid_turn_keeps_wireless_angular_feedforward(self):
        controller = HeadingTurnController(NavigationConfig)
        controller.start(math.radians(6.0))

        result = controller.step(0.0, 0.0, 0.02)

        self.assertEqual(result.debug.get("profile"), "mid")
        self.assertFalse(result.debug.get("suppress_feedforward_w"))

    def test_mid_turn_speed_is_proportional_below_its_limit(self):
        controller = HeadingTurnController(NavigationConfig)
        controller.start(math.radians(10.0))

        result = controller.step(0.0, 0.0, 0.02)

        self.assertEqual(result.debug.get("profile"), "mid")
        self.assertAlmostEqual(
            result.command[2],
            NavigationConfig.TURN_MID_KP * math.radians(10.0),
        )

    def test_mid_turn_is_limited_to_1_5_rad_s_at_25_degrees(self):
        controller = HeadingTurnController(NavigationConfig)
        controller.start(math.radians(25.0))

        result = controller.step(0.0, 0.0, 0.02)

        self.assertEqual(result.debug.get("profile"), "mid")
        self.assertAlmostEqual(
            result.command[2], NavigationConfig.TURN_MID_W_RAD_S
        )


class ApproachTests(unittest.TestCase):
    def test_target_right_uses_clockwise_yaw_without_lateral_translation(self):
        controller = ApproachController(ApproachConfig)
        result = controller.step(target(x=190.0, y=100.0), 400.0)

        self.assertEqual(result.command[0], 0.0)
        self.assertGreater(result.command[1], 0.0)
        self.assertLess(result.command[2], 0.0)

    def test_completion_requires_pixel_and_tof_conditions(self):
        controller = ApproachController(ApproachConfig)
        visual_only = controller.step(target(), 300.0)
        self.assertFalse(visual_only.done)

        reached = controller.step(
            target(),
            ApproachConfig.TARGET_STOP_DISTANCE_MM,
        )
        self.assertTrue(reached.done)
        self.assertEqual(reached.reason, "approach_reached")

    def test_target_loss_uses_original_decay_then_enters_search(self):
        controller = ApproachController(ApproachConfig)
        controller.step(target(x=190.0), 300.0)
        decaying = controller.step(
            target(found=False),
            300.0,
            dt=0.1,
        )
        self.assertFalse(decaying.failed)
        self.assertEqual(decaying.reason, "target_lost_decelerating")
        lost = decaying
        for _ in range(4):
            lost = controller.step(
                target(found=False),
                300.0,
                dt=0.1,
            )
        self.assertTrue(lost.failed)
        self.assertEqual(lost.reason, "spin_search")

    def test_missing_tof_keeps_original_visual_approach(self):
        controller = ApproachController(ApproachConfig)
        missing_tof = controller.step(target(), None)
        self.assertFalse(missing_tof.failed)
        self.assertGreater(missing_tof.command[1], 0.0)

    def test_combined_camera_protocol_field_names_are_accepted(self):
        controller = ApproachController(ApproachConfig)
        result = controller.step(
            {
                "target_found": True,
                "target_x": 160.0,
                "target_y": 170.0,
            },
            150.0,
        )
        self.assertTrue(result.done)


class OrbitTests(unittest.TestCase):
    def test_ccw_orbit_has_positive_vx_and_w(self):
        controller = OrbitController(OrbitConfig)
        controller.start_relative(0.0, math.pi / 2.0, 150.0)
        result = controller.step(target(), 150.0, 0.0)

        self.assertGreater(result.command[0], 0.0)
        self.assertAlmostEqual(result.command[1], 0.0)
        self.assertGreater(result.command[2], 0.0)

    def test_radius_error_commands_motion_toward_target(self):
        controller = OrbitController(OrbitConfig)
        controller.start_relative(0.0, math.pi / 2.0, 150.0)
        result = controller.step(target(), 250.0, 0.0)
        self.assertGreater(result.command[1], 0.0)

    def test_heading_wrap_keeps_shortest_heading_error(self):
        controller = OrbitController(OrbitConfig)
        controller.start_relative(
            math.radians(170.0), math.radians(30.0), 150.0
        )
        result = controller.step(
            target(), 150.0, math.radians(-175.0)
        )
        self.assertAlmostEqual(
            result.debug["heading_error_rad"],
            math.radians(15.0),
            places=6,
        )

    def test_orbit_runs_original_orbit_align_close_in_phases(self):
        controller = OrbitController(OrbitConfig)
        controller.start_relative(0.0, math.radians(10.0), 150.0)
        align = controller.step(
            target(), 150.0, math.radians(10.0), 0.0, 0.02
        )
        self.assertEqual(align.reason, "orbit_enter_align")
        close_in = controller.step(
            target(), 150.0, math.radians(10.0), 0.0, 0.02
        )
        self.assertEqual(close_in.reason, "orbit_enter_close_in")
        reached = controller.step(
            target(), 80.0, math.radians(10.0), 0.0, 0.02
        )
        self.assertTrue(reached.done)
        self.assertEqual(reached.reason, "orbit_reached")

    def test_align_translates_to_rod_pixel_without_turning_off_heading(self):
        class OffsetRodConfig(OrbitConfig):
            ORBIT_ROD_TARGET_X_PX = 205.0

        controller = OrbitController(OffsetRodConfig)
        controller.start_relative(0.0, math.radians(10.0), 150.0)
        controller.step(
            target(), 150.0, math.radians(10.0), 0.0, 0.02
        )

        aligning = controller.step(
            target(x=160.0),
            150.0,
            math.radians(10.0),
            0.0,
            0.02,
        )

        self.assertLess(aligning.command[0], 0.0)
        self.assertEqual(aligning.command[1], 0.0)
        self.assertAlmostEqual(aligning.command[2], 0.0)
        self.assertEqual(aligning.debug["rod_target_x_px"], 205.0)

    def test_align_holds_rod_pixel_while_correcting_heading(self):
        class OffsetRodConfig(OrbitConfig):
            ORBIT_ROD_TARGET_X_PX = 205.0

        controller = OrbitController(OffsetRodConfig)
        controller.start_relative(0.0, math.radians(10.0), 150.0)
        controller.step(
            target(), 150.0, math.radians(10.0), 0.0, 0.02
        )
        correcting = controller.step(
            target(x=205.0),
            150.0,
            math.radians(7.0),
            0.0,
            0.02,
        )

        self.assertEqual(correcting.command[0], 0.0)
        self.assertEqual(correcting.command[1], 0.0)
        self.assertGreater(correcting.command[2], 0.0)

    def test_close_in_maintains_rod_pixel_and_target_heading(self):
        class OffsetRodConfig(OrbitConfig):
            ORBIT_ROD_TARGET_X_PX = 205.0

        controller = OrbitController(OffsetRodConfig)
        controller.start_relative(0.0, math.radians(10.0), 150.0)
        controller.step(
            target(), 150.0, math.radians(10.0), 0.0, 0.02
        )
        controller.step(
            target(x=205.0),
            150.0,
            math.radians(10.0),
            0.0,
            0.02,
        )
        closing = controller.step(
            target(x=180.0),
            150.0,
            math.radians(10.0),
            0.0,
            0.02,
        )

        self.assertLess(closing.command[0], 0.0)
        self.assertEqual(closing.command[1], 0.0)
        self.assertAlmostEqual(closing.command[2], 0.0)

    def test_close_in_uses_y_pixel_not_tof_as_completion_signal(self):
        class OffsetRodConfig(OrbitConfig):
            ORBIT_ROD_TARGET_X_PX = 205.0
            ORBIT_ROD_TARGET_Y_PX = 170.0

        controller = OrbitController(OffsetRodConfig)
        controller.start_relative(0.0, math.radians(10.0), 150.0)
        controller.step(
            target(), 150.0, math.radians(10.0), 0.0, 0.02
        )
        controller.step(
            target(x=205.0, y=170.0),
            150.0,
            math.radians(10.0),
            0.0,
            0.02,
        )
        adjusting_y = controller.step(
            target(x=205.0, y=130.0),
            150.0,
            math.radians(10.0),
            0.0,
            0.02,
        )

        self.assertFalse(adjusting_y.done)
        self.assertEqual(adjusting_y.command[0], 0.0)
        self.assertGreater(adjusting_y.command[1], 0.0)
        self.assertAlmostEqual(adjusting_y.command[2], 0.0)

    def test_close_in_does_not_complete_on_tof_when_y_is_wrong(self):
        class OffsetRodConfig(OrbitConfig):
            ORBIT_ROD_TARGET_X_PX = 205.0
            ORBIT_ROD_TARGET_Y_PX = 170.0

        controller = OrbitController(OffsetRodConfig)
        controller.start_relative(0.0, math.radians(10.0), 150.0)
        controller.step(
            target(), 150.0, math.radians(10.0), 0.0, 0.02
        )
        controller.step(
            target(x=205.0, y=170.0),
            150.0,
            math.radians(10.0),
            0.0,
            0.02,
        )
        blocked = controller.step(
            target(x=205.0, y=130.0),
            80.0,
            math.radians(10.0),
            0.0,
            0.02,
        )

        self.assertFalse(blocked.done)
        self.assertEqual(blocked.command[1], 0.0)

    def test_start_can_override_configured_rod_pixel(self):
        controller = OrbitController(OrbitConfig)
        controller.start_relative(
            0.0,
            math.radians(10.0),
            150.0,
            rod_target_x_px=220.0,
        )
        controller.step(
            target(), 150.0, math.radians(10.0), 0.0, 0.02
        )
        aligning = controller.step(
            target(x=200.0),
            150.0,
            math.radians(10.0),
            0.0,
            0.02,
        )

        self.assertLess(aligning.command[0], 0.0)
        self.assertEqual(aligning.debug["rod_target_x_px"], 220.0)

    def test_lost_target_stops_orbit(self):
        controller = OrbitController(OrbitConfig)
        controller.start_relative(0.0, math.pi / 2.0)
        result = controller.step(target(found=False), 150.0, 0.0)
        self.assertTrue(result.failed)
        self.assertEqual(result.command, (0.0, 0.0, 0.0))


class MigratedParameterTests(unittest.TestCase):
    def test_approach_uses_half_scale_car141929_linear_parameters(self):
        self.assertEqual(ApproachConfig.APPROACH_SPEED_CM_S, 280.0)
        self.assertEqual(ApproachConfig.MIN_APPROACH_SPEED_CM_S, 30.0)
        self.assertEqual(ApproachConfig.PID_APPROACH_W_KP, 0.012)

    def test_orbit_contains_original_pid_set_and_scaled_linear_limits(self):
        self.assertEqual(OrbitConfig.ORBIT_MAX_VX_CM_S, 380.0)
        self.assertEqual(OrbitConfig.ORBIT_MAX_VY_CM_S, 60.0)
        self.assertEqual(OrbitConfig.ORBIT_MAX_W_RAD_S, 4.0)
        self.assertEqual(OrbitConfig.PID_CAMERA_TURN_KP, 0.0145)
        self.assertEqual(OrbitConfig.PID_ORBIT_TOF_KP, 0.25)
        self.assertEqual(OrbitConfig.PID_ORBIT_Y_KP, 0.6)
        self.assertEqual(OrbitConfig.PID_X_KP, 0.335)

    def test_patrol_linear_speeds_use_final_values(self):
        self.assertEqual(NavigationConfig.PATH_FAST_SPEED_CM_S, 325.0)
        self.assertEqual(NavigationConfig.PATH_MID_SPEED_CM_S, 210.0)
        self.assertEqual(NavigationConfig.PATH_SLOW_SPEED_CM_S, 90.0)
        self.assertEqual(
            NavigationConfig.PATH_CROSS_MAX_SPEED_CM_S,
            125.0,
        )
        self.assertEqual(NavigationConfig.PATH_MAX_SPEED_CM_S, 350.0)

    def test_navigation_angular_speeds_are_doubled(self):
        self.assertEqual(NavigationConfig.TRANSLATE_MAX_W_RAD_S, 2.70)
        self.assertEqual(NavigationConfig.TURN_FAST_W_RAD_S, 3.60)
        self.assertEqual(NavigationConfig.TURN_MID_KP, 5.0)
        self.assertEqual(NavigationConfig.TURN_MID_W_RAD_S, 1.50)
        self.assertEqual(NavigationConfig.TURN_SLOW_KP, 4.0)


if __name__ == "__main__":
    unittest.main()

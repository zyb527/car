import math
import os
import sys
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import main_orbit  # noqa: E402


class MainOrbitTests(unittest.TestCase):
    def test_orbit_feedforward_keeps_motor_limited_angular_speed(self):
        class Motor:
            def get_limited_physical_command(self):
                return (12.0, -34.0, 0.25)

        class Sender:
            def __init__(self):
                self.command = None
                self.straight_without_w = None

            def send_motor_command(self, motor, straight_without_w=False):
                self.straight_without_w = straight_without_w
                self.command = motor.get_limited_physical_command()
                return self.command

        sender = Sender()
        command = main_orbit._send_motor_feedforward(
            sender,
            Motor(),
            (30.0, 0.0, -1.6),
        )

        self.assertEqual(command, (12.0, -34.0, 0.25))
        self.assertEqual(sender.command, (12.0, -34.0, 0.25))
        self.assertFalse(sender.straight_without_w)

    def test_centered_target_uses_true_300_mm_geometric_radius(self):
        controller = main_orbit.FixedRadiusOrbitController()

        result = controller.step(
            {
                "found": True,
                "x": controller.config.TARGET_CENTER_X_PX,
                "y": 120.0,
            },
            dt=0.02,
        )

        expected_w = -abs(controller.config.ORBIT_ROTATION_SPEED_RAD_S)
        expected_vx = expected_w * (
            controller.config.ORBIT_RADIUS_MM / 10.0
        )
        self.assertEqual(result.reason, "fixed_radius_orbit")
        self.assertAlmostEqual(result.command[0], expected_vx)
        self.assertAlmostEqual(result.command[1], 0.0)
        self.assertAlmostEqual(result.command[2], expected_w)
        self.assertAlmostEqual(
            abs(result.command[0] / result.command[2]),
            30.0,
        )

    def test_first_visual_y_is_locked_as_no_tof_radius_reference(self):
        controller = main_orbit.FixedRadiusOrbitController()

        controller.step(
            {"found": True, "x": 160.0, "y": 123.0},
            dt=0.02,
        )
        result = controller.step(
            {"found": True, "x": 160.0, "y": 130.0},
            dt=0.02,
        )

        self.assertEqual(controller.orbit_target_y, 123.0)
        self.assertLess(result.command[1], 0.0)

    def test_missing_target_stops_and_clears_locked_reference(self):
        controller = main_orbit.FixedRadiusOrbitController()
        controller.step(
            {"found": True, "x": 160.0, "y": 123.0},
            dt=0.02,
        )

        result = controller.step(None, dt=0.02)

        self.assertEqual(result.command, (0.0, 0.0, 0.0))
        self.assertEqual(result.reason, "orbit_wait_target")
        self.assertIsNone(controller.orbit_target_y)

    def test_radius_config_is_finite_and_positive(self):
        radius = main_orbit.FixedRadiusOrbitConfig.ORBIT_RADIUS_MM
        self.assertTrue(math.isfinite(radius))
        self.assertGreater(radius, 0.0)


if __name__ == "__main__":
    unittest.main()

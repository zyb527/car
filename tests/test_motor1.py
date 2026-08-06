import os
import sys
import unittest


PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from motor import MotorConfig, MotorSystem, map_minimum_wheel_speed  # noqa: E402


class MinimumWheelSpeedMappingTests(unittest.TestCase):
    def test_peak_below_deadband_maps_all_wheels_to_zero(self):
        wheels, scale = map_minimum_wheel_speed(
            (1.9, -1.0, 0.5), 2.0, 4.0
        )
        self.assertEqual(wheels, (0.0, 0.0, 0.0))
        self.assertEqual(scale, 0.0)

    def test_peak_at_deadband_is_scaled_to_minimum(self):
        wheels, scale = map_minimum_wheel_speed((2.0, -1.0, 0.5), 2.0, 4.0)
        self.assertEqual(wheels, (4.0, -2.0, 1.0))
        self.assertEqual(scale, 2.0)

    def test_middle_band_preserves_three_wheel_ratio(self):
        wheels, scale = map_minimum_wheel_speed((-1.5, -1.5, 3.0), 2.0, 4.0)
        self.assertEqual(wheels, (-2.0, -2.0, 4.0))
        self.assertEqual(scale, 4.0 / 3.0)

    def test_peak_at_or_above_minimum_is_unchanged(self):
        self.assertEqual(
            map_minimum_wheel_speed((-2.0, -2.0, 4.0), 2.0, 4.0),
            ((-2.0, -2.0, 4.0), 1.0),
        )
        self.assertEqual(
            map_minimum_wheel_speed((-10.0, 15.0, 2.0)),
            ((-10.0, 15.0, 2.0), 1.0),
        )

    def test_default_config_uses_requested_thresholds(self):
        config = MotorConfig()
        self.assertEqual(config.wheel_speed_deadband_cm_s, 2.0)
        self.assertEqual(config.minimum_active_wheel_speed_cm_s, 4.0)

    def test_motor_system_maps_lateral_command_without_changing_ratio(self):
        motor = MotorSystem()
        motor.command(2.0, 0.0, 0.0)
        motor._control_step(0.01, (0.0, 0.0, 0.0))

        self.assertEqual(motor.get_state()["target_wheels"], (-2.0, -2.0, 4.0))
        self.assertEqual(motor.get_state()["limited_body"], (4.0, 0.0, 0.0))

    def test_motor_system_turns_sub_deadband_command_into_stop(self):
        motor = MotorSystem()
        motor.command(1.0, 0.0, 0.0)
        motor._control_step(0.01, (0.0, 0.0, 0.0))

        self.assertEqual(motor.get_state()["target_wheels"], (0.0, 0.0, 0.0))
        self.assertEqual(motor.get_state()["limited_body"], (0.0, 0.0, 0.0))
        self.assertEqual(motor.get_state()["duty"], (0, 0, 0))


if __name__ == "__main__":
    unittest.main()

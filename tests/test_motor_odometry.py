import math
import os
import sys
import unittest


PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from motor import (  # noqa: E402
    ChassisKinematics,
    MotorConfig,
    MotorSystem,
    SCurveLimiter,
    WheelConfig,
    WheelPIController,
)
from odometry import MahonyAHRS, OdometrySystem  # noqa: E402


class KinematicsTests(unittest.TestCase):
    def test_default_wheel_wiring_matches_legacy_encoder_order(self):
        config = MotorConfig()
        wiring = [
            (wheel.motor_index, wheel.encoder_a, wheel.encoder_b)
            for wheel in config.wheels
        ]
        self.assertEqual(
            wiring,
            [
                ("PWM_D4_DIR_D5", "D13", "D14"),
                ("PWM_C28_DIR_C29", "D15", "D16"),
                ("PWM_C30_DIR_C31", "C2", "C3"),
            ],
        )
        self.assertFalse(config.wheels[0].encoder_invert)
        self.assertFalse(config.wheels[1].encoder_invert)
        self.assertFalse(config.wheels[2].encoder_invert)

    def test_body_wheel_round_trip(self):
        cases = (
            (0.0, 0.0, 0.0),
            (30.0, 0.0, 0.0),
            (0.0, -42.0, 0.0),
            (12.5, 38.0, 1.2),
            (-55.0, 20.0, -2.0),
        )
        for expected in cases:
            wheels = ChassisKinematics.body_to_wheels(
                expected[0], expected[1], expected[2], 9.1, 1.3
            )
            actual = ChassisKinematics.wheels_to_body(
                wheels[0], wheels[1], wheels[2], 9.1, 1.3
            )
            for expected_value, actual_value in zip(expected, actual):
                self.assertAlmostEqual(expected_value, actual_value, places=7)

    def test_wheel_limit_preserves_direction(self):
        original = (100.0, -200.0, 50.0)
        limited, scale = ChassisKinematics.limit_wheels(original, 80.0)
        self.assertAlmostEqual(scale, 0.4)
        self.assertEqual(limited, (40.0, -80.0, 20.0))


class SCurveTests(unittest.TestCase):
    def test_xy_acceleration_and_jerk_limits(self):
        config = MotorConfig()
        limiter = SCurveLimiter(config)
        previous_ax = limiter.ax
        for dt in (0.010, 0.012, 0.008, 0.010) * 30:
            limiter.step(100.0, 0.0, 0.0, dt)
            self.assertLessEqual(
                abs(limiter.ax), config.xy_accel_up_cm_s2 + 1.0e-7
            )
            self.assertLessEqual(
                abs(limiter.ax - previous_ax),
                config.xy_jerk_cm_s3 * dt + 1.0e-7,
            )
            previous_ax = limiter.ax

    def test_reverse_command_brakes_through_zero(self):
        limiter = SCurveLimiter(MotorConfig())
        for _ in range(100):
            limiter.step(60.0, 0.0, 0.0, 0.01)
        self.assertGreater(limiter.vx, 0.0)

        samples = []
        for _ in range(200):
            samples.append(limiter.step(-60.0, 0.0, 0.0, 0.01)[0])
        self.assertTrue(any(value <= 0.0 for value in samples))
        self.assertGreaterEqual(max(samples), 0.0)
        self.assertGreaterEqual(samples[-1], -60.0 - 1.0e-7)

    def test_constant_target_settles_without_chatter(self):
        limiter = SCurveLimiter(MotorConfig())
        for _ in range(250):
            limiter.step(100.0, 0.0, 2.0, 0.01)
        self.assertAlmostEqual(limiter.vx, 100.0, places=7)
        self.assertAlmostEqual(limiter.ax, 0.0, places=7)
        self.assertAlmostEqual(limiter.w, 2.0, places=7)
        self.assertAlmostEqual(limiter.aw, 0.0, places=7)

    def test_motor_move_is_wheel_feasible(self):
        config = MotorConfig()
        config.max_wheel_speed_cm_s = 30.0
        system = MotorSystem(config)
        system.move(120.0, 120.0, 3.0)
        target = system.get_state()["target_body"]
        wheels = ChassisKinematics.body_to_wheels(
            target[0],
            target[1],
            target[2],
            config.robot_radius_cm,
            config.rotation_gain,
        )
        self.assertLessEqual(max(abs(value) for value in wheels), 30.0 + 1.0e-7)

    def test_command_bypasses_s_curve_but_keeps_wheel_limit(self):
        config = MotorConfig()
        config.max_wheel_speed_cm_s = 30.0
        system = MotorSystem(config)
        system.command(120.0, 120.0, 3.0)
        system._control_step(0.01, (0.0, 0.0, 0.0))

        state = system.get_state()
        self.assertFalse(state["s_curve_enabled"])
        self.assertEqual(state["limited_body"], state["target_body"])
        self.assertLessEqual(
            max(abs(value) for value in state["target_wheels"]),
            30.0 + 1.0e-7,
        )

    def test_public_linear_speed_is_scaled_before_wheel_control(self):
        config = MotorConfig()
        system = MotorSystem(config)
        system.command(50.0, 0.0, 0.0)
        target = system.get_state()["target_body"]
        self.assertAlmostEqual(
            target[0], 50.0 * config.body_command_lateral_speed_scale
        )
        self.assertEqual(target[1:], (0.0, 0.0))

        system.command(0.0, 50.0, 0.0)
        target = system.get_state()["target_body"]
        self.assertAlmostEqual(
            target[1], 50.0 * config.body_command_forward_speed_scale
        )

    def test_limited_physical_command_reverses_axis_scales(self):
        config = MotorConfig()
        system = MotorSystem(config)
        system.command(40.0, 50.0, 0.2)
        system._control_step(0.01, (0.0, 0.0, 0.0))
        command = system.get_limited_physical_command()
        self.assertAlmostEqual(command[0], 40.0)
        self.assertAlmostEqual(command[1], 50.0)
        self.assertAlmostEqual(command[2], 0.2)

    def test_command_holds_heading_only_without_rotation_request(self):
        class FakeOdometry:
            def __init__(self):
                self.heading_rad = 0.0
                self.yaw_rate_rad_s = 0.0

            def get_state(self):
                return {
                    "calibrated": True,
                    "heading_rad": self.heading_rad,
                    "yaw_rate_rad_s": self.yaw_rate_rad_s,
                }

        odometry = FakeOdometry()
        system = MotorSystem(MotorConfig(), odometry=odometry)
        system.command(30.0, 0.0, 0.0)
        self.assertTrue(system.get_state()["heading_hold_active"])

        odometry.heading_rad = 0.1
        system.command(30.0, 0.0, 0.0)
        self.assertLess(system.get_state()["target_body"][2], 0.0)

        system.command(30.0, 0.0, 0.2)
        self.assertFalse(system.get_state()["heading_hold_active"])
        self.assertAlmostEqual(system.get_state()["target_body"][2], 0.2)

        # Ensure None heading_rad is safely handled without raising TypeError
        odometry.heading_rad = None
        system.command(30.0, 0.0, 0.0)
        self.assertFalse(system.get_state()["heading_hold_active"])

    def test_switching_from_command_to_move_starts_at_current_velocity(self):
        config = MotorConfig()
        # This test isolates the S-curve handoff from physical-speed
        # calibration, which is covered separately.
        config.body_command_speed_scale = 1.0
        config.body_command_lateral_speed_scale = 1.0
        config.body_command_forward_speed_scale = 1.0
        system = MotorSystem(config)
        system.command(60.0, 0.0, 0.0)
        system._control_step(0.01, (0.0, 0.0, 0.0))
        self.assertAlmostEqual(system.get_limited_command()[0], 60.0)

        system.move(-60.0, 0.0, 0.0)
        system._control_step(0.01, (0.0, 0.0, 0.0))
        vx = system.get_limited_command()[0]
        self.assertGreater(vx, 0.0)
        self.assertLess(vx, 60.0)

    def test_soft_stop_after_command_uses_s_curve(self):
        config = MotorConfig()
        system = MotorSystem(config)
        system.command(60.0, 0.0, 0.0)
        system._control_step(0.01, (0.0, 0.0, 0.0))
        system.soft_stop()
        system._control_step(0.01, (0.0, 0.0, 0.0))

        state = system.get_state()
        self.assertTrue(state["s_curve_enabled"])
        self.assertTrue(state["soft_stopping"])
        self.assertGreater(state["limited_body"][0], 0.0)
        self.assertLess(
            state["limited_body"][0],
            60.0 * config.body_command_lateral_speed_scale,
        )

    def test_calibration_duty_is_limited_and_hard_stop_clears_it(self):
        config = MotorConfig()
        config.max_duty = 900.0
        system = MotorSystem(config)
        system.calibration_duty(1200, -1200, 300)
        state = system.get_state()
        self.assertTrue(state["open_loop_calibration"])
        self.assertTrue(state["motion_active"])

        system.hard_stop()
        state = system.get_state()
        self.assertFalse(state["open_loop_calibration"])
        self.assertEqual(state["duty"], (0, 0, 0))


class WheelControllerTests(unittest.TestCase):
    def test_acceleration_feedforward_uses_filtered_target_acceleration(self):
        wheel = WheelConfig(
            "unused",
            "unused",
            "unused",
            kp=0.0,
            ki=0.0,
            feedforward=0.0,
            ka=2.0,
            stiction_duty=0.0,
            acceleration_lpf_time_constant_s=0.0,
            max_target_acceleration_cm_s2=100.0,
        )
        controller = WheelPIController(wheel, 9000.0)

        # 10 ms 内从 0 跳到 20 cm/s，原始加速度为 2000 cm/s²，
        # 限幅后为 100 cm/s²，因此加速度前馈输出应为 200。
        self.assertEqual(controller.update(20.0, 0.0, 0.01), 200)
        self.assertEqual(controller.target_acceleration, 100.0)
        self.assertEqual(controller.acceleration_feedforward, 200.0)

        # 目标速度不变时，加速度前馈归零；实测速度变化不会进入加速度计算。
        self.assertEqual(controller.update(20.0, 10.0, 0.01), 0)
        self.assertEqual(controller.target_acceleration, 0.0)

    def test_final_output_anti_windup(self):
        wheel = WheelConfig(
            "unused",
            "unused",
            "unused",
            kp=0.0,
            ki=100.0,
            feedforward=0.0,
            stiction_duty=0.0,
        )
        controller = WheelPIController(wheel, 100.0)
        for _ in range(100):
            output = controller.update(100.0, 0.0, 0.01)
        self.assertEqual(output, 100)
        self.assertLessEqual(controller.integral, 100.0)

        output = controller.update(0.0, 100.0, 0.01)
        self.assertLess(output, 100)

    def test_stopped_wheel_resets_controller(self):
        wheel = WheelConfig("unused", "unused", "unused")
        controller = WheelPIController(wheel, 9000.0)
        controller.integral = 500.0
        self.assertEqual(controller.update(0.0, 0.2, 0.01), 0)
        self.assertEqual(controller.integral, 0.0)

    def test_stiction_is_removed_when_wheel_is_already_overspeeding(self):
        wheel = WheelConfig(
            "unused",
            "unused",
            "unused",
            kp=0.0,
            ki=0.0,
            feedforward=0.0,
            stiction_duty=1000.0,
            stiction_full_speed=5.0,
        )
        controller = WheelPIController(wheel, 9000.0)
        self.assertEqual(controller.update(20.0, 0.0, 0.01), 1000)
        # 补偿应开始撤除，但不能在一个编码器采样周期内突变为零。
        self.assertLess(controller.update(20.0, 30.0, 0.01), 1000)
        for _ in range(30):
            output = controller.update(20.0, 30.0, 0.01)
        self.assertLess(abs(output), 50)

    def test_breakaway_changes_once_to_running_offset(self):
        wheel = WheelConfig(
            "unused",
            "unused",
            "unused",
            kp=0.0,
            ki=0.0,
            feedforward=0.0,
            stiction_duty=1000.0,
            running_offset_duty=500.0,
            stiction_full_speed=5.0,
        )
        controller = WheelPIController(wheel, 9000.0)
        self.assertEqual(controller.update(20.0, 0.0, 0.01), 1000)
        for _ in range(30):
            output = controller.update(20.0, 5.0, 0.01)
        self.assertAlmostEqual(output, 500, delta=10)

        for _ in range(10):
            output = controller.update(20.0, 3.0, 0.01)
        self.assertAlmostEqual(output, 500, delta=10)

        self.assertEqual(controller.update(20.0, 0.5, 0.01), 1000)

    def test_encoder_samples_are_accumulated_for_calibration(self):
        class FakeEncoder:
            def __init__(self, value):
                self.value = value

            def get(self):
                return self.value

        system = MotorSystem(MotorConfig())
        system._encoders = (FakeEncoder(2), FakeEncoder(-3), FakeEncoder(4))
        system._update_measured_speeds(0.01)
        system._update_measured_speeds(0.01)
        latest, totals = system.get_encoder_counts()
        self.assertEqual(latest, (2, -3, 4))
        self.assertEqual(totals, (4, -6, 8))
        system.reset_encoder_totals()
        self.assertEqual(system.get_encoder_counts()[1], (0, 0, 0))


class OdometryTests(unittest.TestCase):
    def test_forward_motion_integrates_along_world_x(self):
        odometry = OdometrySystem()
        odometry.reset_heading(0.0)
        wheels = ChassisKinematics.body_to_wheels(0.0, 100.0, 0.0)
        for _ in range(100):
            odometry.update_wheels(wheels[0], wheels[1], wheels[2], 0.01)
        x_cm, y_cm, heading = odometry.get_pose()
        self.assertAlmostEqual(
            x_cm, 100.0 * odometry.config.forward_distance_scale, places=6
        )
        self.assertAlmostEqual(y_cm, 0.0, places=6)
        self.assertAlmostEqual(heading, 0.0, places=6)

    def test_right_motion_is_negative_world_y_at_zero_heading(self):
        odometry = OdometrySystem()
        odometry.reset_heading(0.0)
        wheels = ChassisKinematics.body_to_wheels(50.0, 0.0, 0.0)
        for _ in range(100):
            odometry.update_wheels(wheels[0], wheels[1], wheels[2], 0.01)
        x_cm, y_cm, _ = odometry.get_pose()
        self.assertAlmostEqual(x_cm, 0.0, places=6)
        self.assertAlmostEqual(
            y_cm, -50.0 * odometry.config.lateral_distance_scale, places=6
        )

    def test_stationary_mahony_remains_level(self):
        ahrs = MahonyAHRS()
        self.assertTrue(ahrs.initialize_from_accel(0.0, 0.0, 1.0))
        for _ in range(1000):
            self.assertTrue(
                ahrs.update(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.002)
            )
        roll, pitch, _ = ahrs.attitude()
        self.assertAlmostEqual(roll, 0.0, places=6)
        self.assertAlmostEqual(pitch, 0.0, places=6)
        quaternion_norm = math.sqrt(
            sum(value * value for value in ahrs.quaternion())
        )
        self.assertAlmostEqual(quaternion_norm, 1.0, places=7)

    def test_reset_heading_resets_unwrapped_heading(self):
        odometry = OdometrySystem()
        odometry.heading_unwrapped_rad = 12.0
        odometry.reset_heading(0.3)
        self.assertAlmostEqual(odometry.heading_rad, 0.3)
        self.assertAlmostEqual(odometry.heading_unwrapped_rad, 0.3)


if __name__ == "__main__":
    unittest.main()

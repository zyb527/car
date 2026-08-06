import importlib.util
import math
import pathlib
import unittest


ASSISTANT_DIR = pathlib.Path(__file__).resolve().parents[1]
FOLLOW_PATH = ASSISTANT_DIR / "follow.py"
CONFIG_PATH = ASSISTANT_DIR / "config.py"


def load_follow_module():
    spec = importlib.util.spec_from_file_location(
        "assistant_follow_controller",
        FOLLOW_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_production_config():
    spec = importlib.util.spec_from_file_location(
        "assistant_follow_production_config",
        CONFIG_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ControllerConfig:
    IPM_CALIBRATED = True
    IPM_IMAGE_TO_PLANE = (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    )
    IPM_DENOMINATOR_EPSILON = 1.0e-6

    REF_CX = 160.0
    REF_CY = 120.0
    REF_DISTANCE_PX = 80.0
    REF_LINE_ANGLE_DEG = 0.0
    PARALLEL_ANGLE_CALIBRATION = (
        (160.0, 120.0, 0.0),
        (100.0, 80.0, 0.0),
        (220.0, 80.0, 0.0),
        (100.0, 180.0, 0.0),
        (220.0, 180.0, 0.0),
    )

    FRONT_DEADBAND_PLANE = 0.0
    FRONT_RESPONSE_POINTS_PLANE = (
        (0.0, 0.0),
        (10.0, 5.0),
        (60.0, 30.0),
    )

    LATERAL_DEADBAND_PLANE = 0.0
    LATERAL_RESPONSE_POINTS_PLANE = (
        (0.0, 0.0),
        (5.0, 10.0),
        (30.0, 30.0),
    )

    ANGLE_DEADBAND_DEG = 0.0
    ANGLE_RESPONSE_POINTS = (
        (0.0, 0.0),
        (10.0, 1.0),
        (30.0, 2.0),
    )

    VX_SIGN = 1.0
    VY_SIGN = 1.0
    W_SIGN = 1.0

    MAX_VX = 30.0
    MAX_VY = 30.0
    MAX_W = 2.0
    MAX_COMMAND_VX = 200.0
    MAX_COMMAND_VY = 200.0
    MAX_COMMAND_W = 3.4
    FORMATION_RIGHT_OFFSET_CM = -20.0
    FORMATION_FORWARD_OFFSET_CM = 0.0
    VISUAL_RIGID_ENABLED = True
    POSITION_FILTER_ALPHA = 1.0
    POSITION_FILTER_BETA = 1.0


class PlaneControllerConfig(ControllerConfig):
    CAMERA_OUTPUTS_PLANE_POSE = True
    REF_TARGET_X_CM = 1.0
    REF_TARGET_Y_CM = -6.0
    REF_TARGET_THETA_DEG = 0.0


def measurement(
    mid_x=160.0,
    mid_y=120.0,
    distance_px=80.0,
    angle=0.0,
):
    return {
        "found": True,
        "mid_x": mid_x,
        "mid_y": mid_y,
        "line_angle_deg": angle,
        "distance_px": distance_px,
        "quality": 60,
    }


def plane_measurement(target_x=1.0, target_y=-6.0, theta=0.0):
    return {
        "found": True,
        "target_x_cm": target_x,
        "target_y_cm": target_y,
        "theta_deg": theta,
        "coordinate_space": "plane",
    }


class FollowControllerTests(unittest.TestCase):
    def setUp(self):
        self.follow = load_follow_module()

    def test_identity_ipm_reconstructs_image_pose(self):
        pose = self.follow.measurement_to_plane_pose(
            measurement(
                mid_x=100.0,
                mid_y=80.0,
                distance_px=20.0,
                angle=0.0,
            ),
            ControllerConfig.IPM_IMAGE_TO_PLANE,
        )

        self.assertAlmostEqual(pose["right"], 100.0)
        self.assertAlmostEqual(pose["forward"], 80.0)
        self.assertAlmostEqual(pose["line_angle_deg"], 0.0)
        self.assertAlmostEqual(pose["light_separation"], 20.0)
        self.assertEqual(pose["first"], (90.0, 80.0))
        self.assertEqual(pose["second"], (110.0, 80.0))

    def test_projective_division_is_applied(self):
        matrix = (
            (2.0, 0.0, 10.0),
            (0.0, 3.0, -5.0),
            (0.01, 0.0, 1.0),
        )

        right, forward = self.follow.project_image_point_to_plane(
            100.0,
            20.0,
            matrix,
        )

        self.assertAlmostEqual(right, 105.0)
        self.assertAlmostEqual(forward, 27.5)

    def test_parallel_angle_calibration_returns_exact_sample(self):
        result = self.follow.interpolate_parallel_image_angle_deg(
            243.0,
            150.0,
            (
                (138.5, 155.0, 3.14),
                (243.0, 150.0, -10.89),
            ),
        )

        self.assertAlmostEqual(result, -10.89)

    def test_parallel_angle_calibration_handles_undirected_wrap(self):
        result = self.follow.interpolate_parallel_image_angle_deg(
            0.0,
            0.0,
            (
                (-1.0, 0.0, 89.0),
                (1.0, 0.0, -89.0),
            ),
        )

        self.assertAlmostEqual(abs(result), 90.0)

    def test_production_config_has_no_legacy_ipm_or_fisheye_model(self):
        config = load_production_config()

        self.assertTrue(config.CAMERA_OUTPUTS_PLANE_POSE)
        self.assertFalse(hasattr(config, "IPM_IMAGE_TO_PLANE"))
        self.assertFalse(
            hasattr(config, "PARALLEL_ANGLE_MODEL_COEFFICIENTS")
        )

    def test_production_controller_consumes_camera_plane_pose(self):
        controller = self.follow.FollowController(load_production_config())

        self.assertTrue(controller.camera_outputs_plane_pose)
        self.assertIsNone(controller.image_to_plane)
        self.assertIsNone(controller.parallel_angle_model)
        self.assertIsNone(controller.parallel_angle_calibration)

    def test_plane_pose_controls_position_and_relative_angle_directly(self):
        controller = self.follow.FollowController(PlaneControllerConfig)

        command = controller.update(
            plane_measurement(target_x=6.0, target_y=4.0, theta=10.0),
            now_ms=20,
        )

        self.assertEqual(command[:2], (10.0, 5.0))
        self.assertAlmostEqual(command[2], 1.0)

    def test_zero_camera_theta_disables_only_visual_turn_correction(self):
        controller = self.follow.FollowController(PlaneControllerConfig)

        command = controller.update(
            plane_measurement(target_x=6.0, target_y=4.0, theta=0.0),
            now_ms=20,
        )

        self.assertEqual(command[:2], (10.0, 5.0))
        self.assertEqual(command[2], 0.0)

    def test_projection_rejects_near_zero_denominator(self):
        matrix = (
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (1.0, 0.0, -10.0),
        )

        with self.assertRaises(ValueError):
            self.follow.project_image_point_to_plane(
                10.0,
                20.0,
                matrix,
            )

    def test_controller_does_not_project_endpoints_for_runtime_control(self):
        class EndpointSingularityConfig(ControllerConfig):
            # 中点 x=120 的分母为 2，左端点 x=100 的分母为 0。
            IPM_IMAGE_TO_PLANE = (
                (1.0, 0.0, 0.0),
                (0.0, 1.0, 0.0),
                (0.1, 0.0, -10.0),
            )

        controller = self.follow.FollowController(EndpointSingularityConfig)
        command = controller.update(
            measurement(mid_x=120.0, distance_px=40.0),
            now_ms=20,
        )

        self.assertTrue(all(isinstance(value, float) for value in command))

    def test_controller_does_not_revalidate_ipm_matrix_each_update(self):
        controller = self.follow.FollowController(ControllerConfig)
        original_validator = self.follow._validated_homography

        def fail_if_called(_matrix):
            raise AssertionError("runtime must use the validated matrix")

        self.follow._validated_homography = fail_if_called
        try:
            command = controller.update(measurement(), now_ms=20)
        finally:
            self.follow._validated_homography = original_validator

        self.assertEqual(command, (0.0, 0.0, 0.0))

    def test_controller_uses_rectified_lateral_front_and_angle(self):
        controller = self.follow.FollowController(ControllerConfig)

        command = controller.update(
            measurement(mid_x=165.0, mid_y=130.0, angle=10.0),
            now_ms=20,
        )

        self.assertEqual(command[:2], (10.0, 5.0))
        self.assertAlmostEqual(command[2], 1.0)

    def test_reference_measurement_maps_to_zero_errors(self):
        controller = self.follow.FollowController(ControllerConfig)

        command = controller.update(measurement(), now_ms=20)

        self.assertEqual(command, (0.0, 0.0, 0.0))
        self.assertEqual(
            controller.get_state()["errors"],
            {"front": 0.0, "lateral": 0.0, "angle": 0.0},
        )

    def test_reference_and_measurement_share_same_ipm_bias(self):
        class BiasedConfig(ControllerConfig):
            IPM_IMAGE_TO_PLANE = (
                (1.0, 0.0, 30.0),
                (0.0, 1.0, -12.0),
                (0.0, 0.0, 1.0),
            )

        controller = self.follow.FollowController(BiasedConfig)

        self.assertEqual(
            controller.update(measurement(), now_ms=20),
            (0.0, 0.0, 0.0),
        )

    def test_line_angle_error_uses_180_degree_period(self):
        class WrappedConfig(ControllerConfig):
            REF_LINE_ANGLE_DEG = 89.0
            PARALLEL_ANGLE_CALIBRATION = (
                (160.0, 120.0, 89.0),
            )
            ANGLE_RESPONSE_POINTS = (
                (0.0, 0.0),
                (10.0, 5.0),
            )

        controller = self.follow.FollowController(WrappedConfig)
        command = controller.update(measurement(angle=-89.0), now_ms=20)

        self.assertAlmostEqual(command[2], 1.0)

    def test_relative_heading_uses_raw_angle_and_physical_w_sign(self):
        class RelativeHeadingConfig(ControllerConfig):
            ANGLE_DEADBAND_DEG = 3.0
            W_SIGN = -1.0

        controller = self.follow.FollowController(RelativeHeadingConfig)
        controller.update(measurement(angle=2.0), now_ms=20)
        state = controller.get_state()

        # 角速度修正在 3° 死区内为零，但车体系旋转仍使用完整的 2°。
        self.assertEqual(state["command"][2], 0.0)
        self.assertAlmostEqual(
            state["relative_heading_rad"],
            math.radians(-2.0),
        )

    def test_alpha_beta_rate_estimate_adds_limited_lateral_d_trim(self):
        class DerivativeConfig(ControllerConfig):
            POSITION_FILTER_ALPHA = 1.0
            POSITION_FILTER_BETA = 0.2
            LATERAL_RESPONSE_POINTS_PLANE = (
                (0.0, 0.0),
                (100.0, 0.0),
            )
            LATERAL_D_GAIN = 1.0
            MAX_LATERAL_D_TRIM = 100.0
            MAX_VX = 100.0

        controller = self.follow.FollowController(DerivativeConfig)
        controller.update(measurement(), now_ms=0)
        command = controller.update(
            measurement(mid_x=170.0),
            now_ms=100,
        )

        self.assertAlmostEqual(command[0], 20.0)
        self.assertAlmostEqual(
            controller.get_state()["error_rates"]["lateral"],
            20.0,
        )

    def test_visual_rigid_estimate_keeps_learned_translation_at_zero_relative_rate(self):
        controller = self.follow.FollowController(ControllerConfig)
        controller.set_follower_twist(0.0, 0.0, 0.0)
        controller.update(measurement(), now_ms=0)

        controller.update(measurement(mid_y=122.0), now_ms=100)
        rigid = controller.get_visual_rigid_command()
        self.assertAlmostEqual(rigid[0], 0.0)
        self.assertAlmostEqual(rigid[1], 20.0)
        self.assertAlmostEqual(rigid[2], 0.0)

        controller.set_follower_twist(0.0, 20.0, 0.0)
        controller.update(measurement(mid_y=123.0), now_ms=200)
        rigid = controller.get_visual_rigid_command()
        self.assertAlmostEqual(rigid[0], 0.0)
        self.assertAlmostEqual(rigid[1], 20.0)
        self.assertAlmostEqual(rigid[2], 0.0)

        controller.set_follower_twist(0.0, 20.0, 0.0)
        controller.update(measurement(mid_y=123.0), now_ms=300)
        rigid = controller.get_visual_rigid_command()
        self.assertAlmostEqual(rigid[0], 0.0)
        self.assertAlmostEqual(rigid[1], 20.0)
        self.assertAlmostEqual(rigid[2], 0.0)

    def test_visual_rigid_rejects_follower_self_motion_during_correction(self):
        controller = self.follow.FollowController(ControllerConfig)
        controller.set_follower_twist(0.0, 0.0, 0.0)
        controller.update(measurement(), now_ms=0)

        # 主车静止；辅助车在 100 ms 内从 0 加速到 10 cm/s，因此自身约
        # 前进 0.5 cm，视觉中的主车相对位置相应后退 0.5 cm。
        controller.set_follower_twist(0.0, 10.0, 0.0)
        controller.update(measurement(mid_y=119.5), now_ms=100)

        rigid = controller.get_visual_rigid_command()
        self.assertAlmostEqual(rigid[0], 0.0, places=6)
        self.assertAlmostEqual(rigid[1], 0.0, places=6)
        self.assertAlmostEqual(rigid[2], 0.0, places=6)

    def test_visual_rigid_prediction_uses_full_formation_radius(self):
        controller = self.follow.FollowController(ControllerConfig)
        # 主车静止、辅助车位于理想槽位并以 1 rad/s 逆时针旋转。
        # 100 ms 后，主车在辅助车坐标系中的一阶前向位移应为 -2 cm，
        # 相对航向应为 -0.1 rad；这些变化都来自辅助车自身运动，
        # 不应该被误判成主车速度。
        controller.set_follower_twist(0.0, 0.0, 1.0)
        controller.update(measurement(), now_ms=0)
        controller.set_follower_twist(0.0, 0.0, 1.0)
        controller.update(
            measurement(
                mid_y=118.0,
                angle=math.degrees(-0.1),
            ),
            now_ms=100,
        )

        rigid = controller.get_visual_rigid_command()
        self.assertAlmostEqual(rigid[0], 0.0, places=6)
        self.assertAlmostEqual(rigid[1], 0.0, places=6)
        self.assertAlmostEqual(rigid[2], 0.0, places=6)

    def test_visual_rigid_observer_uses_visual_residual_to_update_main_twist(self):
        class SmoothedRigidConfig(ControllerConfig):
            POSITION_FILTER_BETA = 0.15

        controller = self.follow.FollowController(SmoothedRigidConfig)
        controller.set_follower_twist(0.0, 0.0, 0.0)
        controller.update(measurement(), now_ms=0)
        controller.update(measurement(mid_y=122.0), now_ms=100)
        # 前向视觉残差为 2 cm / 100 ms；beta=0.15 时主车速度状态只
        # 校正为 3 cm/s，而不是直接把相邻帧差分的 20 cm/s 当作命令。
        rigid = controller.get_visual_rigid_command()
        self.assertAlmostEqual(rigid[0], 0.0, places=6)
        self.assertAlmostEqual(rigid[1], 3.0, places=6)
        self.assertAlmostEqual(rigid[2], 0.0, places=6)

    def test_front_channel_uses_asymmetric_opening_and_closing_gains(self):
        class AsymmetricConfig(ControllerConfig):
            POSITION_FILTER_ALPHA = 1.0
            POSITION_FILTER_BETA = 1.0
            FRONT_RESPONSE_POINTS_PLANE = (
                (0.0, 0.0),
                (100.0, 50.0),
            )
            FRONT_OPENING_D_GAIN = 0.25
            FRONT_CLOSING_D_GAIN = 0.40
            FRONT_CLOSING_P_SCALE = 0.70
            FRONT_CLOSING_P_BAND_PLANE = 5.0
            MAX_FRONT_D_TRIM = 100.0
            MAX_VY = 100.0

        controller = self.follow.FollowController(AsymmetricConfig)
        controller.update(measurement(), now_ms=0)

        opening = controller.update(
            measurement(mid_y=130.0),
            now_ms=100,
        )
        self.assertAlmostEqual(opening[1], 30.0)

        closing = controller.update(
            measurement(mid_y=125.0),
            now_ms=200,
        )
        # 误差正好处于 5 cm 制动区边界，P 仍保持 100%。
        self.assertAlmostEqual(closing[1], -17.5)

        near_closing = controller.update(
            measurement(mid_y=122.5),
            now_ms=300,
        )
        # 进入制动区一半后，P 比例从 1.0 渐变到 0.85。
        self.assertAlmostEqual(near_closing[1], -8.9375)

    def test_visual_rigid_rotation_uses_existing_formation_radius(self):
        controller = self.follow.FollowController(ControllerConfig)
        controller.set_follower_twist(0.0, 0.0, 0.0)
        controller.update(measurement(angle=0.0), now_ms=0)
        controller.update(
            measurement(angle=math.degrees(0.1)),
            now_ms=100,
        )

        rigid = controller.get_visual_rigid_command()
        self.assertAlmostEqual(rigid[0], 0.0, places=6)
        self.assertAlmostEqual(rigid[1], -20.0, places=6)
        self.assertAlmostEqual(rigid[2], 1.0, places=6)

    def test_visual_rigid_small_rotation_skips_formation_offset_velocity(self):
        class DeadbandConfig(ControllerConfig):
            RIGID_SLOT_W_COMP_DEADBAND_RAD_S = 0.3

        controller = self.follow.FollowController(DeadbandConfig)
        controller.set_follower_twist(0.0, 0.0, 0.0)
        controller.update(measurement(angle=0.0), now_ms=0)
        controller.update(
            measurement(angle=math.degrees(0.02)),
            now_ms=100,
        )

        rigid = controller.get_visual_rigid_command()
        self.assertAlmostEqual(rigid[0], 0.0, places=6)
        self.assertAlmostEqual(rigid[1], 0.0, places=6)
        self.assertAlmostEqual(rigid[2], 0.2, places=6)

    def test_continuous_deadband_removes_only_center_region(self):
        class DeadbandConfig(ControllerConfig):
            FRONT_DEADBAND_PLANE = 3.0
            FRONT_RESPONSE_POINTS_PLANE = (
                (0.0, 0.0),
                (30.0, 30.0),
            )

        controller = self.follow.FollowController(DeadbandConfig)

        self.assertEqual(
            controller.update(measurement(mid_y=122.0), now_ms=20)[1],
            0.0,
        )
        self.assertEqual(
            controller.update(measurement(mid_y=125.0), now_ms=40)[1],
            2.0,
        )

    def test_direction_signs_are_independent(self):
        class SignConfig(ControllerConfig):
            VX_SIGN = -1.0
            VY_SIGN = -1.0
            W_SIGN = -1.0

        controller = self.follow.FollowController(SignConfig)
        command = controller.update(
            measurement(mid_x=165.0, mid_y=130.0, angle=10.0),
            now_ms=20,
        )

        self.assertEqual(command[:2], (-10.0, -5.0))
        self.assertAlmostEqual(command[2], -1.0)

    def test_uncalibrated_ipm_is_rejected(self):
        class UncalibratedConfig(ControllerConfig):
            IPM_CALIBRATED = False

        with self.assertRaisesRegex(ValueError, "IPM is not calibrated"):
            self.follow.FollowController(UncalibratedConfig)

    def test_production_controller_does_not_require_car_side_ipm(self):
        config = load_production_config()

        controller = self.follow.FollowController(config)
        self.assertIsNone(controller.image_to_plane)

    def test_production_front_center_response_clears_low_speed_deadband(self):
        config = load_production_config()

        self.assertEqual(
            self.follow.piecewise_linear_response(
                config.FRONT_RESPONSE_POINTS_PLANE[1][0],
                config.FRONT_RESPONSE_POINTS_PLANE,
                config.MAX_VY,
            ),
            config.FRONT_RESPONSE_POINTS_PLANE[1][1],
        )

    def test_final_limit_survives_misconfigured_sign_magnitude(self):
        class BadSignConfig(ControllerConfig):
            VX_SIGN = 10.0
            VY_SIGN = 10.0
            W_SIGN = 10.0

        controller = self.follow.FollowController(BadSignConfig)
        command = controller.update(
            measurement(mid_x=260.0, mid_y=220.0, angle=40.0),
            now_ms=20,
        )

        self.assertLessEqual(abs(command[0]), BadSignConfig.MAX_VX)
        self.assertLessEqual(abs(command[1]), BadSignConfig.MAX_VY)
        self.assertLessEqual(abs(command[2]), BadSignConfig.MAX_W)

    def test_outputs_are_limited_and_reset_clears_runtime_state(self):
        class LimitedConfig(ControllerConfig):
            FRONT_RESPONSE_POINTS_PLANE = (
                (0.0, 0.0),
                (40.0, 400.0),
            )
            MAX_VY = 4.0

        controller = self.follow.FollowController(LimitedConfig)
        command = controller.update(measurement(mid_y=160.0), now_ms=20)
        self.assertEqual(command[1], 4.0)

        controller.reset()
        state = controller.get_state()
        self.assertIsNone(state["last_measurement_ms"])
        self.assertIsNone(state["plane_pose"])
        self.assertIsNone(controller.get_visual_rigid_command())


if __name__ == "__main__":
    unittest.main()

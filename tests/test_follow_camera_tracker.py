import pathlib
import importlib.util
import math
import sys
import unittest


CAR_ROOT = pathlib.Path(__file__).resolve().parents[1]
TRACKER_DIR = CAR_ROOT / "跟随摄像头"
TRACKER_PATH = TRACKER_DIR / "ir_tracker.py"
CONFIG_PATH = TRACKER_DIR / "config.py"
MAIN_PATH = TRACKER_DIR / "main.py"
ASSISTANT_DIR = CAR_ROOT / "辅助车"
ASSISTANT_FOLLOW_PATH = ASSISTANT_DIR / "follow.py"
ASSISTANT_CONFIG_PATH = ASSISTANT_DIR / "config.py"

sys.path.insert(0, str(TRACKER_DIR))
import ir_tracker as tracker


def blob(cx, cy, pixels=40, w=8, h=8):
    return {
        "cx": cx,
        "cy": cy,
        "pixels": pixels,
        "w": w,
        "h": h,
    }


def make_config(**overrides):
    config = {
        "blob_pixels_min": 8,
        "blob_area_min": 12,
        "blob_aspect_min": 0.35,
        "blob_aspect_max": 2.8,
        "ref_cx": 160.0,
        "ref_cy": 120.0,
        "ref_line_angle_deg": 45.0,
        "ref_distance_px": 56.57,
        "ipm_image_to_plane": (
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        ),
        "ipm_denominator_epsilon": 1.0e-6,
        "ref_plane_angle_deg": 45.0,
        "pair_distance_min_px": 20.0,
        "pair_distance_max_px": 160.0,
        "pair_size_ratio_min": 0.35,
        "pair_angle_tolerance_deg": 30.0,
        "single_right_max_x": 150.0,
        "single_left_min_x": 170.0,
        "single_history_max_jump_px": 60.0,
        "max_midpoint_jump_px": 60.0,
        "max_distance_jump_px": 45.0,
        "max_angle_jump_deg": 35.0,
        "score_ref_mid_weight": 1.0,
        "score_ref_distance_weight": 1.5,
        "score_ref_angle_weight": 1.0,
        "score_size_balance_weight": 0.5,
        "score_history_mid_weight": 2.0,
        "score_history_distance_weight": 2.0,
        "score_history_angle_weight": 2.0,
        "score_mid_scale_px": 80.0,
        "score_distance_scale_px": 60.0,
        "score_angle_scale_deg": 45.0,
        "acquire_confirm_frames": 2,
        "lost_confirm_frames": 1,
        "ema_alpha": 0.35,
    }
    config.update(overrides)
    return config


class FollowCameraScaffoldTests(unittest.TestCase):
    def test_tracker_module_exists_in_requested_folder(self):
        self.assertTrue(
            TRACKER_PATH.is_file(),
            "应在 car/跟随摄像头/ 下创建 ir_tracker.py",
        )

    def test_config_contains_every_tracker_parameter(self):
        self.assertTrue(CONFIG_PATH.is_file(), "应创建跟随摄像头/config.py")
        spec = importlib.util.spec_from_file_location(
            "follow_camera_config",
            CONFIG_PATH,
        )
        camera_config = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(camera_config)

        required = {
            "blob_pixels_min",
            "blob_area_min",
            "blob_aspect_min",
            "blob_aspect_max",
            "ref_cx",
            "ref_cy",
            "ref_line_angle_deg",
            "ref_distance_px",
            "ipm_image_to_plane",
            "ipm_denominator_epsilon",
            "ref_plane_angle_deg",
            "pair_distance_min_px",
            "pair_distance_max_px",
            "pair_size_ratio_min",
            "pair_angle_tolerance_deg",
            "single_right_max_x",
            "single_left_min_x",
            "single_history_max_jump_px",
            "max_midpoint_jump_px",
            "max_distance_jump_px",
            "max_angle_jump_deg",
            "score_ref_mid_weight",
            "score_ref_distance_weight",
            "score_ref_angle_weight",
            "score_size_balance_weight",
            "score_history_mid_weight",
            "score_history_distance_weight",
            "score_history_angle_weight",
            "score_mid_scale_px",
            "score_distance_scale_px",
            "score_angle_scale_deg",
            "acquire_confirm_frames",
            "ema_alpha",
        }
        self.assertTrue(required.issubset(camera_config.TRACKER_CONFIG))
        self.assertEqual(len(camera_config.ROI), 4)

    def test_production_angle_window_allows_turning_correction(self):
        spec = importlib.util.spec_from_file_location(
            "follow_camera_config_angle_window",
            CONFIG_PATH,
        )
        camera_config = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(camera_config)

        pair = tracker.select_best_pair(
            [blob(132, 92), blob(188, 148)],
            camera_config.TRACKER_CONFIG,
        )
        self.assertIsNotNone(pair)
        self.assertAlmostEqual(pair["line_angle_deg"], 45.0)

    def test_openart_main_wires_safe_detection_and_reporting(self):
        self.assertTrue(MAIN_PATH.is_file(), "应创建跟随摄像头/main.py")
        source = MAIN_PATH.read_text(encoding="utf-8")
        required_fragments = (
            "roi=config.ROI",
            "merge=False",
            "IRTracker",
            "ReportScheduler",
            "format_measurement_line",
            "time.ticks_diff",
            "except Exception",
            "sensor.set_auto_exposure(False",
        )
        for fragment in required_fragments:
            self.assertIn(fragment, source)

    def test_production_reference_pose_is_zero_after_uart_round_trip(self):
        camera_spec = importlib.util.spec_from_file_location(
            "follow_camera_integration_config",
            CONFIG_PATH,
        )
        camera_config = importlib.util.module_from_spec(camera_spec)
        camera_spec.loader.exec_module(camera_config)
        follow_spec = importlib.util.spec_from_file_location(
            "assistant_follow_integration",
            ASSISTANT_FOLLOW_PATH,
        )
        assistant_follow = importlib.util.module_from_spec(follow_spec)
        follow_spec.loader.exec_module(assistant_follow)
        assistant_spec = importlib.util.spec_from_file_location(
            "assistant_follow_integration_config",
            ASSISTANT_CONFIG_PATH,
        )
        assistant_config = importlib.util.module_from_spec(assistant_spec)
        assistant_spec.loader.exec_module(assistant_config)

        half_distance = 0.5 * camera_config.REF_DISTANCE_PX
        angle_rad = math.radians(camera_config.REF_LINE_ANGLE_DEG)
        half_dx = half_distance * math.cos(angle_rad)
        half_dy = half_distance * math.sin(angle_rad)
        reference_blobs = [
            blob(
                camera_config.REF_CX - half_dx,
                camera_config.REF_CY - half_dy,
            ),
            blob(
                camera_config.REF_CX + half_dx,
                camera_config.REF_CY + half_dy,
            ),
        ]
        state = tracker.IRTracker(camera_config.TRACKER_CONFIG)
        state.update(reference_blobs)
        camera_measurement = state.update(reference_blobs)
        line = tracker.format_measurement_line(camera_measurement)
        event = assistant_follow.parse_camera_line(line, assistant_config)
        controller = assistant_follow.FollowController(assistant_config)

        self.assertEqual(event["type"], "measurement")
        self.assertEqual(
            controller.update(event["measurement"], now_ms=0),
            (0.0, 0.0, 0.0),
        )

    def test_ipm_theta_sign_keeps_existing_motor_turn_direction(self):
        camera_spec = importlib.util.spec_from_file_location(
            "follow_camera_turn_sign_config",
            CONFIG_PATH,
        )
        camera_config = importlib.util.module_from_spec(camera_spec)
        camera_spec.loader.exec_module(camera_config)
        follow_spec = importlib.util.spec_from_file_location(
            "assistant_follow_turn_sign",
            ASSISTANT_FOLLOW_PATH,
        )
        assistant_follow = importlib.util.module_from_spec(follow_spec)
        follow_spec.loader.exec_module(assistant_follow)
        assistant_spec = importlib.util.spec_from_file_location(
            "assistant_follow_turn_sign_config",
            ASSISTANT_CONFIG_PATH,
        )
        assistant_config = importlib.util.module_from_spec(assistant_spec)
        assistant_spec.loader.exec_module(assistant_config)

        raw_angle_deg = camera_config.REF_LINE_ANGLE_DEG + 8.0
        angle_rad = math.radians(raw_angle_deg)
        half_distance = 0.5 * camera_config.REF_DISTANCE_PX
        half_dx = half_distance * math.cos(angle_rad)
        half_dy = half_distance * math.sin(angle_rad)
        turned_blobs = [
            blob(
                camera_config.REF_CX - half_dx,
                camera_config.REF_CY - half_dy,
            ),
            blob(
                camera_config.REF_CX + half_dx,
                camera_config.REF_CY + half_dy,
            ),
        ]
        state = tracker.IRTracker(camera_config.TRACKER_CONFIG)
        state.update(turned_blobs)
        camera_measurement = state.update(turned_blobs)
        event = assistant_follow.parse_camera_line(
            tracker.format_measurement_line(camera_measurement),
            assistant_config,
        )
        command = assistant_follow.FollowController(
            assistant_config
        ).update(event["measurement"], now_ms=0)

        self.assertLess(camera_measurement["theta_deg"], 0.0)
        self.assertEqual(assistant_config.W_SIGN, 1.0)
        self.assertEqual(assistant_config.SE2_BODY_FRAME_HEADING_SIGN, 1.0)
        # 旧像素协议中同一姿态是正角误差配 W_SIGN=-1；新物理角是
        # 负误差配 W_SIGN=+1，最终 w 方向保持为负。
        self.assertLess(command[2], 0.0)


class PairSelectionTests(unittest.TestCase):
    def test_select_pair_prefers_reference_geometry_over_largest_reflection(self):
        select_pair = getattr(tracker, "select_best_pair", None)
        self.assertIsNotNone(select_pair, "应实现 select_best_pair")

        candidates = [
            blob(140, 100),
            blob(180, 140),
            blob(40, 40, pixels=300, w=20, h=20),
        ]
        pair = select_pair(candidates, make_config())

        self.assertIsNotNone(pair)
        self.assertAlmostEqual(pair["mid_x"], 160.0)
        self.assertAlmostEqual(pair["mid_y"], 120.0)
        self.assertAlmostEqual(pair["distance_px"], 56.5685, places=3)
        self.assertEqual(pair["quality"], 80)

    def test_select_pair_rejects_invalid_distance(self):
        select_pair = getattr(tracker, "select_best_pair", None)
        self.assertIsNotNone(select_pair, "应实现 select_best_pair")
        pair = select_pair([blob(10, 10), blob(15, 10)], make_config())
        self.assertIsNone(pair)

    def test_select_pair_rejects_invalid_size_ratio(self):
        select_pair = getattr(tracker, "select_best_pair", None)
        self.assertIsNotNone(select_pair, "应实现 select_best_pair")
        pair = select_pair(
            [blob(140, 100, pixels=80), blob(180, 140, pixels=8)],
            make_config(),
        )
        self.assertIsNone(pair)

    def test_select_pair_rejects_invalid_blob_shape(self):
        select_pair = getattr(tracker, "select_best_pair", None)
        self.assertIsNotNone(select_pair, "应实现 select_best_pair")
        pair = select_pair(
            [blob(140, 100, w=30, h=2), blob(180, 140)],
            make_config(),
        )
        self.assertIsNone(pair)

    def test_select_pair_rejects_wrong_reference_angle(self):
        pair = tracker.select_best_pair(
            [blob(140, 140), blob(180, 100)],
            make_config(pair_angle_tolerance_deg=20.0),
        )
        self.assertIsNone(pair)

    def test_history_continuity_beats_reference_position_after_lock(self):
        previous = {
            "mid_x": 100.0,
            "mid_y": 90.0,
            "distance_px": 56.5685,
            "line_angle_deg": 45.0,
        }
        candidates = [
            blob(80, 70),
            blob(120, 110),
            blob(140, 100),
            blob(180, 140),
        ]
        pair = tracker.select_best_pair(
            candidates,
            make_config(
                pair_distance_max_px=65.0,
                max_midpoint_jump_px=100.0,
            ),
            previous=previous,
        )
        self.assertAlmostEqual(pair["mid_x"], 100.0)
        self.assertAlmostEqual(pair["mid_y"], 90.0)


class TrackerStateTests(unittest.TestCase):
    @staticmethod
    def valid_blobs():
        return [blob(140, 100), blob(180, 140)]

    def test_tracker_requires_two_frames_and_loses_immediately(self):
        tracker_class = getattr(tracker, "IRTracker", None)
        self.assertIsNotNone(tracker_class, "应实现 IRTracker")

        state = tracker_class(make_config())
        self.assertFalse(state.update(self.valid_blobs())["found"])
        self.assertTrue(state.update(self.valid_blobs())["found"])
        self.assertFalse(state.update([])["found"])

    def test_tracker_confirms_loss_after_configured_consecutive_misses(self):
        tracker_class = getattr(tracker, "IRTracker", None)
        state = tracker_class(
            make_config(acquire_confirm_frames=1, lost_confirm_frames=3)
        )
        self.assertTrue(state.update(self.valid_blobs())["found"])
        self.assertTrue(state.update([])["found"])
        self.assertTrue(state.update([])["found"])
        self.assertFalse(state.update([])["found"])

    def test_transient_miss_keeps_complete_transmittable_pose(self):
        state = tracker.IRTracker(
            make_config(acquire_confirm_frames=1, lost_confirm_frames=2)
        )
        state.update(self.valid_blobs())

        held = state.update([])

        self.assertTrue(held["found"])
        self.assertEqual(len(tracker.format_measurement_line(held).split(",")), 3)

    def test_miss_after_right_single_keeps_zero_theta(self):
        state = tracker.IRTracker(
            make_config(
                ref_line_angle_deg=0.0,
                ref_plane_angle_deg=0.0,
                ref_distance_px=100.0,
                acquire_confirm_frames=1,
                lost_confirm_frames=2,
                ema_alpha=1.0,
            )
        )
        state.update([blob(20, 100), blob(120, 100)])
        state.update([blob(125, 100)])

        held = state.update([])

        self.assertTrue(held["found"])
        self.assertEqual(held["theta_deg"], 0.0)

    def test_tracker_filters_midpoint_and_distance(self):
        tracker_class = getattr(tracker, "IRTracker", None)
        self.assertIsNotNone(tracker_class, "应实现 IRTracker")

        state = tracker_class(
            make_config(ema_alpha=0.5, acquire_confirm_frames=1)
        )
        first = state.update(self.valid_blobs())
        second = state.update([blob(145, 95), blob(195, 145)])

        self.assertAlmostEqual(second["mid_x"], 165.0)
        self.assertAlmostEqual(
            second["distance_px"],
            (first["distance_px"] + 70.710678) * 0.5,
            places=4,
        )

    def test_line_angle_filter_uses_180_degree_period(self):
        filter_angle = getattr(tracker, "ema_line_angle_deg", None)
        self.assertIsNotNone(filter_angle, "应实现 ema_line_angle_deg")
        filtered = filter_angle(89.0, -89.0, 0.5)
        self.assertLess(abs(abs(filtered) - 90.0), 0.001)

    def test_pair_projects_right_light_and_physical_angle_in_camera(self):
        state = tracker.IRTracker(
            make_config(acquire_confirm_frames=1, ema_alpha=1.0)
        )

        result = state.update(self.valid_blobs())

        self.assertEqual(result["mode"], "pair")
        self.assertAlmostEqual(result["target_x_cm"], 180.0)
        self.assertAlmostEqual(result["target_y_cm"], 140.0)
        self.assertAlmostEqual(result["theta_deg"], 0.0)

    def test_locked_tracker_keeps_right_single_and_disables_visual_theta(self):
        state = tracker.IRTracker(
            make_config(
                ref_line_angle_deg=0.0,
                ref_plane_angle_deg=0.0,
                ref_distance_px=100.0,
                acquire_confirm_frames=1,
                ema_alpha=1.0,
            )
        )
        self.assertTrue(
            state.update([blob(20, 100), blob(120, 100)])["found"]
        )

        result = state.update([blob(125, 101)])

        self.assertTrue(result["found"])
        self.assertEqual(result["mode"], "single_right")
        self.assertAlmostEqual(result["target_x_cm"], 125.0)
        self.assertAlmostEqual(result["target_y_cm"], 101.0)
        self.assertEqual(result["theta_deg"], 0.0)

    def test_left_single_is_lost_immediately(self):
        state = tracker.IRTracker(
            make_config(
                ref_line_angle_deg=0.0,
                ref_plane_angle_deg=0.0,
                ref_distance_px=100.0,
                acquire_confirm_frames=1,
                lost_confirm_frames=3,
            )
        )
        self.assertTrue(
            state.update([blob(180, 100), blob(280, 100)])["found"]
        )

        self.assertFalse(state.update([blob(180, 100)])["found"])

    def test_middle_single_uses_previous_endpoint_history(self):
        state = tracker.IRTracker(
            make_config(
                ref_line_angle_deg=0.0,
                ref_plane_angle_deg=0.0,
                ref_distance_px=100.0,
                acquire_confirm_frames=1,
                ema_alpha=1.0,
            )
        )
        self.assertTrue(
            state.update([blob(60, 100), blob(160, 100)])["found"]
        )

        result = state.update([blob(165, 100)])

        self.assertTrue(result["found"])
        self.assertEqual(result["mode"], "single_right")


class ProtocolTests(unittest.TestCase):
    def test_format_measurement_line_uses_physical_pose_triple(self):
        formatter = getattr(tracker, "format_measurement_line", None)
        self.assertIsNotNone(formatter, "应实现 format_measurement_line")

        line = formatter(
            {
                "found": True,
                "target_x_cm": 1.23456,
                "target_y_cm": -6.78901,
                "theta_deg": 4.25,
            }
        )
        self.assertEqual(line, "1.235,-6.789,4.25\n")
        self.assertEqual(len(line.strip().split(",")), 3)

    def test_format_lost_line_uses_single_field(self):
        formatter = getattr(tracker, "format_measurement_line", None)
        self.assertIsNotNone(formatter, "应实现 format_measurement_line")
        line = formatter({"found": False})
        self.assertEqual(line, "0\n")
        self.assertEqual(len(line.strip().split(",")), 1)

    def test_pixel_geometry_and_quality_are_not_transmitted(self):
        formatter = getattr(tracker, "format_measurement_line", None)
        self.assertIsNotNone(formatter, "应实现 format_measurement_line")
        line = formatter(
            {
                "found": True,
                "target_x_cm": 1.0,
                "target_y_cm": -2.0,
                "theta_deg": 3.0,
                "mid_x": 160,
                "mid_y": 120,
                "line_angle_deg": 45,
                "distance_px": 80,
                "quality": 1200,
            }
        )
        self.assertEqual(line, "1.000,-2.000,3.00\n")
        self.assertNotIn("1200", line)

    def test_report_scheduler_sends_lost_immediately_after_found(self):
        scheduler_class = getattr(tracker, "ReportScheduler", None)
        self.assertIsNotNone(scheduler_class, "应实现 ReportScheduler")

        scheduler = scheduler_class(found_interval_ms=30, lost_interval_ms=200)
        self.assertTrue(scheduler.should_send(True, 0))
        self.assertFalse(scheduler.should_send(True, 10))
        self.assertTrue(scheduler.should_send(True, 30))
        self.assertTrue(scheduler.should_send(False, 31))
        self.assertFalse(scheduler.should_send(False, 100))
        self.assertTrue(scheduler.should_send(False, 231))

    def test_report_scheduler_sends_startup_lost_heartbeat(self):
        scheduler_class = getattr(tracker, "ReportScheduler", None)
        self.assertIsNotNone(scheduler_class, "应实现 ReportScheduler")
        scheduler = scheduler_class(found_interval_ms=30, lost_interval_ms=200)
        self.assertTrue(scheduler.should_send(False, 0))
        self.assertFalse(scheduler.should_send(False, 199))
        self.assertTrue(scheduler.should_send(False, 200))


if __name__ == "__main__":
    unittest.main()

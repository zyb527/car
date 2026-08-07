import math
import importlib.util
import os
import sys
import unittest


PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

import wireless_feedforward as feedforward  # noqa: E402
from main_config import NavigationConfig  # noqa: E402


MAIN_PATH = os.path.join(PROJECT_DIR, "main.py")
MAIN_SPEC = importlib.util.spec_from_file_location(
    "main_follow_test_entry",
    MAIN_PATH,
)
main_test = importlib.util.module_from_spec(MAIN_SPEC)
MAIN_SPEC.loader.exec_module(main_test)

ASSISTANT_PROTOCOL_PATH = os.path.join(
    PROJECT_DIR,
    "辅助车",
    "wireless_feedforward.py",
)
ASSISTANT_PROTOCOL_SPEC = importlib.util.spec_from_file_location(
    "assistant_wireless_feedforward",
    ASSISTANT_PROTOCOL_PATH,
)
assistant_feedforward = importlib.util.module_from_spec(
    ASSISTANT_PROTOCOL_SPEC
)
ASSISTANT_PROTOCOL_SPEC.loader.exec_module(assistant_feedforward)


class FakeWireless:
    def __init__(self, chunks=None):
        self.sent = []
        self.chunks = list(chunks or [])

    def send_bytearray(self, buffer, length):
        payload = bytearray(length)
        for index in range(length):
            value = buffer[index]
            payload[index] = value if value >= 0 else value + 256
        self.sent.append(bytes(payload))

    def receive_bytearray(self, buffer, length):
        if not self.chunks:
            return 0
        chunk = self.chunks.pop(0)
        chunk = chunk[:length]
        for index, value in enumerate(chunk):
            buffer[index] = value if value < 128 else value - 256
        return len(chunk)


class FakeMotor:
    def get_limited_command(self):
        return 3.0, -4.0, 0.5


class FakePhysicalMotor:
    def get_limited_physical_command(self):
        # w 模拟主车电机内部生成的航向保持修正。
        return 8.0, 9.0, -0.4


class FakeOdometry:
    def get_state(self):
        return {
            "body_vx_cm_s": 6.5,
            "body_vy_cm_s": -7.25,
            "yaw_rate_rad_s": 0.35,
        }


class ProtocolTests(unittest.TestCase):
    def test_frame_round_trip(self):
        frame = feedforward.encode_feedforward(12.5, -7.25, 0.42, 17)
        self.assertEqual(len(frame), feedforward.FRAME_SIZE)
        decoded = feedforward.decode_feedforward(frame)
        self.assertAlmostEqual(decoded["vx"], 12.5)
        self.assertAlmostEqual(decoded["vy"], -7.25)
        self.assertAlmostEqual(decoded["w"], 0.42)
        self.assertEqual(decoded["sequence"], 17)

    def test_sender_uses_motor_limited_command(self):
        wireless = FakeWireless()
        sender = feedforward.FeedforwardSender(wireless=wireless)
        command = sender.send_motor_command(FakeMotor())

        self.assertEqual(command, (3.0, -4.0, 0.5))
        decoded = feedforward.decode_feedforward(wireless.sent[0])
        self.assertEqual(
            (decoded["vx"], decoded["vy"], decoded["w"]),
            (3.0, -4.0, 0.5),
        )

    def test_straight_without_w_suppresses_local_heading_hold_w(self):
        wireless = FakeWireless()
        sender = feedforward.FeedforwardSender(wireless=wireless)

        command = sender.send_motor_command(
            FakePhysicalMotor(),
            straight_without_w=True,
        )

        self.assertEqual(command, (8.0, 9.0, 0.0))
        decoded = feedforward.decode_feedforward(wireless.sent[0])
        self.assertEqual(
            (decoded["vx"], decoded["vy"], decoded["w"]),
            (8.0, 9.0, 0.0),
        )

    def test_active_turn_keeps_motor_limited_w(self):
        wireless = FakeWireless()
        sender = feedforward.FeedforwardSender(wireless=wireless)

        command = sender.send_motor_command(FakePhysicalMotor())

        self.assertEqual(command, (8.0, 9.0, -0.4))
        decoded = feedforward.decode_feedforward(wireless.sent[0])
        self.assertAlmostEqual(decoded["w"], -0.4)

    def test_sender_uses_encoder_translation_and_imu_yaw_rate(self):
        wireless = FakeWireless()
        sender = feedforward.FeedforwardSender(wireless=wireless)

        command = sender.send_measured_motion(FakeOdometry())

        self.assertEqual(command, (6.5, -7.25, 0.35))
        decoded = feedforward.decode_feedforward(wireless.sent[0])
        self.assertAlmostEqual(decoded["vx"], 6.5)
        self.assertAlmostEqual(decoded["vy"], -7.25)
        self.assertAlmostEqual(decoded["w"], 0.35)

    def test_measured_motion_can_suppress_yaw_rate_only(self):
        wireless = FakeWireless()
        sender = feedforward.FeedforwardSender(wireless=wireless)

        command = sender.send_measured_motion(
            FakeOdometry(),
            straight_without_w=True,
        )

        self.assertEqual(command, (6.5, -7.25, 0.0))
        decoded = feedforward.decode_feedforward(wireless.sent[0])
        self.assertEqual(
            (decoded["vx"], decoded["vy"], decoded["w"]),
            (6.5, -7.25, 0.0),
        )

    def test_measured_motion_respects_send_period(self):
        wireless = FakeWireless()
        sender = feedforward.FeedforwardSender(
            wireless=wireless,
            period_ms=10,
        )
        sender.last_tx_ms = 100

        self.assertIsNone(
            sender.send_measured_motion_if_due(FakeOdometry(), now_ms=109)
        )
        self.assertEqual(
            sender.send_measured_motion_if_due(FakeOdometry(), now_ms=110),
            (6.5, -7.25, 0.35),
        )
        self.assertEqual(len(wireless.sent), 1)

    def test_blended_motion_uses_configured_measured_weight(self):
        wireless = FakeWireless()
        sender = feedforward.FeedforwardSender(wireless=wireless)

        command = sender.send_blended_motion(
            FakePhysicalMotor(),
            FakeOdometry(),
            measured_weight=0.25,
        )

        expected = (7.625, 4.9375, -0.2125)
        for actual, value in zip(command, expected):
            self.assertAlmostEqual(actual, value)
        decoded = feedforward.decode_feedforward(wireless.sent[0])
        for actual, value in zip(
            (decoded["vx"], decoded["vy"], decoded["w"]),
            expected,
        ):
            self.assertAlmostEqual(actual, value)

    def test_blended_motion_suppresses_w_after_blending(self):
        wireless = FakeWireless()
        sender = feedforward.FeedforwardSender(wireless=wireless)

        command = sender.send_blended_motion(
            FakePhysicalMotor(),
            FakeOdometry(),
            measured_weight=0.25,
            straight_without_w=True,
        )

        self.assertEqual(command, (7.625, 4.9375, 0.0))

    def test_blended_motion_rejects_weight_outside_unit_interval(self):
        sender = feedforward.FeedforwardSender(wireless=FakeWireless())

        for value in (-0.01, 1.01, math.nan):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    sender.send_blended_motion(
                        FakePhysicalMotor(),
                        FakeOdometry(),
                        measured_weight=value,
                    )

    def test_main_and_assistant_protocol_files_are_interoperable(self):
        main_frame = feedforward.encode_feedforward(1.0, 2.0, 0.3, 7)
        assistant_decoded = assistant_feedforward.decode_feedforward(
            main_frame
        )
        self.assertAlmostEqual(assistant_decoded["vx"], 1.0)
        self.assertAlmostEqual(assistant_decoded["vy"], 2.0)
        self.assertAlmostEqual(assistant_decoded["w"], 0.3)
        self.assertEqual(assistant_decoded["sequence"], 7)

    def test_receiver_drains_noise_and_split_frame_then_times_out(self):
        frame = feedforward.encode_feedforward(8.0, 9.0, -0.3, 22)
        wireless = FakeWireless(
            [b"\x00noise" + frame[:13], frame[13:]]
        )
        receiver = feedforward.FeedforwardReceiver(
            wireless=wireless,
            timeout_ms=250,
        )

        command = receiver.poll(100)
        self.assertAlmostEqual(command[0], 8.0)
        self.assertAlmostEqual(command[1], 9.0)
        self.assertAlmostEqual(command[2], -0.3)
        self.assertEqual(receiver.last_sequence, 22)
        self.assertEqual(receiver.get_command(350), command)
        self.assertIsNone(receiver.get_command(351))

    def test_receiver_uses_latest_complete_frame_from_backlog(self):
        first = feedforward.encode_feedforward(1.0, 2.0, 0.1, 30)
        latest = feedforward.encode_feedforward(7.0, 8.0, 0.7, 31)
        receiver = feedforward.FeedforwardReceiver(
            wireless=FakeWireless([first, latest]),
            timeout_ms=250,
        )

        command = receiver.poll(100)

        self.assertAlmostEqual(command[0], 7.0)
        self.assertAlmostEqual(command[1], 8.0)
        self.assertAlmostEqual(command[2], 0.7)
        self.assertEqual(receiver.last_sequence, 31)

    def test_non_finite_command_is_rejected(self):
        with self.assertRaises(ValueError):
            feedforward.encode_feedforward(math.nan, 0.0, 0.0)

    def test_visual_plus_feedforward_reuses_existing_limits(self):
        command = feedforward.combine_commands(
            (15.0, 40.0, 0.7),
            (10.0, 20.0, 0.3),
            20.0,
            45.0,
            0.8,
        )
        self.assertEqual(command, (20.0, 45.0, 0.8))


class MainTestSequenceTests(unittest.TestCase):
    def test_only_translation_with_exact_zero_w_suppresses_wireless_w(self):
        self.assertTrue(
            main_test._is_translation_without_rotation((20.0, 0.0, 0.0))
        )
        self.assertTrue(
            main_test._is_translation_without_rotation((0.0, -20.0, 0.0))
        )
        self.assertFalse(
            main_test._is_translation_without_rotation((0.0, 0.0, 0.0))
        )
        self.assertFalse(
            main_test._is_translation_without_rotation((20.0, 0.0, 0.1))
        )
        self.assertFalse(
            main_test._is_translation_without_rotation((0.0, 0.0, -0.1))
        )

    def test_sequence_and_signs_match_body_coordinate_convention(self):
        commands = [stage[1] for stage in main_test.TEST_STAGES]
        slow_turn_boundary_w = (
            NavigationConfig.TURN_SLOW_KP
            * NavigationConfig.TURN_MID_ERROR_RAD
        )
        self.assertEqual(
            commands,
            [
                (0.0, NavigationConfig.PATH_SLOW_SPEED_CM_S, 0.0),
                (0.0, -NavigationConfig.PATH_SLOW_SPEED_CM_S, 0.0),
                (NavigationConfig.PATH_SLOW_SPEED_CM_S, 0.0, 0.0),
                (-NavigationConfig.PATH_SLOW_SPEED_CM_S, 0.0, 0.0),
                (0.0, 0.0, -slow_turn_boundary_w),
                (0.0, 0.0, slow_turn_boundary_w),
            ],
        )


if __name__ == "__main__":
    unittest.main()

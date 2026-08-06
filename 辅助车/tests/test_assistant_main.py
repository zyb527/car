import importlib.util
import io
import math
import pathlib
import unittest
from contextlib import redirect_stdout


ASSISTANT_DIR = pathlib.Path(__file__).resolve().parents[1]
MAIN_PATH = ASSISTANT_DIR / "main.py"


def load_main_module():
    spec = importlib.util.spec_from_file_location("assistant_main_runtime", MAIN_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def valid_event(mid_x=160.0):
    return {
        "type": "measurement",
        "measurement": {
            "found": True,
            "mid_x": mid_x,
            "mid_y": 120.0,
            "line_angle_deg": 45.0,
            "distance_px": 80.0,
            "quality": 60,
        },
    }


class RuntimeConfig:
    REACQUIRE_FRAMES = 2
    CAMERA_TIMEOUT_MS = 150
    VISUAL_LOSS_DECAY_MS = 250
    MAX_VX = 20.0
    MAX_VY = 45.0
    MAX_W = 0.8
    MAX_COMMAND_VX = 200.0
    MAX_COMMAND_VY = 200.0
    MAX_COMMAND_W = 3.4
    FORMATION_RIGHT_OFFSET_CM = 0.0
    FORMATION_FORWARD_OFFSET_CM = 0.0


class LogConfig:
    OFFLINE_LOG_ENABLED = True
    OFFLINE_LOG_PATH = "unused-test-log.txt"
    OFFLINE_LOG_PERIOD_MS = 200
    OFFLINE_LOG_FLUSH_MS = 1000
    OFFLINE_LOG_MAX_BYTES = 4096


class FakeSensor:
    def __init__(self, batches):
        self.batches = list(batches)

    def poll(self, now_ms):
        return self.batches.pop(0) if self.batches else []


class FakeController:
    def __init__(self, command=(1.0, 2.0, 0.3)):
        self.command = command
        self.reset_count = 0
        self.updates = []

    def update(self, measurement, now_ms):
        self.updates.append((measurement, now_ms))
        return self.command

    def reset(self):
        self.reset_count += 1


class HeadingController(FakeController):
    def __init__(
        self,
        relative_heading_rad,
        command=(0.0, 0.0, 0.0),
        raw_relative_heading_rad=None,
    ):
        super().__init__(command=command)
        self.relative_heading_rad = float(relative_heading_rad)
        self.raw_relative_heading_rad = raw_relative_heading_rad

    def get_state(self):
        state = {
            "relative_heading_rad": self.relative_heading_rad,
        }
        if self.raw_relative_heading_rad is not None:
            state["raw_relative_heading_rad"] = float(
                self.raw_relative_heading_rad
            )
        return state


class VisualRigidController(FakeController):
    def __init__(self, command, rigid_command):
        super().__init__(command=command)
        self.rigid_command = rigid_command
        self.twists = []

    def set_follower_twist(self, vx, vy, w):
        self.twists.append((vx, vy, w))

    def get_visual_rigid_command(self):
        return self.rigid_command


class IntermittentInvalidController(FakeController):
    def __init__(self, outcomes):
        super().__init__()
        self.outcomes = list(outcomes)

    def update(self, measurement, now_ms):
        outcome = self.outcomes.pop(0)
        if isinstance(outcome, Exception):
            raise outcome
        self.command = outcome
        return outcome


class FakeOdometry:
    def __init__(self, twist):
        self.twist = twist

    def get_state(self):
        return {
            "body_vx_cm_s": self.twist[0],
            "body_vy_cm_s": self.twist[1],
            "yaw_rate_rad_s": self.twist[2],
        }


class FakeMotor:
    def __init__(self):
        self.operations = []

    def move(self, vx, vy, w):
        self.operations.append(("move", vx, vy, w))

    def command(self, vx, vy, w):
        self.operations.append(("command", vx, vy, w))

    def hard_stop(self):
        self.operations.append(("hard_stop",))


class FakeFeedforward:
    def __init__(self, commands):
        self.commands = list(commands)
        self.reset_count = 0
        self.poll_count = 0

    def reset(self):
        self.reset_count += 1

    def poll(self, now_ms):
        self.poll_count += 1
        if self.commands:
            return self.commands.pop(0)
        return None


class SequencedFeedforward:
    def __init__(self, commands):
        self.commands = list(commands)
        self.last_sequence = None
        self.last_update_ms = None

    def reset(self):
        self.last_sequence = None
        self.last_update_ms = None

    def poll(self, now_ms):
        if not self.commands:
            return None
        command = self.commands.pop(0)
        self.last_sequence = (
            0 if self.last_sequence is None else self.last_sequence + 1
        )
        self.last_update_ms = now_ms
        return command


class FakeCameraUART:
    def __init__(self, supported_keywords):
        self.supported_keywords = set(supported_keywords)
        self.calls = []

    def init(self, baud, **kwargs):
        self.calls.append((baud, kwargs))
        unsupported = set(kwargs) - self.supported_keywords
        if unsupported:
            raise TypeError("unsupported keyword")


class AssistantRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.main = load_main_module()

    def make_runtime(self, batches):
        self.motor = FakeMotor()
        self.controller = FakeController()
        self.sensor = FakeSensor(batches)
        runtime = self.main.AssistantRuntime(
            self.motor,
            self.sensor,
            self.controller,
            RuntimeConfig,
        )
        return runtime

    def test_camera_uart_init_uses_zero_timeout_when_supported(self):
        class UartConfig:
            BAUD = 115200
            UART_TIMEOUT_MS = 0
            UART_TIMEOUT_CHAR_MS = 0

        uart = FakeCameraUART({"timeout", "timeout_char"})
        mode = self.main._init_camera_uart(uart, UartConfig)

        self.assertEqual(mode, "timeout_char")
        self.assertEqual(
            uart.calls,
            [(115200, {"timeout": 0, "timeout_char": 0})],
        )

    def test_infrared_follow_switch_uses_only_wireless_feedforward(self):
        class FeedforwardOnlyConfig(RuntimeConfig):
            INFRARED_FOLLOW_ENABLED = False

        motor = FakeMotor()
        sensor = FakeSensor([[valid_event()]])
        controller = HeadingController(
            relative_heading_rad=math.radians(25.0),
            command=(50.0, 60.0, 0.7),
        )
        runtime = self.main.AssistantRuntime(
            motor,
            sensor,
            controller,
            FeedforwardOnlyConfig,
            feedforward=FakeFeedforward([(12.0, 34.0, 0.5)]),
        )
        runtime.start()

        state = runtime.step(20)

        self.assertEqual(state, self.main.FEEDFORWARD_ONLY)
        self.assertEqual(runtime.last_visual_command, None)
        self.assertEqual(runtime.se2_body_heading_rad, 0.0)
        self.assertEqual(controller.updates, [])
        self.assertEqual(len(sensor.batches), 1)
        self.assertEqual(
            motor.operations[-1],
            ("command", 12.0, 34.0, 0.5),
        )

    def test_infrared_follow_switch_hard_stops_without_feedforward(self):
        class FeedforwardOnlyConfig(RuntimeConfig):
            INFRARED_FOLLOW_ENABLED = False

        motor = FakeMotor()
        sensor = FakeSensor([[valid_event()]])
        controller = FakeController(command=(50.0, 60.0, 0.7))
        runtime = self.main.AssistantRuntime(
            motor,
            sensor,
            controller,
            FeedforwardOnlyConfig,
            feedforward=FakeFeedforward([None]),
        )
        runtime.start()

        state = runtime.step(20)

        self.assertEqual(state, self.main.LOST_STOP)
        self.assertEqual(runtime.last_event_type, "missing_feedforward")
        self.assertEqual(controller.updates, [])
        self.assertEqual(len(sensor.batches), 1)
        self.assertEqual(motor.operations[-1], ("hard_stop",))

    def test_camera_uart_init_falls_back_for_older_firmware(self):
        class UartConfig:
            BAUD = 115200
            UART_TIMEOUT_MS = 0
            UART_TIMEOUT_CHAR_MS = 0

        uart = FakeCameraUART(set())
        mode = self.main._init_camera_uart(uart, UartConfig)

        self.assertEqual(mode, "basic")
        self.assertEqual(len(uart.calls), 3)
        self.assertEqual(uart.calls[-1], (115200, {}))

    def test_offline_logger_records_feedforward_visual_heading_and_final(self):
        class DiagnosticReceiver:
            last_update_ms = 180
            last_sequence = 27

        runtime = self.make_runtime([])
        runtime.feedforward = DiagnosticReceiver()
        runtime.state = self.main.TRACKING
        runtime.last_feedforward_command = (10.0, 15.0, 0.3)
        runtime.last_visual_command = (-2.0, 1.0, -0.1)
        runtime.se2_body_heading_rad = 0.05
        runtime.last_command = (8.0, 16.0, 0.2)

        stream = io.StringIO()
        logger = self.main.OfflineLogger(LogConfig, stream=stream)
        self.assertTrue(logger.sample(200, runtime, step_elapsed_ms=3))
        self.assertFalse(logger.sample(300, runtime, step_elapsed_ms=4))
        self.assertTrue(logger.sample(400, runtime, step_elapsed_ms=5))
        logger.flush(400)

        lines = stream.getvalue().splitlines()
        self.assertIn("ff_age_ms", lines[0])
        self.assertEqual(len(lines), 3)
        fields = lines[1].split(",")
        self.assertEqual(fields[0], "200")
        self.assertEqual(fields[1], self.main.TRACKING)
        self.assertEqual(fields[2], "3")
        self.assertEqual(fields[3:8], ["0", "0", "0", "0", "0"])
        self.assertEqual(fields[8:11], ["1", "20", "27"])
        self.assertEqual(fields[11:14], ["10.0000", "15.0000", "0.3000"])
        self.assertEqual(fields[14:17], ["-2.0000", "1.0000", "-0.1000"])
        self.assertEqual(fields[17], "0.0500")
        self.assertEqual(fields[18:21], ["8.0000", "16.0000", "0.2000"])

    def test_start_is_hard_stopped_and_two_clean_cycles_are_required(self):
        runtime = self.make_runtime([[valid_event()], [valid_event()]])
        runtime.start()
        self.assertEqual(self.motor.operations, [("hard_stop",)])

        runtime.step(20)
        self.assertEqual(runtime.state, self.main.WAIT_TARGET)
        self.assertEqual(len(self.motor.operations), 1)

        runtime.step(40)
        self.assertEqual(runtime.state, self.main.TRACKING)
        self.assertEqual(self.motor.operations[-1], ("command", 1.0, 2.0, 0.3))

    def test_tracking_reuses_last_safe_command_without_reintegrating(self):
        runtime = self.make_runtime([[valid_event()], [valid_event()], []])
        runtime.start()
        runtime.step(20)
        runtime.step(40)
        update_count = len(self.controller.updates)

        before = len(self.motor.operations)
        runtime.step(60)

        self.assertEqual(len(self.controller.updates), update_count)
        self.assertEqual(len(self.motor.operations) - before, 1)
        self.assertEqual(self.motor.operations[-1], ("command", 1.0, 2.0, 0.3))

    def test_lost_and_timeout_each_hard_stop_immediately(self):
        for unsafe_type in ("lost", "timeout"):
            runtime = self.make_runtime(
                [
                    [valid_event()],
                    [valid_event()],
                    [{"type": unsafe_type}],
                ]
            )
            runtime.start()
            runtime.step(20)
            runtime.step(40)

            before = len(self.motor.operations)
            runtime.step(60)

            self.assertEqual(runtime.state, self.main.LOST_STOP)
            self.assertEqual(len(self.motor.operations) - before, 1)
            self.assertEqual(self.motor.operations[-1], ("hard_stop",))
            self.assertIsNone(runtime.last_command)

    def test_single_invalid_frame_keeps_last_visual_command(self):
        runtime = self.make_runtime(
            [
                [valid_event()],
                [valid_event()],
                [{"type": "invalid", "reason": "field_count"}],
            ]
        )
        runtime.start()
        runtime.step(20)
        runtime.step(40)

        before = len(self.motor.operations)
        runtime.step(60)

        self.assertEqual(runtime.state, self.main.TRACKING)
        self.assertEqual(len(self.motor.operations) - before, 1)
        self.assertEqual(self.motor.operations[-1], ("command", 1.0, 2.0, 0.3))
        self.assertEqual(runtime.last_event_detail, "field_count")

    def test_unsafe_event_cannot_be_overwritten_by_later_valid_frame(self):
        runtime = self.make_runtime(
            [
                [valid_event()],
                [valid_event()],
                [{"type": "lost"}, valid_event(mid_x=170.0)],
            ]
        )
        runtime.start()
        runtime.step(20)
        runtime.step(40)

        before = len(self.motor.operations)
        runtime.step(60)

        self.assertEqual(runtime.state, self.main.LOST_STOP)
        self.assertEqual(len(self.motor.operations) - before, 1)
        self.assertEqual(self.motor.operations[-1], ("hard_stop",))
        self.assertEqual(runtime.acquire_count, 0)

    def test_reacquire_does_not_restore_old_command(self):
        runtime = self.make_runtime(
            [
                [valid_event()],
                [valid_event()],
                [{"type": "lost"}],
                [valid_event(mid_x=170.0)],
                [valid_event(mid_x=170.0)],
            ]
        )
        runtime.start()
        runtime.step(20)
        runtime.step(40)
        runtime.step(60)
        reset_after_loss = self.controller.reset_count

        runtime.step(80)
        self.assertEqual(runtime.state, self.main.LOST_STOP)
        self.assertIsNone(runtime.last_command)

        runtime.step(100)
        self.assertEqual(runtime.state, self.main.TRACKING)
        self.assertGreater(self.controller.reset_count, reset_after_loss)
        self.assertEqual(self.motor.operations[-1], ("command", 1.0, 2.0, 0.3))

    def test_each_step_submits_at_most_one_motor_operation(self):
        runtime = self.make_runtime(
            [
                [valid_event()],
                [valid_event()],
                [{"type": "invalid"}, valid_event(), {"type": "lost"}],
            ]
        )
        runtime.start()

        for now_ms in (20, 40, 60):
            before = len(self.motor.operations)
            runtime.step(now_ms)
            self.assertLessEqual(len(self.motor.operations) - before, 1)

    def test_fresh_feedforward_is_submitted_before_slow_visual_poll(self):
        class OrderSensor:
            def __init__(self, motor):
                self.motor = motor
                self.operation_seen_during_poll = None

            def poll(self, _now_ms):
                self.operation_seen_during_poll = self.motor.operations[-1]
                return [valid_event()]

        self.motor = FakeMotor()
        sensor = OrderSensor(self.motor)
        controller = FakeController(command=(3.0, 4.0, 0.2))
        runtime = self.main.AssistantRuntime(
            self.motor,
            sensor,
            controller,
            RuntimeConfig,
            feedforward=FakeFeedforward([(10.0, 12.0, 0.1)]),
        )
        runtime.start()
        runtime.state = self.main.TRACKING
        runtime._set_raw_visual_command((1.0, 2.0, 0.3))

        runtime.step(20)

        self.assertEqual(
            sensor.operation_seen_during_poll,
            ("command", 11.0, 14.0, 0.4),
        )
        self.assertEqual(runtime.last_visual_command, (3.0, 4.0, 0.2))
        self.assertEqual(
            self.motor.operations[-1],
            ("command", 11.0, 14.0, 0.4),
        )

    def test_feedforward_mode_uses_separate_gentle_visual_parameters(self):
        class GentleConfig(RuntimeConfig):
            REACQUIRE_FRAMES = 1
            FEEDFORWARD_VISUAL_VX_SCALE = 0.5
            FEEDFORWARD_VISUAL_VY_SCALE = 0.5
            FEEDFORWARD_VISUAL_W_SCALE = 0.5
            FEEDFORWARD_VISUAL_MAX_VX = 80.0
            FEEDFORWARD_VISUAL_MAX_VY = 80.0
            FEEDFORWARD_VISUAL_MAX_W = 1.5
            FEEDFORWARD_VISUAL_FILTER_ALPHA = 1.0
            FEEDFORWARD_VISUAL_MODE_BLEND_ALPHA = 1.0

        raw_visual = (200.0, -200.0, 3.0)

        pure_motor = FakeMotor()
        pure_runtime = self.main.AssistantRuntime(
            pure_motor,
            FakeSensor([[valid_event()]]),
            FakeController(command=raw_visual),
            GentleConfig,
        )
        pure_runtime.start()
        pure_runtime.step(20)
        self.assertEqual(pure_runtime.last_visual_command, raw_visual)

        ff_motor = FakeMotor()
        ff_runtime = self.main.AssistantRuntime(
            ff_motor,
            FakeSensor([[valid_event()], []]),
            FakeController(command=raw_visual),
            GentleConfig,
            feedforward=FakeFeedforward(
                [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0)]
            ),
        )
        ff_runtime.start()
        ff_runtime.step(20)
        ff_runtime.step(40)

        self.assertEqual(
            ff_runtime.last_visual_command,
            (80.0, -80.0, 1.5),
        )
        self.assertEqual(
            ff_motor.operations[-1],
            ("command", 80.0, -80.0, 1.5),
        )

    def test_visual_parameter_mode_switch_is_blended(self):
        class BlendConfig(RuntimeConfig):
            FEEDFORWARD_VISUAL_VX_SCALE = 0.5
            FEEDFORWARD_VISUAL_VY_SCALE = 0.5
            FEEDFORWARD_VISUAL_W_SCALE = 0.5
            FEEDFORWARD_VISUAL_MAX_VX = 200.0
            FEEDFORWARD_VISUAL_MAX_VY = 200.0
            FEEDFORWARD_VISUAL_MAX_W = 3.0
            FEEDFORWARD_VISUAL_FILTER_ALPHA = 1.0
            FEEDFORWARD_VISUAL_MODE_BLEND_ALPHA = 0.5

        runtime = self.main.AssistantRuntime(
            FakeMotor(),
            FakeSensor([]),
            FakeController(),
            BlendConfig,
        )
        runtime._set_raw_visual_command((100.0, 40.0, 2.0))
        self.assertEqual(runtime.last_visual_command, (100.0, 40.0, 2.0))

        runtime._advance_feedforward_visual_mode(True)
        self.assertEqual(runtime.last_visual_command, (75.0, 30.0, 1.5))

        runtime._advance_feedforward_visual_mode(True)
        self.assertEqual(runtime.last_visual_command, (62.5, 25.0, 1.25))

        runtime._advance_feedforward_visual_mode(False)
        self.assertEqual(runtime.last_visual_command, (81.25, 32.5, 1.625))

    def test_feedforward_repoll_submits_new_frame_before_controller(self):
        class ImmediateConfig(RuntimeConfig):
            REACQUIRE_FRAMES = 1

        class OrderController(FakeController):
            def __init__(self, motor):
                super().__init__(command=(1.0, 2.0, 0.1))
                self.motor = motor
                self.operation_seen_during_update = None

            def update(self, measurement, now_ms):
                self.operation_seen_during_update = self.motor.operations[-1]
                return super().update(measurement, now_ms)

        self.motor = FakeMotor()
        controller = OrderController(self.motor)
        runtime = self.main.AssistantRuntime(
            self.motor,
            FakeSensor([[valid_event()]]),
            controller,
            ImmediateConfig,
            feedforward=SequencedFeedforward(
                [
                    (0.0, 0.0, 0.0),
                    (0.0, 90.0, 0.0),
                ]
            ),
        )
        runtime.start()

        runtime.step(20)

        self.assertEqual(
            controller.operation_seen_during_update,
            ("command", 0.0, 90.0, 0.0),
        )
        self.assertEqual(
            self.motor.operations[-1],
            ("command", 0.0, 90.0, 0.0),
        )

    def test_wireless_feedforward_is_added_to_visual_correction(self):
        self.motor = FakeMotor()
        self.controller = FakeController(command=(1.0, 2.0, 0.1))
        self.sensor = FakeSensor([[valid_event()], [valid_event()]])
        feedforward = FakeFeedforward(
            [
                (10.0, 12.0, 0.2),
                (10.0, 12.0, 0.2),
                (10.0, 12.0, 0.2),
            ]
        )
        runtime = self.main.AssistantRuntime(
            self.motor,
            self.sensor,
            self.controller,
            RuntimeConfig,
            feedforward=feedforward,
        )
        runtime.start()
        runtime.step(20)
        runtime.step(40)
        runtime.step(60)

        self.assertEqual(
            self.motor.operations[-1],
            ("command", 11.0, 14.0, 0.30000000000000004),
        )

    def test_stale_feedforward_falls_back_to_visual_command(self):
        self.motor = FakeMotor()
        self.controller = FakeController(command=(1.0, 2.0, 0.1))
        self.sensor = FakeSensor([[valid_event()], [valid_event()], []])
        feedforward = FakeFeedforward(
            [
                (10.0, 12.0, 0.2),
                (10.0, 12.0, 0.2),
                None,
            ]
        )
        runtime = self.main.AssistantRuntime(
            self.motor,
            self.sensor,
            self.controller,
            RuntimeConfig,
            feedforward=feedforward,
        )
        runtime.start()
        runtime.step(20)
        runtime.step(40)
        runtime.step(60)

        self.assertEqual(
            self.motor.operations[-1],
            ("command", 1.0, 2.0, 0.1),
        )

    def test_fresh_feedforward_continues_when_visual_is_lost(self):
        self.motor = FakeMotor()
        self.controller = FakeController(command=(1.0, 2.0, 0.1))
        self.sensor = FakeSensor(
            [[valid_event()], [valid_event()], [{"type": "lost"}]]
        )
        feedforward = FakeFeedforward(
            [
                (90.0, 80.0, 0.7),
                (90.0, 80.0, 0.7),
                (90.0, 80.0, 0.7),
            ]
        )
        runtime = self.main.AssistantRuntime(
            self.motor,
            self.sensor,
            self.controller,
            RuntimeConfig,
            feedforward=feedforward,
        )
        runtime.start()
        runtime.step(20)
        runtime.step(40)
        runtime.step(60)

        self.assertEqual(runtime.state, self.main.FEEDFORWARD_ONLY)
        self.assertEqual(
            self.motor.operations[-1],
            ("command", 91.0, 82.0, 0.7999999999999999),
        )

    def test_visual_correction_decays_after_loss_with_fresh_feedforward(self):
        class ImmediateConfig(RuntimeConfig):
            REACQUIRE_FRAMES = 1

        self.motor = FakeMotor()
        runtime = self.main.AssistantRuntime(
            self.motor,
            FakeSensor(
                [
                    [valid_event()],
                    [{"type": "lost"}],
                    [],
                    [],
                ]
            ),
            FakeController(command=(1.0, 2.0, 0.1)),
            ImmediateConfig,
            feedforward=FakeFeedforward(
                [
                    (90.0, 80.0, 0.7),
                    (90.0, 80.0, 0.7),
                    (90.0, 80.0, 0.7),
                    (90.0, 80.0, 0.7),
                ]
            ),
        )
        runtime.start()
        runtime.step(0)
        runtime.step(100)
        self.assertEqual(
            self.motor.operations[-1],
            ("command", 91.0, 82.0, 0.7999999999999999),
        )

        runtime.step(225)
        self.assertEqual(
            self.motor.operations[-1],
            ("command", 90.5, 81.0, 0.75),
        )

        runtime.step(350)
        self.assertEqual(
            self.motor.operations[-1],
            ("command", 90.0, 80.0, 0.7),
        )

    def test_feedforward_only_stops_after_wireless_expires(self):
        self.motor = FakeMotor()
        self.controller = FakeController()
        self.sensor = FakeSensor(
            [[valid_event()], [valid_event()], [{"type": "lost"}], []]
        )
        feedforward = FakeFeedforward(
            [
                (90.0, 80.0, 0.7),
                (90.0, 80.0, 0.7),
                (90.0, 80.0, 0.7),
                None,
            ]
        )
        runtime = self.main.AssistantRuntime(
            self.motor,
            self.sensor,
            self.controller,
            RuntimeConfig,
            feedforward=feedforward,
        )
        runtime.start()
        runtime.step(20)
        runtime.step(40)
        runtime.step(60)
        runtime.step(80)

        self.assertEqual(runtime.state, self.main.LOST_STOP)
        self.assertEqual(self.motor.operations[-1], ("hard_stop",))

    def test_combined_command_is_not_clipped_by_visual_trim_limits(self):
        self.motor = FakeMotor()
        self.controller = FakeController(command=(1.0, 2.0, 0.1))
        self.sensor = FakeSensor(
            [[valid_event()], [valid_event()], []]
        )
        feedforward = FakeFeedforward(
            [
                (90.0, 80.0, 0.7),
                (90.0, 80.0, 0.7),
                (90.0, 80.0, 0.7),
            ]
        )
        runtime = self.main.AssistantRuntime(
            self.motor,
            self.sensor,
            self.controller,
            RuntimeConfig,
            feedforward=feedforward,
        )
        runtime.start()
        runtime.step(20)
        runtime.step(40)
        runtime.step(60)

        self.assertEqual(
            self.motor.operations[-1],
            ("command", 91.0, 82.0, 0.7999999999999999),
        )

    def test_visual_rigid_command_is_used_only_without_wireless_feedforward(self):
        class ImmediateConfig(RuntimeConfig):
            REACQUIRE_FRAMES = 1

        self.motor = FakeMotor()
        controller = VisualRigidController(
            command=(1.0, 2.0, 0.1),
            rigid_command=(20.0, 30.0, 0.4),
        )
        runtime = self.main.AssistantRuntime(
            self.motor,
            FakeSensor([[valid_event()]]),
            controller,
            ImmediateConfig,
            odometry=FakeOdometry((3.0, 4.0, 0.2)),
        )
        runtime.start()
        runtime.step(20)

        self.assertEqual(controller.twists, [(3.0, 4.0, 0.2)])
        self.assertEqual(
            self.motor.operations[-1],
            ("command", 21.0, 32.0, 0.5),
        )

        self.motor = FakeMotor()
        controller = VisualRigidController(
            command=(1.0, 2.0, 0.1),
            rigid_command=(20.0, 30.0, 0.4),
        )
        runtime = self.main.AssistantRuntime(
            self.motor,
            FakeSensor([[valid_event()], []]),
            controller,
            ImmediateConfig,
            feedforward=FakeFeedforward(
                [(10.0, 15.0, 0.3), (10.0, 15.0, 0.3)]
            ),
            odometry=FakeOdometry((3.0, 4.0, 0.2)),
        )
        runtime.start()
        runtime.step(20)
        runtime.step(40)

        self.assertEqual(
            self.motor.operations[-1],
            ("command", 11.0, 17.0, 0.4),
        )

    def test_transient_ipm_projection_error_reuses_last_safe_command(self):
        class ImmediateConfig(RuntimeConfig):
            REACQUIRE_FRAMES = 1

        error = ValueError("IPM projection denominator is too small")
        self.motor = FakeMotor()
        controller = IntermittentInvalidController(
            [(1.0, 2.0, 0.1), error, error]
        )
        runtime = self.main.AssistantRuntime(
            self.motor,
            FakeSensor(
                [[valid_event()], [valid_event()], [valid_event()]]
            ),
            controller,
            ImmediateConfig,
        )
        runtime.start()

        runtime.step(0)
        runtime.step(100)
        self.assertEqual(runtime.state, self.main.TRACKING)
        self.assertEqual(runtime.last_event_type, "invalid")
        self.assertEqual(
            self.motor.operations[-1],
            ("command", 1.0, 2.0, 0.1),
        )

        runtime.step(151)
        self.assertEqual(runtime.state, self.main.LOST_STOP)
        self.assertEqual(self.motor.operations[-1], ("hard_stop",))

    def test_ipm_projection_error_falls_back_to_fresh_wireless(self):
        class ImmediateConfig(RuntimeConfig):
            REACQUIRE_FRAMES = 1

        error = ValueError("IPM projection denominator is too small")
        self.motor = FakeMotor()
        runtime = self.main.AssistantRuntime(
            self.motor,
            FakeSensor([[valid_event()]]),
            IntermittentInvalidController([error]),
            ImmediateConfig,
            feedforward=FakeFeedforward([(10.0, 15.0, 0.3)]),
        )
        runtime.start()
        runtime.step(0)

        self.assertEqual(runtime.state, self.main.FEEDFORWARD_ONLY)
        self.assertEqual(
            self.motor.operations[-1],
            ("command", 10.0, 15.0, 0.3),
        )

    def test_rotation_feedforward_includes_formation_offset_velocity(self):
        class OffsetConfig(RuntimeConfig):
            FORMATION_RIGHT_OFFSET_CM = -18.0
            FORMATION_FORWARD_OFFSET_CM = 0.0

        self.motor = FakeMotor()
        self.controller = FakeController(command=(0.0, 0.0, 0.0))
        self.sensor = FakeSensor([[]])
        feedforward = FakeFeedforward([(0.0, 0.0, 1.0)])
        runtime = self.main.AssistantRuntime(
            self.motor,
            self.sensor,
            self.controller,
            OffsetConfig,
            feedforward=feedforward,
        )
        runtime.start()
        runtime.step(20)

        self.assertEqual(runtime.state, self.main.FEEDFORWARD_ONLY)
        self.assertEqual(
            self.motor.operations[-1],
            ("command", 0.0, -18.0, 1.0),
        )

    def test_se2_visual_w_uses_dedicated_right_and_forward_offsets(self):
        class VisualOffsetConfig(RuntimeConfig):
            SE2_BODY_FRAME_ROTATION_ENABLED = True
            SE2_VISUAL_W_RIGHT_OFFSET_CM = 18.0
            SE2_VISUAL_W_FORWARD_OFFSET_CM = 4.0
            FORMATION_RIGHT_OFFSET_CM = -99.0
            FORMATION_FORWARD_OFFSET_CM = -88.0
            RIGID_SLOT_W_COMP_DEADBAND_RAD_S = 0.3

        runtime = self.main.AssistantRuntime(
            FakeMotor(),
            FakeSensor([]),
            FakeController(),
            VisualOffsetConfig,
        )

        self.assertEqual(
            runtime._combined_command((10.0, 20.0, 1.0), None),
            (6.0, 38.0, 1.0),
        )

    def test_se2_visual_w_offsets_are_disabled_with_se2(self):
        class DisabledVisualOffsetConfig(RuntimeConfig):
            SE2_BODY_FRAME_ROTATION_ENABLED = False
            SE2_VISUAL_W_RIGHT_OFFSET_CM = 18.0
            SE2_VISUAL_W_FORWARD_OFFSET_CM = 4.0

        runtime = self.main.AssistantRuntime(
            FakeMotor(),
            FakeSensor([]),
            FakeController(),
            DisabledVisualOffsetConfig,
        )

        self.assertEqual(
            runtime._combined_command((10.0, 20.0, 1.0), None),
            (10.0, 20.0, 1.0),
        )

    def test_se2_visual_w_offsets_reuse_rigid_slot_deadband(self):
        class VisualOffsetConfig(RuntimeConfig):
            SE2_BODY_FRAME_ROTATION_ENABLED = True
            SE2_VISUAL_W_RIGHT_OFFSET_CM = 18.0
            SE2_VISUAL_W_FORWARD_OFFSET_CM = 4.0
            RIGID_SLOT_W_COMP_DEADBAND_RAD_S = 0.3

        runtime = self.main.AssistantRuntime(
            FakeMotor(),
            FakeSensor([]),
            FakeController(),
            VisualOffsetConfig,
        )

        self.assertEqual(
            runtime._combined_command((10.0, 20.0, 0.29), None),
            (10.0, 20.0, 0.29),
        )

    def test_small_rotation_keeps_w_but_skips_formation_offset_velocity(self):
        class OffsetConfig(RuntimeConfig):
            FORMATION_RIGHT_OFFSET_CM = -20.0
            RIGID_SLOT_W_COMP_DEADBAND_RAD_S = 0.3

        self.motor = FakeMotor()
        runtime = self.main.AssistantRuntime(
            self.motor,
            FakeSensor([[]]),
            FakeController(command=(0.0, 0.0, 0.0)),
            OffsetConfig,
            feedforward=FakeFeedforward([(0.0, 0.0, -0.29)]),
        )
        runtime.start()
        runtime.step(20)

        self.assertEqual(
            self.motor.operations[-1],
            ("command", 0.0, 0.0, -0.29),
        )

    def test_stationary_feedforward_temporarily_suppresses_visual_xy(self):
        class StopPriorityConfig(RuntimeConfig):
            REACQUIRE_FRAMES = 1
            FEEDFORWARD_STATIONARY_VISUAL_XY_SUPPRESS_ENABLED = True
            FEEDFORWARD_STATIONARY_LINEAR_THRESHOLD_CM_S = 1.0
            FEEDFORWARD_STATIONARY_W_THRESHOLD_RAD_S = 0.01

        self.motor = FakeMotor()
        runtime = self.main.AssistantRuntime(
            self.motor,
            FakeSensor([[valid_event()], [valid_event()], [valid_event()]]),
            FakeController(command=(4.0, -5.0, 0.6)),
            StopPriorityConfig,
            feedforward=FakeFeedforward([
                (0.5, -0.9, 0.009),
                (0.5, -0.9, 0.009),
                (10.0, 12.0, 0.2),
            ]),
        )
        runtime.start()
        runtime.step(20)
        runtime.step(40)
        self.assertEqual(
            self.motor.operations[-1],
            ("command", 0.5, -0.9, 0.609),
        )

        runtime.step(60)
        self.assertEqual(
            self.motor.operations[-1],
            ("command", 14.0, 7.0, 0.8),
        )

    def test_follow_control_debug_prints_errors_and_visual_command(self):
        class DebugConfig(RuntimeConfig):
            REACQUIRE_FRAMES = 1
            FOLLOW_CONTROL_DEBUG_OUTPUT = True

        class DebugController(FakeController):
            def get_state(self):
                return {
                    "errors": {
                        "lateral": 1.25,
                        "front": -2.5,
                        "angle": 3.75,
                    }
                }

        self.motor = FakeMotor()
        output = io.StringIO()
        runtime = self.main.AssistantRuntime(
            self.motor,
            FakeSensor([[valid_event()]]),
            DebugController(command=(4.0, -5.0, 0.6)),
            DebugConfig,
        )
        runtime.start()
        with redirect_stdout(output):
            runtime.step(20)

        self.assertEqual(
            output.getvalue().strip(),
            "follow x=1.25 y=-2.50 theta=3.75 vx=4.00 vy=-5.00 w=0.600",
        )

    def test_visual_linear_scale_preserves_existing_follow_tuning(self):
        class VisualScaleConfig(RuntimeConfig):
            VISUAL_FOLLOW_LINEAR_COMMAND_SCALE = 0.58

        runtime = self.main.AssistantRuntime(
            FakeMotor(),
            FakeSensor([[]]),
            FakeController(),
            VisualScaleConfig,
        )
        runtime._set_raw_visual_command((10.0, -20.0, 0.3))

        self.assertEqual(runtime.raw_visual_command, (5.8, -11.6, 0.3))

    def test_se2_rotates_main_body_slot_velocity_into_follower_body(self):
        class Se2Config(RuntimeConfig):
            REACQUIRE_FRAMES = 1
            RELATIVE_HEADING_LIMIT_DEG = 90.0
            SE2_BODY_FRAME_HEADING_SIGN = 1.0
            SE2_BODY_FRAME_MAX_STEP_DEG = 180.0

        self.motor = FakeMotor()
        self.controller = HeadingController(
            0.0,
            raw_relative_heading_rad=math.pi * 0.5,
        )
        self.sensor = FakeSensor([[valid_event()], []])
        feedforward = FakeFeedforward(
            [(10.0, 0.0, 0.0), (10.0, 0.0, 0.0)]
        )
        runtime = self.main.AssistantRuntime(
            self.motor,
            self.sensor,
            self.controller,
            Se2Config,
            feedforward=feedforward,
        )
        runtime.start()

        runtime.step(20)
        runtime.step(40)

        operation = self.motor.operations[-1]
        self.assertEqual(operation[0], "command")
        self.assertAlmostEqual(operation[1], 0.0, places=6)
        self.assertAlmostEqual(operation[2], 10.0, places=6)
        self.assertAlmostEqual(operation[3], 0.0, places=6)

    def test_se2_heading_sign_is_independent_from_yaw_control_heading(self):
        class Se2Config(RuntimeConfig):
            REACQUIRE_FRAMES = 1
            RELATIVE_HEADING_LIMIT_DEG = 90.0
            SE2_BODY_FRAME_HEADING_SIGN = -1.0
            SE2_BODY_FRAME_MAX_STEP_DEG = 180.0

        self.motor = FakeMotor()
        self.controller = HeadingController(
            math.pi * 0.5,
            raw_relative_heading_rad=math.pi * 0.5,
        )
        self.sensor = FakeSensor([[valid_event()], []])
        feedforward = FakeFeedforward(
            [(10.0, 0.0, 0.0), (10.0, 0.0, 0.0)]
        )
        runtime = self.main.AssistantRuntime(
            self.motor,
            self.sensor,
            self.controller,
            Se2Config,
            feedforward=feedforward,
        )
        runtime.start()

        runtime.step(20)
        runtime.step(40)

        operation = self.motor.operations[-1]
        self.assertAlmostEqual(operation[1], 0.0, places=6)
        self.assertAlmostEqual(operation[2], -10.0, places=6)

    def test_se2_heading_step_is_limited_without_changing_yaw_control_heading(self):
        class Se2Config(RuntimeConfig):
            REACQUIRE_FRAMES = 1
            RELATIVE_HEADING_LIMIT_DEG = 90.0
            SE2_BODY_FRAME_HEADING_SIGN = 1.0
            SE2_BODY_FRAME_MAX_STEP_DEG = 5.0

        self.motor = FakeMotor()
        controller = HeadingController(
            math.radians(30.0),
            raw_relative_heading_rad=math.radians(30.0),
        )
        runtime = self.main.AssistantRuntime(
            self.motor,
            FakeSensor([[valid_event()]]),
            controller,
            Se2Config,
            feedforward=FakeFeedforward([(0.0, 10.0, 0.0)]),
        )
        runtime.start()

        runtime.step(20)

        self.assertAlmostEqual(runtime.relative_heading_rad, math.radians(30.0))
        self.assertAlmostEqual(runtime.se2_body_heading_rad, math.radians(5.0))

    def test_visual_loss_clears_relative_heading_before_feedforward_only(self):
        class Se2Config(RuntimeConfig):
            REACQUIRE_FRAMES = 1
            RELATIVE_HEADING_LIMIT_DEG = 90.0

        self.motor = FakeMotor()
        self.controller = HeadingController(math.pi * 0.5)
        self.sensor = FakeSensor(
            [
                [valid_event()],
                [{"type": "lost"}],
            ]
        )
        feedforward = FakeFeedforward(
            [
                (10.0, 0.0, 0.0),
                (10.0, 0.0, 0.0),
            ]
        )
        runtime = self.main.AssistantRuntime(
            self.motor,
            self.sensor,
            self.controller,
            Se2Config,
            feedforward=feedforward,
        )
        runtime.start()
        runtime.step(20)

        runtime.step(40)

        self.assertEqual(runtime.state, self.main.FEEDFORWARD_ONLY)
        self.assertEqual(
            self.motor.operations[-1],
            ("command", 10.0, 0.0, 0.0),
        )


if __name__ == "__main__":
    unittest.main()

import importlib.util
import pathlib
import unittest


ASSISTANT_DIR = pathlib.Path(__file__).resolve().parents[1]
FOLLOW_PATH = ASSISTANT_DIR / "follow.py"
CONFIG_PATH = ASSISTANT_DIR / "config.py"
CAMERA_CONFIG_PATH = ASSISTANT_DIR.parent / "跟随摄像头" / "config.py"


def load_follow_module():
    spec = importlib.util.spec_from_file_location("assistant_follow_sensor", FOLLOW_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_config_module():
    spec = importlib.util.spec_from_file_location(
        "assistant_follow_config",
        CONFIG_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_config_module_from_path(module_name, path):
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class SensorConfig:
    TARGET_X_MIN_CM = -25.0
    TARGET_X_MAX_CM = 25.0
    TARGET_Y_MIN_CM = -30.0
    TARGET_Y_MAX_CM = 25.0
    THETA_MIN_DEG = -90.0
    THETA_MAX_DEG = 90.0
    IMAGE_WIDTH = 320
    IMAGE_HEIGHT = 240
    MID_Y_MIN = 0.0
    MID_Y_MAX = 239.0
    LINE_ANGLE_MIN_DEG = -90.0
    LINE_ANGLE_MAX_DEG = 90.0
    DISTANCE_MIN_PX = 20.0
    DISTANCE_MAX_PX = 160.0
    MIN_QUALITY = 16
    MAX_QUALITY = 999
    CAMERA_TIMEOUT_MS = 150
    UART_BUFFER_MAX_BYTES = 128
    UART_READ_MAX_BYTES = 256
    REF_DISTANCE_PX = 71.9


class FakeUART:
    def __init__(self, chunks=()):
        self.chunks = list(chunks)
        self.read_lengths = []

    def any(self):
        return len(self.chunks[0]) if self.chunks else 0

    def read(self, length=None):
        self.read_lengths.append(length)
        if not self.chunks:
            return None
        chunk = self.chunks.pop(0)
        if length is None or len(chunk) <= length:
            return chunk
        self.chunks.insert(0, chunk[length:])
        return chunk[:length]

    def push(self, data):
        self.chunks.append(data)


class FollowSensorTests(unittest.TestCase):
    def setUp(self):
        self.follow = load_follow_module()

    def test_partial_line_is_buffered_until_newline(self):
        uart = FakeUART([b"0.868,-6.065,0", b".0\n"])
        sensor = self.follow.FollowSensor(uart, SensorConfig)

        self.assertEqual(sensor.poll(0), [])
        self.assertEqual(uart.read_lengths, [len(b"0.868,-6.065,0")])
        events = sensor.poll(10)

        self.assertEqual([event["type"] for event in events], ["measurement"])
        self.assertAlmostEqual(events[0]["measurement"]["theta_deg"], 0.0)
        self.assertEqual(
            uart.read_lengths,
            [len(b"0.868,-6.065,0"), len(b".0\n")],
        )

    def test_physical_pose_triple_is_parsed_without_pixel_conversion(self):
        event = self.follow.parse_camera_line(
            b"0.868,-6.065,0.00\n",
            SensorConfig,
        )

        self.assertEqual(event["type"], "measurement")
        self.assertEqual(
            event["measurement"],
            {
                "found": True,
                "target_x_cm": 0.868,
                "target_y_cm": -6.065,
                "theta_deg": 0.0,
                "coordinate_space": "plane",
            },
        )

    def test_physical_pose_triple_rejects_out_of_range_values(self):
        for line in (
            b"26.0,0.0,0.0\n",
            b"0.0,-31.0,0.0\n",
            b"0.0,0.0,91.0\n",
        ):
            self.assertEqual(
                self.follow.parse_camera_line(line, SensorConfig)["type"],
                "invalid",
            )

    def test_legacy_pixel_frame_is_rejected(self):
        event = self.follow.parse_camera_line(
            b"1,160,120,45.0\n",
            SensorConfig,
        )

        self.assertEqual(event["type"], "invalid")
        self.assertEqual(event["reason"], "field_count")

    def test_multiple_lines_keep_only_latest_measurement_or_lost(self):
        uart = FakeUART(
            [b"0.868,-6.065,0.0\r\n0\n"]
        )
        sensor = self.follow.FollowSensor(uart, SensorConfig)

        events = sensor.poll(25)

        self.assertEqual([event["type"] for event in events], ["lost"])

    def test_stale_lost_is_preserved_before_latest_measurement(self):
        uart = FakeUART(
            [b"0\n0.868,-6.065,0.0\n"]
        )
        sensor = self.follow.FollowSensor(uart, SensorConfig)

        events = sensor.poll(25)

        self.assertEqual(
            [event["type"] for event in events],
            ["lost", "measurement"],
        )
        self.assertIsNotNone(sensor.latest_measurement)

    def test_malformed_nonfinite_and_out_of_range_frames_are_invalid(self):
        uart = FakeUART(
            [
                b"1,160,120,45,80\n"
                b"nan,-6.0,0.0\n"
                b"26.0,-6.0,0.0\n"
            ]
        )
        sensor = self.follow.FollowSensor(uart, SensorConfig)

        events = sensor.poll(0)

        self.assertEqual([event["type"] for event in events], ["invalid"])

    def test_invalid_frame_preserves_measurement_until_timeout(self):
        uart = FakeUART(
            [
                b"1,160,120,45,80,60\n",
                b"0.868,-6.065,0.0\n",
                b"broken\n",
            ]
        )
        sensor = self.follow.FollowSensor(uart, SensorConfig)

        self.assertEqual(sensor.poll(0)[0]["type"], "invalid")
        sensor.poll(10)
        previous = sensor.latest_measurement
        event = sensor.poll(30)[0]

        self.assertEqual(event["type"], "invalid")
        self.assertIs(sensor.latest_measurement, previous)
        self.assertEqual(sensor.poll(160), [])
        self.assertEqual(sensor.poll(161)[0]["type"], "timeout")
        self.assertIsNone(sensor.latest_measurement)

    def test_production_config_accepts_camera_boundary_frames(self):
        config = load_config_module()
        for line in (
            b"-25,-30,-90\n",
            b"25,25,90\n",
        ):
            event = self.follow.parse_camera_line(line, config)
            self.assertEqual(event["type"], "measurement")

    def test_production_camera_and_assistant_plane_references_match(self):
        camera_config = load_config_module_from_path(
            "follow_camera_config",
            CAMERA_CONFIG_PATH,
        )
        assistant_config = load_config_module()
        for name in (
            "REF_TARGET_X_CM",
            "REF_TARGET_Y_CM",
        ):
            self.assertEqual(
                getattr(assistant_config, name),
                getattr(camera_config, name),
            )

    def test_explicit_lost_frame_never_reuses_old_measurement(self):
        uart = FakeUART(
            [
                b"0.868,-6.065,0.0\n",
                b"0\n",
            ]
        )
        sensor = self.follow.FollowSensor(uart, SensorConfig)

        self.assertEqual(sensor.poll(0)[0]["type"], "measurement")
        event = sensor.poll(30)[0]

        self.assertEqual(event["type"], "lost")
        self.assertIsNone(sensor.latest_measurement)

    def test_timeout_is_reported_once_until_new_valid_measurement(self):
        uart = FakeUART([b"0.868,-6.065,0.0\n"])
        sensor = self.follow.FollowSensor(uart, SensorConfig)

        sensor.poll(0)
        self.assertEqual(sensor.poll(150), [])
        self.assertEqual(sensor.poll(151)[0]["type"], "timeout")
        self.assertEqual(sensor.poll(200), [])

        uart.push(b"0.900,-6.000,0.2\n")
        self.assertEqual(sensor.poll(210)[0]["type"], "measurement")
        self.assertEqual(sensor.poll(361)[0]["type"], "timeout")

    def test_buffer_overflow_is_invalid_and_clears_partial_data(self):
        uart = FakeUART([b"1" * 129])
        sensor = self.follow.FollowSensor(uart, SensorConfig)

        events = sensor.poll(0)

        self.assertEqual(events[0]["type"], "invalid")
        self.assertEqual(sensor.buffer, b"")


if __name__ == "__main__":
    unittest.main()

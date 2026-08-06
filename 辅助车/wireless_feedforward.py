"""主车到辅助车的车体速度无线前馈。

本文件与主车目录中的同名文件保持协议一致，便于辅助车单独烧录。
协议沿用 car141929 的 9-float 扩展帧；速度单位为 cm/s 和 rad/s。
"""

import struct
import time
from array import array


WIRELESS_BAUD = 230400
FEEDFORWARD_STATE = 31
FEEDFORWARD_TIMEOUT_MS = 300
FRAME_HEADER = b"\x5A\xA5"
FRAME_TAIL = b"\x0D\x0A"
FLOAT_COUNT = 9
FRAME_SIZE = 2 + FLOAT_COUNT * 4 + 2
MAX_RX_BUFFER_BYTES = FRAME_SIZE * 3
MAX_RX_READS_PER_POLL = 4


def _ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.monotonic() * 1000.0)


def _ticks_diff(new_value, old_value):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(new_value, old_value)
    return new_value - old_value


def _finite(value):
    value = float(value)
    return value == value and -1.0e30 < value < 1.0e30


def _create_wireless(baud):
    from seekfree import WIRELESS_UART

    return WIRELESS_UART(int(baud))


def encode_feedforward(vx, vy, w, sequence=0):
    values = (float(vx), float(vy), float(w))
    if not all(_finite(value) for value in values):
        raise ValueError("feedforward command must be finite")
    data = struct.pack(
        "<fffffffff",
        0.0,
        0.0,
        float(FEEDFORWARD_STATE),
        0.0,
        0.0,
        float(int(sequence) % 1000000),
        values[0],
        values[1],
        values[2],
    )
    return FRAME_HEADER + data + FRAME_TAIL


def decode_feedforward(frame):
    if isinstance(frame, str):
        frame = frame.encode("latin1")
    frame = bytes(frame)
    if len(frame) != FRAME_SIZE:
        return None
    if frame[:2] != FRAME_HEADER or frame[-2:] != FRAME_TAIL:
        return None
    try:
        values = struct.unpack("<fffffffff", frame[2:-2])
    except Exception:
        return None
    if int(values[2]) != FEEDFORWARD_STATE:
        return None
    vx, vy, w = values[6], values[7], values[8]
    if not all(_finite(value) for value in (vx, vy, w)):
        return None
    return {
        "vx": vx,
        "vy": vy,
        "w": w,
        "sequence": int(values[5]),
    }


def _write_signed(payload, target):
    for index, value in enumerate(payload):
        target[index] = value if value < 128 else value - 256


class FeedforwardSender:
    def __init__(self, wireless=None, baud=WIRELESS_BAUD):
        self.wireless = wireless or _create_wireless(baud)
        self.send_buffer = array("b", [0] * FRAME_SIZE)
        self.sequence = 0
        self.last_command = (0.0, 0.0, 0.0)

    def send(self, vx, vy, w):
        payload = encode_feedforward(vx, vy, w, self.sequence)
        _write_signed(payload, self.send_buffer)
        self.wireless.send_bytearray(self.send_buffer, FRAME_SIZE)
        self.last_command = (float(vx), float(vy), float(w))
        self.sequence = (self.sequence + 1) % 1000000

    def send_motor_command(self, motor):
        command = motor.get_limited_command()
        self.send(command[0], command[1], command[2])
        return command


class FeedforwardReceiver:
    def __init__(
        self,
        wireless=None,
        baud=WIRELESS_BAUD,
        timeout_ms=FEEDFORWARD_TIMEOUT_MS,
    ):
        self.wireless = wireless or _create_wireless(baud)
        self.timeout_ms = int(timeout_ms)
        self.receive_buffer = array("b", [0] * 64)
        self.stream_buffer = b""
        self.last_command = None
        self.last_update_ms = None
        self.last_sequence = None
        self.last_error = None

    def reset(self):
        self.stream_buffer = b""
        self.last_command = None
        self.last_update_ms = None
        self.last_sequence = None
        self.last_error = None

    def _read(self):
        length = self.wireless.receive_bytearray(
            self.receive_buffer,
            len(self.receive_buffer),
        )
        if not length:
            return b""
        data = bytearray(length)
        for index in range(length):
            value = self.receive_buffer[index]
            data[index] = value if value >= 0 else value + 256
        return bytes(data)

    def _consume(self, now_ms):
        while True:
            start = self.stream_buffer.find(FRAME_HEADER)
            if start < 0:
                self.stream_buffer = self.stream_buffer[-1:]
                break
            if start:
                self.stream_buffer = self.stream_buffer[start:]
            if len(self.stream_buffer) < FRAME_SIZE:
                break
            decoded = decode_feedforward(self.stream_buffer[:FRAME_SIZE])
            if decoded is None:
                self.stream_buffer = self.stream_buffer[1:]
                continue
            self.stream_buffer = self.stream_buffer[FRAME_SIZE:]
            self.last_command = (
                decoded["vx"],
                decoded["vy"],
                decoded["w"],
            )
            self.last_update_ms = now_ms
            self.last_sequence = decoded["sequence"]

    def poll(self, now_ms=None):
        if now_ms is None:
            now_ms = _ticks_ms()
        try:
            # 尽量排空硬件缓存，只把本轮最后一个完整帧留给控制器。
            for _ in range(MAX_RX_READS_PER_POLL):
                incoming = self._read()
                if not incoming:
                    break
                self.stream_buffer += incoming
                if len(self.stream_buffer) > MAX_RX_BUFFER_BYTES:
                    self.stream_buffer = self.stream_buffer[
                        -MAX_RX_BUFFER_BYTES:
                    ]
                self._consume(now_ms)
            self.last_error = None
        except Exception as error:
            self.last_error = repr(error)
        return self.get_command(now_ms)

    def get_command(self, now_ms=None):
        if now_ms is None:
            now_ms = _ticks_ms()
        if self.last_command is None or self.last_update_ms is None:
            return None
        if _ticks_diff(now_ms, self.last_update_ms) > self.timeout_ms:
            return None
        return self.last_command


def _clamp(value, limit):
    limit = max(0.0, float(limit))
    return max(-limit, min(float(value), limit))


def combine_commands(
    visual_command,
    feedforward_command,
    max_vx,
    max_vy,
    max_w,
):
    visual = visual_command or (0.0, 0.0, 0.0)
    feedforward = feedforward_command or (0.0, 0.0, 0.0)
    return (
        _clamp(visual[0] + feedforward[0], max_vx),
        _clamp(visual[1] + feedforward[1], max_vy),
        _clamp(visual[2] + feedforward[2], max_w),
    )

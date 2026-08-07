"""主车到辅助车的车体速度无线前馈。

协议沿用 car141929 的 0x5A 0xA5 帧头、float32 小端数据和
0x0D 0x0A 帧尾，并使用旧双车方案已经验证过的 9-float 扩展帧。
vx、vy 的单位为 cm/s，w 的单位为 rad/s。

正式主流程融合电机 S 曲线目标指令与实测速度：实测 vx、vy 来自编码器
里程计，实测 w 来自 IMU。保留纯目标和纯实测接口，供独立调试使用。
"""

import struct
import time
from array import array


WIRELESS_BAUD = 230400
FEEDFORWARD_STATE = 31
FEEDFORWARD_TIMEOUT_MS = 300
DEFAULT_TX_PERIOD_MS = 10

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


def _sleep_ms(milliseconds):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(int(milliseconds))
    else:
        time.sleep(float(milliseconds) / 1000.0)


def _finite(value):
    value = float(value)
    return value == value and -1.0e30 < value < 1.0e30


def _create_wireless(baud):
    from seekfree import WIRELESS_UART

    return WIRELESS_UART(int(baud))


def encode_feedforward(vx, vy, w, sequence=0):
    """编码兼容旧 9-float 双车帧的专用前馈帧。"""
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
    """解码一帧；不是专用前馈帧或内容损坏时返回 None。"""
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
    """主车发送端。"""

    def __init__(
        self,
        wireless=None,
        baud=WIRELESS_BAUD,
        period_ms=DEFAULT_TX_PERIOD_MS,
    ):
        self.wireless = wireless or _create_wireless(baud)
        self.period_ms = max(1, int(period_ms))
        self.send_buffer = array("b", [0] * FRAME_SIZE)
        self.sequence = 0
        self.last_command = (0.0, 0.0, 0.0)
        self.last_tx_ms = None

    def send(self, vx, vy, w):
        payload = encode_feedforward(vx, vy, w, self.sequence)
        _write_signed(payload, self.send_buffer)
        self.wireless.send_bytearray(self.send_buffer, FRAME_SIZE)
        self.last_command = (float(vx), float(vy), float(w))
        self.sequence = (self.sequence + 1) % 1000000
        self.last_tx_ms = _ticks_ms()
        return self.last_command

    def send_motor_command(self, motor, straight_without_w=False):
        """发送电机经过 S 曲线和底盘限幅后的完整车体速度。"""
        if hasattr(motor, "get_limited_physical_command"):
            command = motor.get_limited_physical_command()
        else:
            # 保持桌面假电机和旧电机接口兼容。
            command = motor.get_limited_command()
        
        if straight_without_w:
            command = (command[0], command[1], 0.0)
            
        self.send(command[0], command[1], command[2])
        return command

    def send_motor_command_if_due(self, motor, now_ms=None, straight_without_w=False):
        """达到发送周期时发送一次，否则保持当前无线状态。"""
        if now_ms is None:
            now_ms = _ticks_ms()
        if (
            self.last_tx_ms is not None
            and _ticks_diff(now_ms, self.last_tx_ms) < self.period_ms
        ):
            return None
        return self.send_motor_command(motor, straight_without_w=straight_without_w)

    def send_measured_motion(self, odometry, straight_without_w=False):
        """发送编码器实测平移速度和 IMU 实测角速度。"""
        state = odometry.get_state()
        command = (
            float(state["body_vx_cm_s"]),
            float(state["body_vy_cm_s"]),
            float(state["yaw_rate_rad_s"]),
        )
        if straight_without_w:
            command = (command[0], command[1], 0.0)
        self.send(command[0], command[1], command[2])
        return command

    def send_measured_motion_if_due(
        self,
        odometry,
        now_ms=None,
        straight_without_w=False,
    ):
        """达到发送周期时发送一帧实测车体速度。"""
        if now_ms is None:
            now_ms = _ticks_ms()
        if (
            self.last_tx_ms is not None
            and _ticks_diff(now_ms, self.last_tx_ms) < self.period_ms
        ):
            return None
        return self.send_measured_motion(
            odometry,
            straight_without_w=straight_without_w,
        )

    def send_blended_motion(
        self,
        motor,
        odometry,
        measured_weight,
        straight_without_w=False,
    ):
        """融合 S 曲线目标指令和编码器/IMU 实测速度后发送。"""
        measured_weight = float(measured_weight)
        if not 0.0 <= measured_weight <= 1.0:
            raise ValueError("measured feedforward weight must be within 0..1")
        target_weight = 1.0 - measured_weight
        if hasattr(motor, "get_limited_physical_command"):
            target = motor.get_limited_physical_command()
        else:
            target = motor.get_limited_command()
        state = odometry.get_state()
        measured = (
            float(state["body_vx_cm_s"]),
            float(state["body_vy_cm_s"]),
            float(state["yaw_rate_rad_s"]),
        )
        command = tuple(
            target_weight * float(target[index])
            + measured_weight * measured[index]
            for index in range(3)
        )
        if straight_without_w:
            command = (command[0], command[1], 0.0)
        self.send(command[0], command[1], command[2])
        return command

    def send_blended_motion_if_due(
        self,
        motor,
        odometry,
        measured_weight,
        now_ms=None,
        straight_without_w=False,
    ):
        """达到发送周期时发送一帧目标与实测的融合速度。"""
        if now_ms is None:
            now_ms = _ticks_ms()
        if (
            self.last_tx_ms is not None
            and _ticks_diff(now_ms, self.last_tx_ms) < self.period_ms
        ):
            return None
        return self.send_blended_motion(
            motor,
            odometry,
            measured_weight,
            straight_without_w=straight_without_w,
        )

    def send_zero_frames(self, count=5):
        """连续发送安全零速度帧，用于启动和退出交接。"""
        sent = 0
        for _ in range(max(0, int(count))):
            try:
                self.send(0.0, 0.0, 0.0)
            except Exception:
                break
            sent += 1
            _sleep_ms(self.period_ms)
        return sent

    def hold_zero_for(self, duration_ms):
        """在指定启动保持时间内按发送周期持续发送零速度。"""
        duration_ms = max(0, int(duration_ms))
        start_ms = _ticks_ms()
        self.last_tx_ms = None
        while _ticks_diff(_ticks_ms(), start_ms) < duration_ms:
            now_ms = _ticks_ms()
            if (
                self.last_tx_ms is None
                or _ticks_diff(now_ms, self.last_tx_ms) >= self.period_ms
            ):
                self.send(0.0, 0.0, 0.0)
            _sleep_ms(1)


class FeedforwardReceiver:
    """辅助车接收端，支持粘包、拆包、噪声字节和新鲜度超时。"""

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
            # 一轮内尽量排空硬件接收缓存，并在 _consume 中覆盖为序号最新
            # 的完整帧，避免控制器持续执行排队的旧速度。
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
    """将主车前馈叠加到辅助车视觉修正，并复用现有速度上限。"""
    visual = visual_command or (0.0, 0.0, 0.0)
    feedforward = feedforward_command or (0.0, 0.0, 0.0)
    return (
        _clamp(visual[0] + feedforward[0], max_vx),
        _clamp(visual[1] + feedforward[1], max_vy),
        _clamp(visual[2] + feedforward[2], max_w),
    )

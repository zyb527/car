"""用于测量双车指令同步性的无线层钩子。

未来的无线接收端应针对每一帧有效指令调用一次 record_received_frame()。
两辆车分别保存自己的 JSONL 日志；运行结束后将两个文件复制到电脑比较。
"""

import math

from calibration_store import CalibrationLog, DEFAULT_LOG_PATH, ticks_ms
from common import mean, ticks_diff


class DualCarSyncRecorder:
    def __init__(self, role, path=DEFAULT_LOG_PATH):
        self.role = role
        self.log = CalibrationLog("dual_car_sync_" + role, path)
        self.previous_sequence = None
        self.previous_receive_ms = None
        self.frame_intervals_ms = []
        self.command_errors = []
        self.dropped_frames = 0
        self.frames = 0

    def record_received_frame(
        self,
        sequence,
        peer_command,
        motor,
        peer_mode=None,
        receive_time_ms=None,
    ):
        now = ticks_ms() if receive_time_ms is None else receive_time_ms
        local_command = motor.get_limited_command()
        error = (
            local_command[0] - peer_command[0],
            local_command[1] - peer_command[1],
            local_command[2] - peer_command[2],
        )
        error_norm = math.sqrt(
            error[0] * error[0]
            + error[1] * error[1]
            + error[2] * error[2]
        )
        interval_ms = None
        if self.previous_receive_ms is not None:
            interval_ms = ticks_diff(now, self.previous_receive_ms)
            self.frame_intervals_ms.append(interval_ms)
        if self.previous_sequence is not None:
            gap = int(sequence) - int(self.previous_sequence)
            if gap > 1:
                self.dropped_frames += gap - 1

        self.frames += 1
        self.command_errors.append(error_norm)
        self.previous_sequence = sequence
        self.previous_receive_ms = now
        self.log.write(
            "sync_frame",
            role=self.role,
            sequence=sequence,
            receive_time_ms=now,
            receive_interval_ms=interval_ms,
            peer_mode=peer_mode,
            peer_command=peer_command,
            local_limited_command=local_command,
            command_error=error,
            command_error_norm=error_norm,
            local_wheel_speeds=motor.get_wheel_speeds(),
        )

    def close(self, status="complete", error=None):
        max_interval = (
            max(self.frame_intervals_ms) if self.frame_intervals_ms else None
        )
        self.log.write(
            "sync_summary",
            role=self.role,
            received_frames=self.frames,
            dropped_frames=self.dropped_frames,
            mean_receive_interval_ms=mean(self.frame_intervals_ms),
            max_receive_interval_ms=max_interval,
            mean_command_error=mean(self.command_errors),
            max_command_error=(
                max(self.command_errors) if self.command_errors else None
            ),
        )
        self.log.close(status, error)

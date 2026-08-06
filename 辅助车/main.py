"""辅助车无线前馈加视觉修正的并排跟随入口。

板端运行时执行 main()。AssistantRuntime 本身不导入任何硬件模块，
可以用假串口、假电机和假控制器在桌面测试。
"""

import math
import time


WAIT_TARGET = "WAIT_TARGET"
TRACKING = "TRACKING"
FEEDFORWARD_ONLY = "FEEDFORWARD_ONLY"
LOST_STOP = "LOST_STOP"

_UNSAFE_EVENT_TYPES = ("lost", "timeout")


def _ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.monotonic() * 1000.0)


def _ticks_diff(new_value, old_value):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(new_value, old_value)
    return new_value - old_value


def _sleep_ms(milliseconds):
    milliseconds = max(0, int(milliseconds))
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(milliseconds)
    else:
        time.sleep(milliseconds / 1000.0)


def _slot_compensation_w(w, config):
    """只为 w×编队偏置项去除小角速度，不改变角速度命令本身。"""
    w = float(w)
    deadband = max(
        0.0,
        float(getattr(config, "RIGID_SLOT_W_COMP_DEADBAND_RAD_S", 0.0)),
    )
    return 0.0 if abs(w) < deadband else w


def _log_value(value):
    if value is None:
        return ""
    try:
        value = float(value)
    except (TypeError, ValueError):
        return ""
    if value != value or not -1.0e30 < value < 1.0e30:
        return ""
    return "{:.4f}".format(value)


def _log_command(command):
    if command is None or len(command) != 3:
        return ("", "", "")
    return (
        _log_value(command[0]),
        _log_value(command[1]),
        _log_value(command[2]),
    )


class OfflineLogger:
    """低频采样、批量刷盘的板端跟随诊断日志。"""

    HEADER = (
        "time_ms,state,step_elapsed_ms,ff_poll_ms,observer_ms,"
        "sensor_poll_ms,controller_ms,command_submit_ms,"
        "ff_fresh,ff_age_ms,ff_sequence,ff_vx,ff_vy,ff_w,"
        "visual_vx,visual_vy,visual_w,heading_rad,"
        "final_vx,final_vy,final_w\n"
    )

    def __init__(self, config, stream=None):
        self.enabled = bool(
            getattr(config, "OFFLINE_LOG_ENABLED", False)
        )
        self.path = str(
            getattr(
                config,
                "OFFLINE_LOG_PATH",
                "assistant_follow_log.txt",
            )
        )
        self.period_ms = max(
            10,
            int(getattr(config, "OFFLINE_LOG_PERIOD_MS", 200)),
        )
        self.flush_ms = max(
            self.period_ms,
            int(getattr(config, "OFFLINE_LOG_FLUSH_MS", 1000)),
        )
        self.max_bytes = max(
            1024,
            int(getattr(config, "OFFLINE_LOG_MAX_BYTES", 1024 * 1024)),
        )
        self.stream = None
        self.owns_stream = stream is None
        self.pending = []
        self.bytes_written = 0
        self.last_sample_ms = None
        self.last_flush_ms = None
        self.error = None

        if not self.enabled:
            return
        try:
            if stream is None:
                try:
                    import os

                    self.bytes_written = int(os.stat(self.path)[6])
                except Exception:
                    self.bytes_written = 0
                if self.bytes_written >= self.max_bytes:
                    self.enabled = False
                    self.error = "log_full"
                    return
                self.stream = open(self.path, "a")
            else:
                self.stream = stream

            if self.bytes_written == 0:
                self.stream.write(self.HEADER)
                self.bytes_written += len(self.HEADER)
            else:
                marker = "# new_session_ms={}\n".format(_ticks_ms())
                self.stream.write(marker + self.HEADER)
                self.bytes_written += len(marker) + len(self.HEADER)
            if hasattr(self.stream, "flush"):
                self.stream.flush()
        except Exception as error:
            self.error = repr(error)
            self.enabled = False
            self._close_stream()

    def _close_stream(self):
        if (
            self.stream is not None
            and self.owns_stream
            and hasattr(self.stream, "close")
        ):
            try:
                self.stream.close()
            except Exception:
                pass
        self.stream = None

    def _feedforward_diagnostics(self, runtime, now_ms):
        receiver = runtime.feedforward
        if receiver is None:
            return "", "", ""
        last_update_ms = getattr(receiver, "last_update_ms", None)
        age_ms = (
            ""
            if last_update_ms is None
            else str(_ticks_diff(now_ms, last_update_ms))
        )
        sequence = getattr(receiver, "last_sequence", None)
        return (
            "1" if runtime.last_feedforward_command is not None else "0",
            age_ms,
            "" if sequence is None else str(sequence),
        )

    def sample(self, now_ms, runtime, step_elapsed_ms):
        if not self.enabled or self.stream is None:
            return False
        if (
            self.last_sample_ms is not None
            and _ticks_diff(now_ms, self.last_sample_ms) < self.period_ms
        ):
            return False

        ff_fresh, ff_age_ms, ff_sequence = (
            self._feedforward_diagnostics(runtime, now_ms)
        )
        ff = _log_command(runtime.last_feedforward_command)
        visual = _log_command(runtime.last_visual_command)
        final = _log_command(runtime.last_command)
        fields = (
            str(now_ms),
            str(runtime.state),
            str(step_elapsed_ms),
            str(runtime.last_ff_poll_ms),
            str(runtime.last_observer_ms),
            str(runtime.last_sensor_poll_ms),
            str(runtime.last_controller_ms),
            str(runtime.last_command_submit_ms),
            ff_fresh,
            ff_age_ms,
            ff_sequence,
            ff[0],
            ff[1],
            ff[2],
            visual[0],
            visual[1],
            visual[2],
            _log_value(runtime.se2_body_heading_rad),
            final[0],
            final[1],
            final[2],
        )
        line = ",".join(fields) + "\n"
        pending_bytes = sum(len(item) for item in self.pending)
        if (
            self.bytes_written + pending_bytes + len(line)
            > self.max_bytes
        ):
            self.flush(now_ms)
            self.enabled = False
            self.error = "log_full"
            self._close_stream()
            return False

        self.pending.append(line)
        self.last_sample_ms = now_ms
        if (
            self.last_flush_ms is None
            or _ticks_diff(now_ms, self.last_flush_ms) >= self.flush_ms
        ):
            self.flush(now_ms)
        return True

    def flush(self, now_ms=None):
        if self.stream is None or not self.pending:
            return
        try:
            payload = "".join(self.pending)
            self.stream.write(payload)
            if hasattr(self.stream, "flush"):
                self.stream.flush()
            self.bytes_written += len(payload)
            self.pending = []
            self.last_flush_ms = (
                _ticks_ms() if now_ms is None else now_ms
            )
        except Exception as error:
            self.error = repr(error)
            self.enabled = False
            self.pending = []
            self._close_stream()

    def close(self):
        self.flush()
        self._close_stream()
        self.enabled = False


class AssistantRuntime:
    """每轮最多向底盘提交一次操作的跟随运行时。"""

    def __init__(
        self,
        motor,
        sensor,
        controller,
        config,
        feedforward=None,
        odometry=None,
    ):
        self.motor = motor
        self.sensor = sensor
        self.controller = controller
        self.config = config
        self.feedforward = feedforward
        self.odometry = odometry
        self.state = WAIT_TARGET
        self.acquire_count = 0
        self.last_visual_command = None
        self.raw_visual_command = None
        self.filtered_feedforward_visual_command = None
        self.feedforward_visual_blend = 0.0
        self.feedforward_visual_mode_active = False
        self.visual_loss_decay_command = None
        self.visual_loss_decay_started_ms = None
        self.last_command = None
        self.last_event_type = None
        self.last_event_detail = None
        self.last_error = None
        self.relative_heading_rad = 0.0
        self.se2_body_heading_rad = 0.0
        self.last_visual_rigid_command = None
        self.last_control_measurement_ms = None
        self.last_feedforward_command = None
        self.last_ff_poll_ms = 0
        self.last_observer_ms = 0
        self.last_sensor_poll_ms = 0
        self.last_controller_ms = 0
        self.last_command_submit_ms = 0
        self._step_start_ticks_ms = None

    def start(self):
        self.state = WAIT_TARGET
        self.acquire_count = 0
        self.last_visual_command = None
        self.raw_visual_command = None
        self.filtered_feedforward_visual_command = None
        self.feedforward_visual_blend = 0.0
        self.feedforward_visual_mode_active = False
        self.visual_loss_decay_command = None
        self.visual_loss_decay_started_ms = None
        self.last_command = None
        self.last_event_type = None
        self.last_event_detail = None
        self.last_error = None
        self.relative_heading_rad = 0.0
        self.se2_body_heading_rad = 0.0
        self.last_visual_rigid_command = None
        self.last_control_measurement_ms = None
        self.last_feedforward_command = None
        self.last_ff_poll_ms = 0
        self.last_observer_ms = 0
        self.last_sensor_poll_ms = 0
        self.last_controller_ms = 0
        self.last_command_submit_ms = 0
        self._step_start_ticks_ms = None
        self.controller.reset()
        if self.feedforward is not None:
            self.feedforward.reset()
        self.motor.hard_stop()

    def _enter_lost_stop(self, reason):
        should_submit_stop = self.state != LOST_STOP or self.last_command is not None
        self.state = LOST_STOP
        self.acquire_count = 0
        self.last_visual_command = None
        self.raw_visual_command = None
        self.filtered_feedforward_visual_command = None
        self.feedforward_visual_blend = 0.0
        self.feedforward_visual_mode_active = False
        self.visual_loss_decay_command = None
        self.visual_loss_decay_started_ms = None
        self.last_command = None
        self.last_event_type = reason
        self.relative_heading_rad = 0.0
        self.se2_body_heading_rad = 0.0
        self.last_visual_rigid_command = None
        self.last_control_measurement_ms = None
        self.controller.reset()
        if should_submit_stop:
            self.motor.hard_stop()
            self._record_command_submit_time()

    def _record_command_submit_time(self):
        if self._step_start_ticks_ms is None:
            return
        self.last_command_submit_ms = _ticks_diff(
            _ticks_ms(),
            self._step_start_ticks_ms,
        )

    def _clear_visual_loss_decay(self):
        self.visual_loss_decay_command = None
        self.visual_loss_decay_started_ms = None

    def _feedforward_visual_target(self, raw_command):
        scales = (
            float(getattr(self.config, "FEEDFORWARD_VISUAL_VX_SCALE", 1.0)),
            float(getattr(self.config, "FEEDFORWARD_VISUAL_VY_SCALE", 1.0)),
            float(getattr(self.config, "FEEDFORWARD_VISUAL_W_SCALE", 1.0)),
        )
        limits = (
            float(getattr(self.config, "FEEDFORWARD_VISUAL_MAX_VX", self.config.MAX_VX)),
            float(getattr(self.config, "FEEDFORWARD_VISUAL_MAX_VY", self.config.MAX_VY)),
            float(getattr(self.config, "FEEDFORWARD_VISUAL_MAX_W", self.config.MAX_W)),
        )
        return tuple(
            max(-limits[index], min(raw_command[index] * scales[index], limits[index]))
            for index in range(3)
        )

    def _refresh_selected_visual_command(self):
        raw_command = self.raw_visual_command
        if raw_command is None:
            self.last_visual_command = None
            return
        feedforward_command = self.filtered_feedforward_visual_command
        if feedforward_command is None:
            feedforward_command = self._feedforward_visual_target(raw_command)
        blend = max(0.0, min(float(self.feedforward_visual_blend), 1.0))
        self.last_visual_command = tuple(
            raw_command[index]
            + blend * (feedforward_command[index] - raw_command[index])
            for index in range(3)
        )

    def _set_raw_visual_command(self, command):
        raw_command = tuple(float(value) for value in command)
        linear_scale = max(
            0.0,
            float(
                getattr(
                    self.config,
                    "VISUAL_FOLLOW_LINEAR_COMMAND_SCALE",
                    1.0,
                )
            ),
        )
        raw_command = (
            raw_command[0] * linear_scale,
            raw_command[1] * linear_scale,
            raw_command[2],
        )
        self.raw_visual_command = raw_command
        target = self._feedforward_visual_target(raw_command)
        alpha = max(
            0.0,
            min(
                float(getattr(self.config, "FEEDFORWARD_VISUAL_FILTER_ALPHA", 1.0)),
                1.0,
            ),
        )
        previous = self.filtered_feedforward_visual_command
        if previous is None:
            previous = (0.0, 0.0, 0.0)
        self.filtered_feedforward_visual_command = tuple(
            previous[index] + alpha * (target[index] - previous[index])
            for index in range(3)
        )
        self._refresh_selected_visual_command()

    def _print_follow_control_debug(self):
        """输出当前视觉误差和实际采用的视觉修正，不包含无线前馈。"""
        if not bool(
            getattr(self.config, "FOLLOW_CONTROL_DEBUG_OUTPUT", False)
        ) or not hasattr(self.controller, "get_state"):
            return
        try:
            errors = self.controller.get_state().get("errors", {})
            command = self.last_visual_command
            if command is None:
                return
            print(
                "follow x={:.2f} y={:.2f} theta={:.2f} "
                "vx={:.2f} vy={:.2f} w={:.3f}".format(
                    float(errors.get("lateral", 0.0)),
                    float(errors.get("front", 0.0)),
                    float(errors.get("angle", 0.0)),
                    float(command[0]),
                    float(command[1]),
                    float(command[2]),
                )
            )
        except Exception:
            pass

    def _clear_visual_tracking_commands(self):
        self.last_visual_command = None
        self.raw_visual_command = None
        self.filtered_feedforward_visual_command = None

    def _advance_feedforward_visual_mode(self, active):
        active = bool(active)
        target = 1.0 if active else 0.0
        alpha = max(
            0.0,
            min(
                float(
                    getattr(
                        self.config,
                        "FEEDFORWARD_VISUAL_MODE_BLEND_ALPHA",
                        1.0,
                    )
                ),
                1.0,
            ),
        )
        self.feedforward_visual_blend += alpha * (
            target - self.feedforward_visual_blend
        )
        self.feedforward_visual_mode_active = active
        self._refresh_selected_visual_command()

    def _start_visual_loss_decay(self, now_ms):
        if (
            self.visual_loss_decay_command is None
            and self.last_visual_command is not None
        ):
            self.visual_loss_decay_command = tuple(
                float(value) for value in self.last_visual_command
            )
            self.visual_loss_decay_started_ms = now_ms

    def _visual_loss_decay_at(self, now_ms):
        command = self.visual_loss_decay_command
        started_ms = self.visual_loss_decay_started_ms
        if command is None or started_ms is None:
            return None
        duration_ms = max(
            0,
            int(getattr(self.config, "VISUAL_LOSS_DECAY_MS", 250)),
        )
        if duration_ms <= 0:
            self._clear_visual_loss_decay()
            return None
        elapsed_ms = max(0, _ticks_diff(now_ms, started_ms))
        if elapsed_ms >= duration_ms:
            self._clear_visual_loss_decay()
            return None
        scale = 1.0 - float(elapsed_ms) / float(duration_ms)
        return tuple(value * scale for value in command)

    def _update_relative_heading(self):
        if not hasattr(self.controller, "get_state"):
            self.relative_heading_rad = 0.0
            self.se2_body_heading_rad = 0.0
            return
        state = self.controller.get_state()
        value = state.get("relative_heading_rad", 0.0)
        try:
            value = float(value)
        except (TypeError, ValueError):
            value = 0.0
        if value != value or not -1.0e6 < value < 1.0e6:
            value = 0.0
        limit_deg = float(
            getattr(self.config, "RELATIVE_HEADING_LIMIT_DEG", 45.0)
        )
        limit_rad = math.radians(max(0.0, limit_deg))
        self.relative_heading_rad = max(
            -limit_rad,
            min(value, limit_rad),
        )
        # 航向 P 的符号由 FollowController 的 W_SIGN 决定；车体系转换
        # 则必须独立使用经过现场标定的几何角度符号，不能复用 W_SIGN。
        raw_value = state.get("raw_relative_heading_rad", value)
        try:
            raw_value = float(raw_value)
        except (TypeError, ValueError):
            raw_value = value
        if raw_value != raw_value or not -1.0e6 < raw_value < 1.0e6:
            raw_value = value
        heading_sign = float(
            getattr(self.config, "SE2_BODY_FRAME_HEADING_SIGN", 1.0)
        )
        if heading_sign != heading_sign or abs(heading_sign) < 1.0e-6:
            heading_sign = 1.0
        candidate = max(
            -limit_rad,
            min(heading_sign * raw_value, limit_rad),
        )
        deadband_rad = math.radians(
            max(
                0.0,
                float(
                    getattr(
                        self.config,
                        "SE2_BODY_FRAME_ANGLE_DEADBAND_DEG",
                        0.0,
                    )
                ),
            )
        )
        if abs(candidate) <= deadband_rad:
            candidate = 0.0
        delta = math.atan2(
            math.sin(candidate - self.se2_body_heading_rad),
            math.cos(candidate - self.se2_body_heading_rad),
        )
        max_step_rad = math.radians(
            max(
                0.0,
                float(
                    getattr(
                        self.config,
                        "SE2_BODY_FRAME_MAX_STEP_DEG",
                        180.0,
                    )
                ),
            )
        )
        if max_step_rad > 0.0:
            delta = max(-max_step_rad, min(delta, max_step_rad))
        else:
            delta = 0.0
        self.se2_body_heading_rad = max(
            -limit_rad,
            min(self.se2_body_heading_rad + delta, limit_rad),
        )

    def _update_controller_follower_twist(self):
        if (
            self.odometry is None
            or not hasattr(self.odometry, "get_state")
            or not hasattr(self.controller, "set_follower_twist")
        ):
            return
        state = self.odometry.get_state()
        self.controller.set_follower_twist(
            state.get("body_vx_cm_s", 0.0),
            state.get("body_vy_cm_s", 0.0),
            state.get("yaw_rate_rad_s", 0.0),
        )

    def _update_visual_rigid_command(self):
        self.last_visual_rigid_command = None
        if not hasattr(self.controller, "get_visual_rigid_command"):
            return
        command = self.controller.get_visual_rigid_command()
        if command is None or len(command) != 3:
            return
        try:
            command = tuple(float(value) for value in command)
        except (TypeError, ValueError):
            return
        if not all(
            value == value and -1.0e6 < value < 1.0e6
            for value in command
        ):
            return
        self.last_visual_rigid_command = command

    def _combined_command(
        self,
        visual_command,
        feedforward_command,
        apply_main_rigid_slot=True,
        suppress_visual_xy=False,
    ):
        visual_command = visual_command or (0.0, 0.0, 0.0)
        se2_enabled = bool(
            getattr(
                self.config,
                "SE2_BODY_FRAME_ROTATION_ENABLED",
                True,
            )
        )
        if se2_enabled:
            visual_w = float(visual_command[2])
            visual_compensation_w = _slot_compensation_w(
                visual_w,
                self.config,
            )
            visual_right_offset = float(
                getattr(
                    self.config,
                    "SE2_VISUAL_W_RIGHT_OFFSET_CM",
                    0.0,
                )
            )
            visual_forward_offset = float(
                getattr(
                    self.config,
                    "SE2_VISUAL_W_FORWARD_OFFSET_CM",
                    0.0,
                )
            )
            visual_command = (
                float(visual_command[0])
                - visual_compensation_w * visual_forward_offset,
                float(visual_command[1])
                + visual_compensation_w * visual_right_offset,
                visual_w,
            )
        if suppress_visual_xy:
            visual_command = (0.0, 0.0, visual_command[2])
        feedforward_command = feedforward_command or (0.0, 0.0, 0.0)
        feedforward_w = float(feedforward_command[2])
        slot_compensation_w = _slot_compensation_w(
            feedforward_w,
            self.config,
        )
        right_offset = float(self.config.FORMATION_RIGHT_OFFSET_CM)
        forward_offset = float(self.config.FORMATION_FORWARD_OFFSET_CM)
        if apply_main_rigid_slot:
            # 无线前馈位于主车车体系：先算固定槽位的 v + w×r，再旋转到
            # 辅助车车体系。纯视觉刚体命令已经完成该解算，不能重复处理。
            slot_right_main = (
                float(feedforward_command[0])
                - slot_compensation_w * forward_offset
            )
            slot_forward_main = (
                float(feedforward_command[1])
                + slot_compensation_w * right_offset
            )
            if se2_enabled:
                cosine = math.cos(self.se2_body_heading_rad)
                sine = math.sin(self.se2_body_heading_rad)
                slot_right_follower = (
                    cosine * slot_right_main
                    - sine * slot_forward_main
                )
                slot_forward_follower = (
                    sine * slot_right_main
                    + cosine * slot_forward_main
                )
            else:
                slot_right_follower = slot_right_main
                slot_forward_follower = slot_forward_main
            feedforward_command = (
                slot_right_follower,
                slot_forward_follower,
                feedforward_w,
            )
        else:
            feedforward_command = tuple(
                float(value) for value in feedforward_command
            )
        limits = (
            float(
                getattr(
                    self.config,
                    "MAX_COMMAND_VX",
                    self.config.MAX_VX,
                )
            ),
            float(
                getattr(
                    self.config,
                    "MAX_COMMAND_VY",
                    self.config.MAX_VY,
                )
            ),
            float(
                getattr(
                    self.config,
                    "MAX_COMMAND_W",
                    self.config.MAX_W,
                )
            ),
        )
        combined = (
            visual_command[0] + feedforward_command[0],
            visual_command[1] + feedforward_command[1],
            visual_command[2] + feedforward_command[2],
        )
        return (
            max(-limits[0], min(combined[0], limits[0])),
            max(-limits[1], min(combined[1], limits[1])),
            max(-limits[2], min(combined[2], limits[2])),
        )

    def _wireless_feedforward_is_stationary(self, command):
        if (
            command is None
            or not bool(
                getattr(
                    self.config,
                    "FEEDFORWARD_STATIONARY_VISUAL_XY_SUPPRESS_ENABLED",
                    False,
                )
            )
        ):
            return False
        linear_threshold = max(
            0.0,
            float(
                getattr(
                    self.config,
                    "FEEDFORWARD_STATIONARY_LINEAR_THRESHOLD_CM_S",
                    0.0,
                )
            ),
        )
        angular_threshold = max(
            0.0,
            float(
                getattr(
                    self.config,
                    "FEEDFORWARD_STATIONARY_W_THRESHOLD_RAD_S",
                    0.0,
                )
            ),
        )
        return (
            abs(float(command[0])) < linear_threshold
            and abs(float(command[1])) < linear_threshold
            and abs(float(command[2])) < angular_threshold
        )

    def _required_acquire_cycles(self):
        return max(1, int(self.config.REACQUIRE_FRAMES))

    def _handle_invalid_control_measurement(
        self,
        error,
        now_ms,
        feedforward_command,
    ):
        """丢弃单帧无法投影的测量，连续超时后再执行安全降级。"""
        self.last_error = error
        self.last_event_type = "invalid"
        self.last_event_detail = str(error)
        valid_age_ms = None
        if self.last_control_measurement_ms is not None:
            valid_age_ms = _ticks_diff(
                now_ms,
                self.last_control_measurement_ms,
            )
        if (
            self.last_visual_command is not None
            and valid_age_ms is not None
            and valid_age_ms <= int(self.config.CAMERA_TIMEOUT_MS)
        ):
            self._submit_command(
                self.last_visual_command,
                feedforward_command,
                TRACKING,
            )
            return

        self.acquire_count = 0
        self._start_visual_loss_decay(now_ms)
        self._clear_visual_tracking_commands()
        self.last_visual_rigid_command = None
        self.relative_heading_rad = 0.0
        self.se2_body_heading_rad = 0.0
        self.controller.reset()
        if feedforward_command is not None:
            self._submit_command(
                self._visual_loss_decay_at(now_ms),
                feedforward_command,
                FEEDFORWARD_ONLY,
            )
            return
        self._enter_lost_stop("controller_invalid_timeout")

    def _submit_command(self, visual_command, feedforward_command, state):
        suppress_visual_xy = self._wireless_feedforward_is_stationary(
            feedforward_command
        )
        base_command = feedforward_command
        apply_main_rigid_slot = True
        if (
            base_command is None
            and self.last_visual_rigid_command is not None
        ):
            base_command = self.last_visual_rigid_command
            apply_main_rigid_slot = False
        self.last_command = self._combined_command(
            visual_command,
            base_command,
            apply_main_rigid_slot=apply_main_rigid_slot,
            suppress_visual_xy=suppress_visual_xy,
        )
        self.state = state
        # 无线前馈已经过主车 S 曲线；纯视觉刚体速度则必须及时随新测量更新。
        # 两种基础速度都直接提交，避免辅助车再过一次 S 曲线造成跟随滞后。
        self.motor.command(
            self.last_command[0],
            self.last_command[1],
            self.last_command[2],
        )
        self._record_command_submit_time()

    def step(self, now_ms):
        """运行一个主循环；新鲜前馈先下发，视觉结果缓存到下一轮。"""
        self.last_ff_poll_ms = 0
        self.last_observer_ms = 0
        self.last_sensor_poll_ms = 0
        self.last_controller_ms = 0
        self.last_command_submit_ms = 0
        self._step_start_ticks_ms = _ticks_ms()
        visual_was_tracking = (
            self.state == TRACKING
            and self.last_visual_command is not None
        )

        phase_start_ms = _ticks_ms()
        feedforward_command = None
        if self.feedforward is not None:
            feedforward_command = self.feedforward.poll(now_ms)
        self.last_feedforward_command = feedforward_command
        self.last_ff_poll_ms = _ticks_diff(_ticks_ms(), phase_start_ms)

        # 纯前馈诊断/运行模式：不读摄像头，不运行视觉控制器，
        # 也不使用视觉航向旋转前馈。因此摄像头即使插着，命令路径也与
        # 拔掉摄像头时的纯无线前馈一致。
        if not bool(
            getattr(self.config, "INFRARED_FOLLOW_ENABLED", True)
        ):
            self.acquire_count = 0
            self._clear_visual_tracking_commands()
            self._clear_visual_loss_decay()
            self.feedforward_visual_blend = 0.0
            self.feedforward_visual_mode_active = False
            self.relative_heading_rad = 0.0
            self.se2_body_heading_rad = 0.0
            self.last_visual_rigid_command = None
            self.last_control_measurement_ms = None
            if feedforward_command is not None:
                self.last_event_type = "infrared_follow_disabled"
                self.last_event_detail = None
                self._submit_command(
                    None,
                    feedforward_command,
                    FEEDFORWARD_ONLY,
                )
                return self.state
            self.last_event_detail = "infrared_follow_disabled"
            self._enter_lost_stop("missing_feedforward")
            return self.state

        self._advance_feedforward_visual_mode(
            feedforward_command is not None
        )

        # 前馈是快速通道：使用上一帧已经验证过的视觉修正立即提交，不等待
        # 本轮 UART/视觉控制。随后算出的视觉结果只更新缓存，供下一轮使用。
        cached_visual_command = (
            self.last_visual_command
            if self.last_visual_command is not None
            else self._visual_loss_decay_at(now_ms)
        )
        feedforward_submitted_early = feedforward_command is not None
        if feedforward_submitted_early:
            self._submit_command(
                cached_visual_command,
                feedforward_command,
                (
                    TRACKING
                    if self.last_visual_command is not None
                    else FEEDFORWARD_ONLY
                ),
            )

        # 纯视觉刚体观测器按 10 ms 主循环推进；视觉帧仅用于后续校正。
        phase_start_ms = _ticks_ms()
        self._update_controller_follower_twist()
        if hasattr(self.controller, "predict_visual_rigid"):
            self.controller.predict_visual_rigid(now_ms)
            self._update_visual_rigid_command()
        self.last_observer_ms = _ticks_diff(_ticks_ms(), phase_start_ms)

        phase_start_ms = _ticks_ms()
        try:
            events = self.sensor.poll(now_ms)
        except Exception as error:
            self.last_sensor_poll_ms = _ticks_diff(
                _ticks_ms(),
                phase_start_ms,
            )
            self.last_error = error
            self._enter_lost_stop("sensor_exception")
            raise
        self.last_sensor_poll_ms = _ticks_diff(
            _ticks_ms(),
            phase_start_ms,
        )

        # 视觉 UART 读取本身可能耗时几十毫秒。真实无线接收器用 sequence
        # 标记新帧，因此在进入更慢的控制器前再排空一次无线缓存；如果这段
        # 时间内到达了新命令，立即覆盖前面提交的旧前馈。
        if (
            self.feedforward is not None
            and hasattr(self.feedforward, "last_sequence")
        ):
            sequence_before_repoll = self.feedforward.last_sequence
            repoll_start_ms = _ticks_ms()
            repolled_command = self.feedforward.poll(now_ms)
            self.last_ff_poll_ms += _ticks_diff(
                _ticks_ms(),
                repoll_start_ms,
            )
            sequence_after_repoll = self.feedforward.last_sequence
            repoll_has_new_command = (
                repolled_command is not None
                and (
                    feedforward_command is None
                    or sequence_after_repoll != sequence_before_repoll
                    or tuple(repolled_command) != tuple(feedforward_command)
                )
            )
            if repolled_command is not None:
                feedforward_command = repolled_command
                self.last_feedforward_command = repolled_command
            if repoll_has_new_command:
                if not self.feedforward_visual_mode_active:
                    self._advance_feedforward_visual_mode(True)
                cached_visual_command = (
                    self.last_visual_command
                    if self.last_visual_command is not None
                    else self._visual_loss_decay_at(now_ms)
                )
                self._submit_command(
                    cached_visual_command,
                    feedforward_command,
                    (
                        TRACKING
                        if self.last_visual_command is not None
                        else FEEDFORWARD_ONLY
                    ),
                )
                feedforward_submitted_early = True

        unsafe_type = None
        unsafe_event = None
        latest_measurement = None
        for event in events:
            event_type = event.get("type")
            self.last_event_type = event_type
            if event_type in _UNSAFE_EVENT_TYPES:
                unsafe_type = event_type
                unsafe_event = event
            elif event_type == "measurement":
                latest_measurement = event.get("measurement")
            elif event_type == "invalid":
                # 单个协议坏帧不打断跟随；FollowSensor 会在连续 150 ms
                # 没有有效视觉测量后产生 timeout。
                self.last_event_detail = event.get("reason")
            else:
                unsafe_type = "unknown_event"
                unsafe_event = event

        # 视觉丢失时清除旧视觉修正。只要主车前馈仍新鲜，就按旧方案
        # 继续前馈并让旧视觉修正短暂衰减；视觉和无线两路都失效才硬停车。
        if unsafe_type is not None:
            self.acquire_count = 0
            self._start_visual_loss_decay(now_ms)
            self._clear_visual_tracking_commands()
            self.relative_heading_rad = 0.0
            self.se2_body_heading_rad = 0.0
            self.last_visual_rigid_command = None
            self.last_event_type = unsafe_type
            self.last_event_detail = (
                unsafe_event.get("reason")
                or unsafe_event.get("age_ms")
                or "camera_reported_lost"
            )
            self.controller.reset()
            if feedforward_command is not None:
                self._submit_command(
                    self._visual_loss_decay_at(now_ms),
                    feedforward_command,
                    FEEDFORWARD_ONLY,
                )
                return self.state
            self._enter_lost_stop(unsafe_type)
            return self.state

        if latest_measurement is not None:
            controller_start_ms = _ticks_ms()
            # 不能使用前馈快速提交后临时写入的 state 判断视觉是否已捕获。
            needs_acquire = not visual_was_tracking
            if needs_acquire:
                # 一轮最多增加一次，避免串口积压多帧导致瞬间恢复视觉修正。
                self.acquire_count += 1
                if self.acquire_count < self._required_acquire_cycles():
                    if (
                        feedforward_command is not None
                        and not feedforward_submitted_early
                    ):
                        self._submit_command(
                            None,
                            feedforward_command,
                            FEEDFORWARD_ONLY,
                        )
                    return self.state

                self.controller.reset()
            try:
                raw_visual_command = self.controller.update(
                    latest_measurement,
                    now_ms,
                )
                self._set_raw_visual_command(raw_visual_command)
                self._print_follow_control_debug()
            except ValueError as error:
                self._handle_invalid_control_measurement(
                    error,
                    now_ms,
                    feedforward_command,
                )
                return self.state
            except Exception as error:
                self.last_error = error
                self._enter_lost_stop("controller_exception")
                raise

            self._update_relative_heading()
            self._update_visual_rigid_command()
            self._clear_visual_loss_decay()
            self.last_control_measurement_ms = now_ms
            self.state = TRACKING
            self.acquire_count = 0
            self.last_controller_ms = _ticks_diff(
                _ticks_ms(),
                controller_start_ms,
            )

        if self.last_visual_command is None and feedforward_command is None:
            if self.state == WAIT_TARGET:
                return self.state
            self._enter_lost_stop("missing_sources")
            return self.state

        next_state = (
            TRACKING
            if self.last_visual_command is not None
            else FEEDFORWARD_ONLY
        )
        if feedforward_submitted_early:
            # 当前视觉修正已缓存；不要在慢视觉路径末尾重复提交。若上面检测到
            # LOST/TIMEOUT/异常，安全分支仍会立即覆盖为纯前馈或硬停车。
            self.state = next_state
            return self.state
        self._submit_command(
            self.last_visual_command,
            feedforward_command,
            next_state,
        )
        return self.state

    def get_state(self):
        return {
            "state": self.state,
            "acquire_count": self.acquire_count,
            "last_command": self.last_command,
            "last_event_type": self.last_event_type,
            "last_event_detail": self.last_event_detail,
            "last_error": self.last_error,
            "last_feedforward_command": self.last_feedforward_command,
            "last_ff_poll_ms": self.last_ff_poll_ms,
            "last_observer_ms": self.last_observer_ms,
            "last_sensor_poll_ms": self.last_sensor_poll_ms,
            "last_controller_ms": self.last_controller_ms,
            "last_command_submit_ms": self.last_command_submit_ms,
            "last_visual_rigid_command": self.last_visual_rigid_command,
            "last_control_measurement_ms": self.last_control_measurement_ms,
        }


def _debug(config, message):
    if getattr(config, "DEBUG_OUTPUT", False):
        try:
            print(message)
        except Exception:
            pass


def _init_camera_uart(uart, config):
    """Configure non-blocking camera UART with firmware-compatible fallback."""
    baud = int(config.BAUD)
    timeout = int(getattr(config, "UART_TIMEOUT_MS", 0))
    timeout_char = int(getattr(config, "UART_TIMEOUT_CHAR_MS", 0))
    try:
        uart.init(
            baud,
            timeout=timeout,
            timeout_char=timeout_char,
        )
        return "timeout_char"
    except (TypeError, ValueError):
        try:
            uart.init(baud, timeout=timeout)
            return "timeout"
        except (TypeError, ValueError):
            uart.init(baud)
            return "basic"


def main():
    """创建板端硬件并持续运行动态跟随。"""
    import config
    from follow import FollowController, FollowSensor
    from machine import UART
    from motor import MotorSystem
    from odometry import OdometrySystem
    from wireless_feedforward import FeedforwardReceiver

    motor = None
    runtime = None
    offline_logger = None
    try:
        odometry = OdometrySystem()
        motor = MotorSystem(odometry=odometry)
        motor.start()

        infrared_follow_enabled = bool(
            getattr(config, "INFRARED_FOLLOW_ENABLED", True)
        )
        if infrared_follow_enabled:
            camera_uart = UART(config.UART_ID)
            camera_uart_mode = _init_camera_uart(camera_uart, config)
            _debug(config, "camera UART init: " + camera_uart_mode)
            sensor = FollowSensor(camera_uart, config)
        else:
            sensor = None
            _debug(config, "infrared follow disabled; feedforward only")
        controller = FollowController(config)
        feedforward = FeedforwardReceiver(
            baud=config.WIRELESS_BAUD,
            timeout_ms=config.FEEDFORWARD_TIMEOUT_MS,
        )
        runtime = AssistantRuntime(
            motor,
            sensor,
            controller,
            config,
            feedforward=feedforward,
            odometry=odometry,
        )
        runtime.start()
        offline_logger = OfflineLogger(config)
        if offline_logger.error is not None:
            _debug(
                config,
                "offline log disabled: " + str(offline_logger.error),
            )
        _debug(config, "assistant visual follow started")

        last_print_state = None
        while True:
            loop_start_ms = _ticks_ms()
            current_state = runtime.step(loop_start_ms)
            step_elapsed_ms = _ticks_diff(_ticks_ms(), loop_start_ms)
            try:
                offline_logger.sample(
                    loop_start_ms,
                    runtime,
                    step_elapsed_ms,
                )
            except Exception as error:
                # 日志属于诊断旁路，任何格式化或文件系统异常都不能打断控制。
                offline_logger.error = repr(error)
                offline_logger.enabled = False
                offline_logger.pending = []
                offline_logger._close_stream()
            if current_state != last_print_state:
                message = "state: " + current_state
                if current_state in (LOST_STOP, FEEDFORWARD_ONLY):
                    message += " reason=" + str(runtime.last_event_type)
                    message += ":" + str(runtime.last_event_detail)
                _debug(config, message)
                last_print_state = current_state

            elapsed_ms = _ticks_diff(_ticks_ms(), loop_start_ms)
            remaining_ms = int(config.CONTROL_PERIOD_MS) - elapsed_ms
            if remaining_ms > 0:
                _sleep_ms(remaining_ms)

    except KeyboardInterrupt:
        if runtime is not None:
            runtime.last_error = "KeyboardInterrupt"
        _debug(config, "assistant stopped by user")
    except Exception as error:
        if runtime is not None:
            runtime.last_error = error
        _debug(config, "assistant error: " + repr(error))
        try:
            import sys

            sys.print_exception(error)
        except Exception:
            pass
    finally:
        if offline_logger is not None:
            offline_logger.close()
        if motor is not None:
            try:
                motor.hard_stop()
            finally:
                motor.stop()


if __name__ == "__main__":
    main()

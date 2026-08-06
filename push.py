"""主车推行控制器。

接口与 ``approach.py``、``orbit.py`` 一致：每次 ``step`` 返回一个
``MotionStep``，调用方将 command 交给 MotorSystem。推行过程中主车锁定
推进航向，用主摄目标像素保持横向/纵向接触，支持黄线停车及双车同侧推行避障。
"""

import math
import time

from control import MotionStep, PIDController, clamp, finite, normalize_angle


class State:
    """MicroPython-compatible Push states (avoid CPython's enum module)."""

    PUSH_NORMAL = "PUSH_NORMAL"
    AVOID_ENTER = "AVOID_ENTER"
    AVOID_TRACK = "AVOID_TRACK"
    AVOID_CLEAR_HOLD = "AVOID_CLEAR_HOLD"
    AVOID_RETURN = "AVOID_RETURN"
    YELLOW_STOP = "YELLOW_STOP"
    YELLOW_DELAY = "YELLOW_DELAY"


def _value(source, key, default=None):
    if isinstance(source, dict):
        return source.get(key, default)
    return getattr(source, key, default)


def _cfg(config, key, default=None):
    if isinstance(config, dict):
        return config.get(key, default)
    return getattr(config, key, default)


def _hypot(x, y):
    """MicroPython-compatible two-dimensional vector magnitude."""
    return math.sqrt(x * x + y * y)


def _ticks_diff(new_value, old_value):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(new_value, old_value)
    return new_value - old_value


def _target_data(target):
    if target is None:
        return None
    found = bool(
        _value(target, "found", _value(target, "target_found", False))
    )
    if not found:
        return None
    x = float(_value(target, "x", _value(target, "target_x", 0.0)))
    y = float(_value(target, "y", _value(target, "target_y", 0.0)))
    class_id = int(
        _value(target, "class_id", _value(target, "target_id", 0)) or 0
    )
    if not finite(x) or not finite(y):
        return False
    return x, y, class_id


def _hazard_data(hazard):
    if hazard is None:
        return None
    found = bool(
        _value(hazard, "hazard_found", _value(hazard, "found", False))
    )
    kind = int(_value(hazard, "hazard_type", 0) or 0)
    x = _value(hazard, "hazard_x", _value(hazard, "x", 0.0))
    y = _value(hazard, "hazard_y", _value(hazard, "y", 0.0))
    try:
        x = float(x)
        y = float(y)
    except (TypeError, ValueError):
        return False
    if not finite(x) or not finite(y):
        return False
    return {
        "found": found,
        "hazard_type": kind,
        "x": x,
        "y": y,
        "frame_ms": _value(hazard, "frame_ms", None),
        "frame_sequence": _value(hazard, "frame_sequence", None),
    }


class PushController:
    """直推、视觉保持及黄线/障碍安全处理。"""

    def __init__(self, config):
        self.config = config
        self.x_pid = PIDController(
            _cfg(config, "PID_X_KP", 0.3),
            _cfg(config, "PID_X_KI", 0.0),
            _cfg(config, "PID_X_KD", 0.0),
            output_limit=_cfg(config, "MAX_LATERAL_SPEED_CM_S", 100.0),
            integral_limit=_cfg(config, "PID_X_I_LIMIT", 100.0),
        )
        self.y_pid = PIDController(
            _cfg(config, "PID_Y_KP", 0.3),
            _cfg(config, "PID_Y_KI", 0.0),
            _cfg(config, "PID_Y_KD", 0.0),
            output_limit=_cfg(config, "MAX_FORWARD_ADJUST_CM_S", 90.0),
            integral_limit=_cfg(config, "PID_Y_I_LIMIT", 200.0),
        )
        self.contact_pid = PIDController(
            _cfg(config, "CONTACT_KP", 0.0),
            _cfg(config, "CONTACT_KI", 0.0),
            _cfg(config, "CONTACT_KD", 0.0),
            output_limit=_cfg(config, "CONTACT_MAX_ADJUST_CM_S", 40.0),
            integral_limit=_cfg(config, "CONTACT_I_LIMIT", 100.0),
        )
        self.reset()

    def reset(self):
        self.active = False
        self.target_heading_rad = 0.0
        self.class_id = 0
        self.x_pid.reset()
        self.y_pid.reset()
        self.contact_pid.reset()

        self.state = State.PUSH_NORMAL
        self.time_in_state = 0.0
        self.total_time = 0.0
        self.target_lost_time = 0.0

        self.original_heading = 0.0
        self.avoid_direction = 1.0  # 1 for left, -1 for right
        self.current_gear = 0       # 0: None, 1: Far, 2: Near, 3: Danger
        self.stable_gear_frames = 0
        self.candidate_gear = 0
        self.current_w = 0.0
        self.last_avoid_angle = 0.0
        self.last_speed_scale = 1.0
        self.avoid_angle_rad = 0.0
        self.return_stable_s = 0.0
        self.clear_start_frame_ms = None
        self.last_clear_frame_ms = None
        self.last_hazard_sequence = None

    def start(self, current_heading_rad, class_id=0):
        self.reset()
        self.active = True
        self.target_heading_rad = current_heading_rad
        self.class_id = class_id
        self.original_heading = current_heading_rad

    def _transition_to(self, new_state):
        self.state = new_state
        self.time_in_state = 0.0

    def _heading_command(
        self,
        desired_heading_rad,
        current_heading_rad,
        yaw_rate_rad_s,
    ):
        error = normalize_angle(desired_heading_rad - current_heading_rad)
        if abs(error) <= _cfg(self.config, "HEADING_DEADBAND_RAD", 0.0):
            w = 0.0
        else:
            w = _cfg(self.config, "HEADING_KP", 2.0) * error
            w -= _cfg(self.config, "HEADING_KD", 0.0) * float(
                yaw_rate_rad_s
            )
        maximum = _cfg(self.config, "MAX_W_RAD_S", 1.2)
        return clamp(w, -maximum, maximum), error

    def _slew_avoid_angle(self, target_angle_rad, dt):
        maximum_rate = _cfg(
            self.config,
            "AVOID_TARGET_ANGLE_SLEW_RAD_S",
            math.radians(90.0),
        )
        maximum_change = max(0.0, float(maximum_rate)) * dt
        delta = clamp(
            float(target_angle_rad) - self.avoid_angle_rad,
            -maximum_change,
            maximum_change,
        )
        self.avoid_angle_rad += delta
        return self.avoid_angle_rad

    def _gear_from_hazard_y(self, hazard_y):
        near = _cfg(self.config, "AVOID_Y_NEAR_PX", 30.0)
        danger = _cfg(self.config, "AVOID_Y_DANGER_PX", 60.0)
        near_hysteresis = _cfg(
            self.config, "AVOID_PIXEL_HYSTERESIS_NEAR", 3.0
        )
        danger_hysteresis = _cfg(
            self.config, "AVOID_PIXEL_HYSTERESIS_DANGER", 5.0
        )
        if self.current_gear >= 3:
            if hazard_y >= danger - danger_hysteresis:
                return 3
            if hazard_y >= near + near_hysteresis:
                return 2
            return 1
        if self.current_gear == 2:
            if hazard_y >= danger + danger_hysteresis:
                return 3
            if hazard_y >= near - near_hysteresis:
                return 2
            return 1
        if hazard_y >= danger + danger_hysteresis:
            return 3
        if hazard_y >= near + near_hysteresis:
            return 2
        return 1

    def _update_gear(self, hazard_y):
        candidate = self._gear_from_hazard_y(hazard_y)
        if candidate >= self.current_gear:
            self.current_gear = candidate
            self.candidate_gear = candidate
            self.stable_gear_frames = 0
            return
        if candidate == self.candidate_gear:
            self.stable_gear_frames += 1
        else:
            self.candidate_gear = candidate
            self.stable_gear_frames = 1
        if self.stable_gear_frames >= int(
            _cfg(self.config, "AVOID_GEAR_DOWN_STABLE_FRAMES", 5)
        ):
            self.current_gear = candidate
            self.stable_gear_frames = 0

    def _gear_command(self):
        if self.current_gear >= 3:
            return (
                _cfg(self.config, "AVOID_ANGLE_DANGER_RAD", math.radians(60.0)),
                _cfg(self.config, "AVOID_SPEED_SCALE_DANGER", 0.30),
            )
        if self.current_gear == 2:
            return (
                _cfg(self.config, "AVOID_ANGLE_NEAR_RAD", math.radians(45.0)),
                _cfg(self.config, "AVOID_SPEED_SCALE_NEAR", 0.50),
            )
        return (
            _cfg(self.config, "AVOID_ANGLE_FAR_RAD", math.radians(20.0)),
            _cfg(self.config, "AVOID_SPEED_SCALE_FAR", 0.75),
        )

    def _target_lost_continue_step(self, current_heading_rad, yaw_rate_rad_s):
        """测试专用：丢目标后保持固定前推和既有航向保持。"""
        target_w, _ = self._heading_command(
            self.target_heading_rad,
            current_heading_rad,
            yaw_rate_rad_s,
        )
        return MotionStep(
            (
                0.0,
                _cfg(self.config, "TARGET_LOSS_FORWARD_SPEED_CM_S", 150.0),
                target_w,
            ),
            reason="push_target_lost_continue",
        )

    def step(
        self,
        target,
        tof,
        current_heading_rad,
        hazard=None,
        yaw_rate_rad_s=0.0,
        dt=0.02,
    ):
        if not self.active:
            return MotionStep.stop("not_active")

        self.total_time += dt
        self.time_in_state += dt

        target_data = _target_data(target)
        if target_data is False:
            return MotionStep.stop("push_target_error", failed=True)
        hazard_data = _hazard_data(hazard)
        if hazard_data is False:
            return MotionStep.stop("push_hazard_error", failed=True)

        # 黄线判断：增加 y > 100 阈值，并加入 1 秒延迟
        if (
            hazard_data
            and hazard_data["found"]
            and hazard_data["hazard_type"]
            == _cfg(self.config, "HAZARD_YELLOW", 6)
            and hazard_data["y"] > 100
        ):
            if self.state != State.YELLOW_DELAY and self.state != State.YELLOW_STOP:
                self._transition_to(State.YELLOW_DELAY)

        if self.state == State.YELLOW_DELAY:
            if self.time_in_state >= 0.3:
                self._transition_to(State.YELLOW_STOP)
                self.active = False
                return MotionStep.stop(
                    "push_yellow_line_hard_stop",
                    done=True,
                    debug={
                        "hard_stop": True,
                        "yellow_type": 6,
                        "yellow_delay_completed": True,
                    },
                )
            # 延迟期间不返回 stop，继续走下面的代码保持速度
        elif self.state == State.YELLOW_STOP:
            return MotionStep.stop("push_yellow_line_hard_stop", done=True)

        if self.total_time > _cfg(self.config, "PUSH_DURATION_S", 10.0):
            self.active = False
            return MotionStep.stop("push_duration_complete", done=True)

        avoidance_enabled = _cfg(
            self.config, "PUSH_AVOIDANCE_ENABLED", True
        )
        obstacle_found = bool(
            hazard_data
            and hazard_data["found"]
            and hazard_data["hazard_type"]
            == _cfg(self.config, "HAZARD_OBSTACLE", 7)
        )
        explicit_clear = bool(hazard_data and not hazard_data["found"])
        frame_sequence = (
            hazard_data.get("frame_sequence") if hazard_data else None
        )
        is_new_hazard_frame = bool(
            hazard_data
            and (
                frame_sequence is None
                or frame_sequence != self.last_hazard_sequence
            )
        )
        if hazard_data and frame_sequence is not None:
            self.last_hazard_sequence = frame_sequence

        if self.state == State.PUSH_NORMAL and avoidance_enabled:
            if obstacle_found:
                self._transition_to(State.AVOID_ENTER)

        if self.state == State.AVOID_ENTER:
            if not obstacle_found:
                return MotionStep.stop(
                    "push_hazard_timeout_before_avoidance", failed=True
                )
            h_x = hazard_data["x"]
            center_x = _cfg(self.config, "AVOID_CENTER_X_PX", 160.0)
            deadband = _cfg(self.config, "AVOID_CENTER_DEADBAND_PX", 10.0)

            if h_x < center_x - deadband:
                self.avoid_direction = -1.0  # Object left, steer right
            elif h_x > center_x + deadband:
                self.avoid_direction = 1.0   # Object right, steer left
            else:
                pref = _cfg(self.config, "PREFERRED_AVOID_DIRECTION", "left")
                self.avoid_direction = 1.0 if pref == "left" else -1.0

            self.original_heading = self.target_heading_rad
            self.current_gear = 0
            self.current_w = 0.0
            self.avoid_angle_rad = 0.0
            self._update_gear(hazard_data["y"])
            self._transition_to(State.AVOID_TRACK)

        if self.state in (
            State.AVOID_TRACK,
            State.AVOID_CLEAR_HOLD,
            State.AVOID_RETURN,
        ):
            if self.state == State.AVOID_TRACK:
                if obstacle_found:
                    self._update_gear(hazard_data["y"])
                elif explicit_clear and is_new_hazard_frame:
                    frame_ms = hazard_data.get("frame_ms")
                    self.clear_start_frame_ms = frame_ms
                    self.last_clear_frame_ms = frame_ms
                    self._transition_to(State.AVOID_CLEAR_HOLD)
                elif hazard_data is None:
                    return MotionStep.stop("push_hazard_timeout", failed=True)

            elif self.state == State.AVOID_CLEAR_HOLD:
                if obstacle_found:
                    self._transition_to(State.AVOID_TRACK)
                    self.clear_start_frame_ms = None
                    self.last_clear_frame_ms = None
                elif hazard_data is None:
                    return MotionStep.stop("push_hazard_timeout", failed=True)
                elif explicit_clear and is_new_hazard_frame:
                    frame_ms = hazard_data.get("frame_ms")
                    if frame_ms is not None:
                        self.last_clear_frame_ms = frame_ms
                    clear_hold_s = _cfg(
                        self.config, "AVOID_CLEAR_HOLD_S", 0.5
                    )
                    if (
                        self.clear_start_frame_ms is not None
                        and self.last_clear_frame_ms is not None
                    ):
                        clear_complete = (
                            _ticks_diff(
                                self.last_clear_frame_ms,
                                self.clear_start_frame_ms,
                            )
                        ) >= int(clear_hold_s * 1000.0)
                    else:
                        clear_complete = self.time_in_state >= clear_hold_s
                    if clear_complete:
                        self.return_stable_s = 0.0
                        self._transition_to(State.AVOID_RETURN)

            elif self.state == State.AVOID_RETURN:
                if obstacle_found:
                    self._transition_to(State.AVOID_TRACK)
                elif hazard_data is None:
                    return MotionStep.stop("push_hazard_timeout", failed=True)

        avoidance_active = self.state in (
            State.AVOID_TRACK,
            State.AVOID_CLEAR_HOLD,
            State.AVOID_RETURN,
        )

        continue_after_target_loss = _cfg(
            self.config, "TARGET_LOSS_CONTINUE_ENABLED", False
        )
        target_lost_continue = False
        if target_data is None:
            self.target_lost_time += dt
            if not avoidance_active:
                if continue_after_target_loss:
                    if _cfg(
                        self.config, "PUSH_SINGLE_VEHICLE_MODE", False
                    ):
                        return self._target_lost_continue_step(
                            current_heading_rad, yaw_rate_rad_s
                        )
                    target_lost_continue = True
                else:
                    return MotionStep.stop("push_target_lost", failed=True)
            target_x_px = _cfg(self.config, "TARGET_CENTER_X_PX", 160.0)
            target_y_px = _cfg(self.config, "TARGET_Y_PX", 170.0)
        else:
            self.target_lost_time = 0.0
            target_x_px, target_y_px, _ = target_data

        x_error = target_x_px - _cfg(
            self.config, "TARGET_CENTER_X_PX", 160.0
        )
        y_error = _cfg(self.config, "TARGET_Y_PX", 170.0) - target_y_px
        vox = self.x_pid.update(x_error, dt) if target_data else 0.0

        fixed_forward_speed = _cfg(
            self.config, "PUSH_FIXED_FORWARD_SPEED_CM_S", None
        )
        if fixed_forward_speed is None:
            ramp_s = _cfg(self.config, "PUSH_RAMP_S", 0.7)
            start_v = _cfg(self.config, "PUSH_START_SPEED_CM_S", 35.0)
            target_v = _cfg(self.config, "PUSH_SPEED_CM_S", 225.0)
            if self.total_time < ramp_s and ramp_s > 0.0:
                base_voy = start_v + (
                    target_v - start_v
                ) * (self.total_time / ramp_s)
            else:
                base_voy = target_v
        else:
            base_voy = float(fixed_forward_speed)

        closed_loop_push = avoidance_active or _cfg(
            self.config, "PUSH_Y_TOF_GOVERNOR_ENABLED", False
        )
        y_adjust = 0.0
        contact_adjust = 0.0
        if closed_loop_push and target_data is not None:
            y_adjust = self.y_pid.update(y_error, dt)
        else:
            self.y_pid.reset()

        try:
            tof_value = float(tof)
            tof_valid = (
                finite(tof_value)
                and _cfg(self.config, "TOF_VALID_MIN_MM", 20.0)
                <= tof_value
                <= _cfg(self.config, "TOF_VALID_MAX_MM", 1500.0)
            )
        except (TypeError, ValueError):
            tof_valid = False
            tof_value = 0.0
        if closed_loop_push and tof_valid:
            contact_error = tof_value - _cfg(
                self.config, "CONTACT_DISTANCE_MM", 30.0
            )
            contact_adjust = self.contact_pid.update(contact_error, dt)
        else:
            self.contact_pid.reset()
        voy = base_voy + y_adjust + contact_adjust

        speed_scale = 1.0
        if avoidance_active:
            if self.state == State.AVOID_RETURN:
                target_avoid_angle = 0.0
                speed_scale = 1.0
            else:
                angle_magnitude, speed_scale = self._gear_command()
                target_avoid_angle = angle_magnitude * self.avoid_direction
                self.last_avoid_angle = target_avoid_angle
                self.last_speed_scale = speed_scale
            applied_angle = self._slew_avoid_angle(target_avoid_angle, dt)
            desired_heading = normalize_angle(
                self.original_heading + applied_angle
            )
            target_w_raw, heading_error = self._heading_command(
                desired_heading,
                current_heading_rad,
                yaw_rate_rad_s,
            )
            max_accel = _cfg(
                self.config, "AVOID_MAX_W_ACCEL_RAD_S2", 6.0
            )
            max_delta_w = max(0.0, float(max_accel)) * dt
            self.current_w = clamp(
                target_w_raw,
                self.current_w - max_delta_w,
                self.current_w + max_delta_w,
            )
            max_w = _cfg(self.config, "AVOID_MAX_W_RAD_S", 1.2)
            target_w = clamp(self.current_w, -max_w, max_w)
            voy *= speed_scale
            if self.state == State.AVOID_RETURN:
                angle_done = abs(self.avoid_angle_rad) <= math.radians(1.0)
                heading_done = abs(heading_error) <= _cfg(
                    self.config,
                    "AVOID_RETURN_TOLERANCE_RAD",
                    math.radians(2.0),
                )
                if angle_done and heading_done:
                    self.return_stable_s += dt
                    if self.return_stable_s >= _cfg(
                        self.config, "AVOID_RETURN_STABLE_S", 0.1
                    ):
                        self.target_heading_rad = self.original_heading
                        self.current_w = 0.0
                        self._transition_to(State.PUSH_NORMAL)
                else:
                    self.return_stable_s = 0.0
        else:
            target_w, heading_error = self._heading_command(
                self.target_heading_rad,
                current_heading_rad,
                yaw_rate_rad_s,
            )

        if _cfg(self.config, "PUSH_SINGLE_VEHICLE_MODE", False):
            # 单车直推不使用双车编队刚体补偿。
            vmx, vmy, wm = vox, voy, target_w
            vfx, vfy = vmx, vmy
            f_speed = 0.0
        else:
            # 双车同侧推行的刚体补偿。
            L = _cfg(self.config, "OBJECT_FORWARD_OFFSET_CM", 10.0)
            B = _cfg(self.config, "FORMATION_BASELINE_CM", 20.0)
            vmx = vox + target_w * L
            vmy = voy + target_w * (B / 2.0)
            wm = target_w
            vfx = vox + target_w * L
            vfy = voy - target_w * (B / 2.0)
            f_speed = _hypot(vfx, vfy)

        # Max Limits Proportional Scaling
        max_xy = _cfg(self.config, "VEHICLE_MAX_XY_SPEED_CM_S", 700.0)
        max_v_w = _cfg(self.config, "VEHICLE_MAX_W_RAD_S", 3.4)

        scale = 1.0
        m_speed = _hypot(vmx, vmy)
        if m_speed > max_xy:
            scale = min(scale, max_xy / m_speed)
        if f_speed > max_xy:
            scale = min(scale, max_xy / f_speed)
        if abs(wm) > max_v_w:
            scale = min(scale, max_v_w / abs(wm))

        vmx *= scale
        vmy *= scale
        wm *= scale
        vfx *= scale
        vfy *= scale

        return MotionStep(
            (vmx, vmy, wm),
            reason=(
                "push_target_lost_continue"
                if target_lost_continue
                else "push_running_" + self.state.lower()
            ),
            debug={
                "push_hazard": (
                    None
                    if hazard_data is None
                    else (
                        hazard_data["found"],
                        hazard_data["hazard_type"],
                        hazard_data["x"],
                        hazard_data["y"],
                    )
                ),
                "avoid_gear": self.current_gear,
                "avoid_angle_rad": self.avoid_angle_rad,
                "avoid_speed_scale": speed_scale,
                "object_command": (vox, voy, target_w),
                "predicted_main_command": (vmx, vmy, wm),
                "predicted_follower_command": (vfx, vfy, wm),
                "rigid_scale": scale,
                "y_adjust_cm_s": y_adjust,
                "contact_adjust_cm_s": contact_adjust,
            },
        )

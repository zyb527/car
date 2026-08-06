"""car141929 绕行逻辑的当前底盘接口版本。

保留原绕行、最终对准、贴靠三个阶段和原PID结构。所有 vx/vy 输出按
当前底盘比例缩放，角速度逻辑保持 car141929。
"""

import math

from control import (
    MotionStep,
    PIDController,
    clamp,
    finite,
    limit_vector,
    normalize_angle,
)


PHASE_ORBITING = "ORBITING"
PHASE_ALIGN = "ALIGN"
PHASE_CLOSE_IN = "CLOSE_IN"


def _target_value(target, key, default=None):
    if isinstance(target, dict):
        return target.get(key, default)
    return getattr(target, key, default)


def _cfg(config, name, default=None):
    if isinstance(config, dict):
        return config.get(name, default)
    return getattr(config, name, default)


def get_orbit_direction(config):
    direction = str(
        _cfg(config, "ORBIT_DIRECTION", "left")
    ).lower()
    if direction not in ("left", "right"):
        direction = "left"
    return direction


def optimize_orbit_direction(current_heading_rad, target_heading_rad):
    diff = normalize_angle(target_heading_rad - current_heading_rad)
    if diff >= 0.0:
        return "right"
    return "left"


def get_orbit_base_w(config, direction=None):
    if direction is None:
        direction = get_orbit_direction(config)
    base_w_abs = abs(
        _cfg(config, "ORBIT_ROTATION_SPEED_RAD_S", 0.15)
    )
    if direction == "left":
        return -base_w_abs
    return base_w_abs


def calc_camera_turn(target_x, camera_turn_pid, dt, config):
    error_x = (
        target_x - _cfg(config, "TARGET_CENTER_X_PX", 160.0)
    )
    dead_band = _cfg(
        config,
        "CAMERA_TURN_DEAD_BAND_X_PX",
        10.0,
    )
    if abs(error_x) <= dead_band:
        camera_turn_pid.reset()
        return 0.0
    return camera_turn_pid.update(error_x, dt)


def calc_orbit_visual_heading_w(
    target_x,
    camera_turn_pid,
    dt,
    config,
    weight,
):
    visual_w = calc_camera_turn(
        target_x,
        camera_turn_pid,
        dt,
        config,
    )
    # car141929 camera_turn_sign=-1。
    return visual_w * -1.0 * float(weight)


def calc_orbit_radius_correction(
    target_y,
    tof_found,
    tof_distance_mm,
    orbit_radius_mm,
    orbit_target_y,
    tof_pid,
    y_pid,
    dt,
    config,
):
    vy_tof = 0.0
    if tof_found:
        distance_error_mm = tof_distance_mm - orbit_radius_mm
        vy_tof = tof_pid.update(distance_error_mm, dt)
    else:
        tof_pid.reset()

    y_error = orbit_target_y - target_y
    if abs(y_error) <= _cfg(
        config,
        "ORBIT_Y_DEAD_BAND_PX",
        4.0,
    ):
        y_pid.reset()
        vy_y = 0.0
    else:
        vy_y = y_pid.update(y_error, dt)

    if tof_found:
        weight_tof = _cfg(config, "ORBIT_TOF_WEIGHT", 0.2)
        vy_cmd = (
            vy_tof * weight_tof
            + vy_y * (1.0 - weight_tof)
        )
    else:
        vy_cmd = vy_y
    maximum = _cfg(config, "ORBIT_MAX_VY_CM_S", 60.0)
    return clamp(vy_cmd, -maximum, maximum)


def apply_radius_band_vy(
    vy_cmd,
    tof_found,
    distance_mm,
    orbit_radius_mm,
    config,
):
    if (
        not _cfg(config, "ORBIT_BAND_VY_ENABLED", True)
        or not tof_found
    ):
        return vy_cmd, 0.0

    scale = _cfg(config, "LINEAR_SPEED_SCALE", 0.5)
    band_vy = 0.0
    if distance_mm < orbit_radius_mm:
        band_vy = max(
            -80.0 * scale,
            (distance_mm - orbit_radius_mm) * 0.75 * scale,
        )
    elif 190.0 < distance_mm <= 320.0:
        band_vy = min(
            45.0 * scale,
            (distance_mm - orbit_radius_mm) * 0.25 * scale,
        )

    if band_vy < 0.0:
        vy_cmd = min(vy_cmd, band_vy)
    elif band_vy > 0.0:
        vy_cmd = max(vy_cmd, band_vy)
    maximum = _cfg(config, "ORBIT_MAX_VY_CM_S", 60.0)
    return clamp(vy_cmd, -maximum, maximum), band_vy


def calc_orbit_command(
    target_x,
    target_y,
    tof_found,
    tof_distance_mm,
    orbit_radius_mm,
    orbit_target_y,
    camera_turn_pid,
    tof_pid,
    y_pid,
    dt,
    config,
    heading_error=None,
    direction=None,
):
    base_w = get_orbit_base_w(config, direction)
    if heading_error is not None:
        abs_error = abs(heading_error)
        slow_start = _cfg(
            config,
            "ORBIT_SLOW_DOWN_START_RAD",
            math.radians(20.0),
        )
        min_scale = _cfg(
            config,
            "ORBIT_SLOW_DOWN_MIN_SCALE",
            0.35,
        )
        if abs_error < slow_start:
            scale = min_scale + (
                1.0 - min_scale
            ) * (abs_error / slow_start)
            base_w *= scale

    correction = calc_orbit_visual_heading_w(
        target_x,
        camera_turn_pid,
        dt,
        config,
        _cfg(config, "ORBIT_CAMERA_W_WEIGHT", 0.65),
    )
    max_w = _cfg(config, "ORBIT_MAX_W_RAD_S", 4.0)
    w_cmd = clamp(base_w + correction, -max_w, max_w)
    w_scale = _cfg(config, "ORBIT_W_SCALE", 1.0)
    # 几何圆弧严格满足 v_tangent = angular_rate * radius。若切向速度
    # 受限，则同步限制角速度，避免经验比例压缩真实轨迹半径。
    w_cmd *= w_scale
    radius_cm = (
        float(orbit_radius_mm)
        + float(_cfg(config, "TOF_CENTER_OFFSET_MM", 0.0))
    ) / 10.0
    max_vx = abs(float(_cfg(config, "ORBIT_MAX_VX_CM_S", 380.0)))
    if radius_cm > 1.0e-6:
        w_limit_by_speed = max_vx / radius_cm
        w_cmd = clamp(w_cmd, -w_limit_by_speed, w_limit_by_speed)
        vx_cmd = w_cmd * radius_cm
    else:
        vx_cmd = 0.0
    vy_cmd = calc_orbit_radius_correction(
        target_y,
        tof_found,
        tof_distance_mm,
        orbit_radius_mm,
        orbit_target_y,
        tof_pid,
        y_pid,
        dt,
        config,
    )
    vy_cmd, _ = apply_radius_band_vy(
        vy_cmd,
        tof_found,
        tof_distance_mm,
        orbit_radius_mm,
        config,
    )
    return vx_cmd, vy_cmd, w_cmd


class OrbitController:
    """绕行、斜推杆横向对位、贴近三阶段控制器。"""

    def __init__(self, config):
        self.config = config
        self.camera_turn_pid = PIDController(
            config.PID_CAMERA_TURN_KP,
            config.PID_CAMERA_TURN_KI,
            config.PID_CAMERA_TURN_KD,
            output_limit=config.PID_CAMERA_TURN_OUTPUT_LIMIT,
            integral_limit=config.PID_CAMERA_TURN_I_LIMIT,
        )
        self.tof_pid = PIDController(
            config.PID_ORBIT_TOF_KP,
            config.PID_ORBIT_TOF_KI,
            config.PID_ORBIT_TOF_KD,
            output_limit=config.ORBIT_MAX_VY_CM_S,
            integral_limit=config.PID_ORBIT_TOF_I_LIMIT,
        )
        self.y_pid = PIDController(
            config.PID_ORBIT_Y_KP,
            config.PID_ORBIT_Y_KI,
            config.PID_ORBIT_Y_KD,
            output_limit=config.ORBIT_MAX_VY_CM_S,
            integral_limit=config.PID_ORBIT_Y_I_LIMIT,
        )
        self.x_pid = PIDController(
            config.PID_X_KP,
            config.PID_X_KI,
            config.PID_X_KD,
            output_limit=config.ORBIT_MAX_VY_CM_S,
            integral_limit=config.PID_X_I_LIMIT,
        )
        self.reset()

    def _reset_pids(self):
        self.camera_turn_pid.reset()
        self.tof_pid.reset()
        self.y_pid.reset()
        self.x_pid.reset()

    def reset(self):
        self.active = False
        self.phase = PHASE_ORBITING
        self.direction = get_orbit_direction(self.config)
        self.target_heading_rad = 0.0
        self.orbit_radius_mm = self.config.ORBIT_MIN_RADIUS_MM
        self.orbit_target_y = self.config.ORBIT_ROD_TARGET_Y_PX
        self.rod_target_x_px = float(
            _cfg(
                self.config,
                "ORBIT_ROD_TARGET_X_PX",
                self.config.TARGET_CENTER_X_PX,
            )
        )
        self.rod_target_y_px = float(
            _cfg(
                self.config,
                "ORBIT_ROD_TARGET_Y_PX",
                self.config.ORBIT_ROD_TARGET_Y_PX,
            )
        )
        self.phase_elapsed_s = 0.0
        self.last_heading_error = None
        self.loss_elapsed_s = 0.0
        self.last_command = (0.0, 0.0, 0.0)
        self.class_id = 0
        self.entry_tof_mm = None
        self.control_tof_mm = None
        self.entry_center_radius_mm = None
        self.control_center_radius_mm = None
        self.push_ready_elapsed_s = 0.0
        self._reset_pids()

    def start_absolute(
        self,
        current_heading_rad,
        target_heading_rad,
        direction=None,
        orbit_radius_mm=None,
        orbit_target_y=None,
        class_id=0,
        rod_target_x_px=None,
        rod_target_y_px=None,
    ):
        current = float(current_heading_rad)
        target = normalize_angle(float(target_heading_rad))
        if direction is None:
            self.direction = optimize_orbit_direction(current, target)
        else:
            self.direction = (
                "right" if float(direction) >= 0.0 else "left"
            )
        self.target_heading_rad = target
        self.orbit_radius_mm = float(
            self.config.ORBIT_MIN_RADIUS_MM
            if orbit_radius_mm is None
            else orbit_radius_mm
        )
        self.orbit_target_y = float(
            self.config.ORBIT_ROD_TARGET_Y_PX
            if orbit_target_y is None
            else orbit_target_y
        )
        self.class_id = int(class_id)
        self.rod_target_x_px = float(
            _cfg(
                self.config,
                "ORBIT_ROD_TARGET_X_PX",
                self.config.TARGET_CENTER_X_PX,
            )
            if rod_target_x_px is None
            else rod_target_x_px
        )
        self.rod_target_y_px = float(
            _cfg(
                self.config,
                "ORBIT_ROD_TARGET_Y_PX",
                self.config.ORBIT_ROD_TARGET_Y_PX,
            )
            if rod_target_y_px is None
            else rod_target_y_px
        )
        self.phase = PHASE_ORBITING
        self.phase_elapsed_s = 0.0
        self.last_heading_error = None
        self.loss_elapsed_s = 0.0
        self.last_command = (0.0, 0.0, 0.0)
        self.push_ready_elapsed_s = 0.0
        self._reset_pids()
        self.active = True

    def start_from_approach(
        self,
        current_heading_rad,
        target_heading_rad,
        measured_tof_mm,
        orbit_target_y,
        class_id=0,
    ):
        """冻结入轨测距，并在过近时以释放距离作为控制半径。"""
        entry_mm = float(measured_tof_mm)
        control_mm = (
            self.config.TOF_EMERGENCY_RELEASE_MM
            if entry_mm <= self.config.TOF_EMERGENCY_MM
            else entry_mm
        )
        self.entry_tof_mm = entry_mm
        self.control_tof_mm = control_mm
        self.entry_center_radius_mm = (
            entry_mm + self.config.TOF_CENTER_OFFSET_MM
        )
        self.control_center_radius_mm = (
            control_mm + self.config.TOF_CENTER_OFFSET_MM
        )
        self.start_absolute(
            current_heading_rad,
            target_heading_rad,
            orbit_radius_mm=control_mm,
            orbit_target_y=orbit_target_y,
            class_id=class_id,
        )

    def _target_data(self, target):
        found = target is not None and bool(
            _target_value(
                target,
                "found",
                _target_value(target, "target_found", False),
            )
        )
        if not found:
            return None
        x = float(
            _target_value(
                target,
                "x",
                _target_value(target, "target_x", 0.0),
            )
        )
        y = float(
            _target_value(
                target,
                "y",
                _target_value(target, "target_y", 0.0),
            )
        )
        class_id = int(
            _target_value(
                target,
                "class_id",
                _target_value(target, "target_class_id", self.class_id),
            )
            or 0
        )
        if not finite(x) or not finite(y):
            return False
        return x, y, class_id

    def _valid_tof(self, distance_mm):
        if distance_mm is None:
            return False
        distance = float(distance_mm)
        return (
            finite(distance)
            and self.config.TOF_VALID_MIN_MM
            <= distance
            <= self.config.TOF_VALID_MAX_MM
        )

    def _heading_derivative(self, error, dt):
        derivative = 0.0
        if self.last_heading_error is not None:
            derivative = normalize_angle(
                error - self.last_heading_error
            ) / dt
        self.last_heading_error = error
        return derivative

    def _enter_phase(self, phase):
        self.phase = phase
        self.phase_elapsed_s = 0.0
        self.last_heading_error = None
        self._reset_pids()

    def _apply_align_stiction_compensation(self, w, heading_error):
        """Optionally raise a tiny final-heading command above wheel deadband."""
        minimum_w = float(
            _cfg(self.config, "ORBIT_ALIGN_MIN_W_RAD_S", 0.0)
        )
        if minimum_w <= 0.0:
            return w
        error_threshold = float(
            _cfg(
                self.config,
                "ORBIT_ALIGN_MIN_W_ERROR_RAD",
                self.config.ORBIT_STOP_ERROR_RAD,
            )
        )
        if (
            abs(heading_error) > error_threshold
            and abs(w) < minimum_w
        ):
            return minimum_w if heading_error > 0.0 else -minimum_w
        return w

    def step(
        self,
        target,
        tof_distance_mm,
        heading_rad,
        yaw_rate_rad_s=0.0,
        dt=0.02,
    ):
        del yaw_rate_rad_s
        if not self.active:
            return MotionStep.stop("orbit_not_started", failed=True)
        dt = clamp(float(dt), 0.001, 0.1)
        self.phase_elapsed_s += dt
        heading = float(heading_rad)
        heading_error = normalize_angle(
            self.target_heading_rad - heading
        )
        target_data = self._target_data(target)
        if target_data is False:
            self.reset()
            return MotionStep.stop("invalid_target", failed=True)

        # 默认维持原有行为：视觉丢失立即失败。独立测试可设置
        # TARGET_LOSS_DECAY_S，在丢失后按上一条命令线性渐停。
        if target_data is None:
            loss_decay_s = float(
                _cfg(self.config, "TARGET_LOSS_DECAY_S", 0.0)
            )
            if loss_decay_s > 0.0:
                self.loss_elapsed_s += dt
                if self.loss_elapsed_s <= loss_decay_s:
                    decay = 1.0 - self.loss_elapsed_s / loss_decay_s
                    command = tuple(
                        value * decay for value in self.last_command
                    )
                    return MotionStep(
                        command,
                        reason="target_lost_decelerating",
                        debug={"loss_elapsed_s": self.loss_elapsed_s},
                    )
            self.reset()
            return MotionStep.stop("spin_search", failed=True)

        self.loss_elapsed_s = 0.0

        tof_found = self._valid_tof(tof_distance_mm)
        distance = (
            float(tof_distance_mm) if tof_found else 0.0
        )
        target_x, target_y, class_id = target_data
        self.class_id = class_id

        if self.phase == PHASE_ORBITING:
            if (
                abs(heading_error)
                <= self.config.ORBIT_ENTER_ALIGN_ERROR_RAD
            ):
                self._enter_phase(PHASE_ALIGN)
                return MotionStep.stop(
                    "orbit_enter_align",
                    debug={"phase": self.phase},
                )
            command = calc_orbit_command(
                target_x,
                target_y,
                tof_found,
                distance,
                self.orbit_radius_mm,
                self.orbit_target_y,
                self.camera_turn_pid,
                self.tof_pid,
                self.y_pid,
                dt,
                self.config,
                heading_error=heading_error,
                direction=self.direction,
            )
        elif self.phase == PHASE_ALIGN:
            # 绕行已完成。此处不再把目标拉回画面中心，而是横移车体，
            # 让物体落到斜推杆正前方的标定像素。
            x_error = target_x - self.rod_target_x_px
            heading_ok = (
                abs(heading_error)
                <= self.config.ORBIT_STOP_ERROR_RAD
            )
            x_ok = (
                abs(x_error)
                <= self.config.ORBIT_STOP_X_ERROR_PX
            )
            timed_out = (
                self.phase_elapsed_s
                >= self.config.ORBIT_ALIGN_TIMEOUT_S
            )
            if heading_ok and x_ok:
                self._enter_phase(PHASE_CLOSE_IN)
                return MotionStep.stop(
                    "orbit_enter_close_in",
                    debug={
                        "phase": self.phase,
                        "rod_target_x_px": self.rod_target_x_px,
                        "rod_target_y_px": self.rod_target_y_px,
                    },
                )
            # 连续保持测试不应在把目标从画面中心移到最终对准点的过程中
            # 因 1 秒任务超时而失败；主任务默认仍保持原超时行为。
            if timed_out and not getattr(
                self.config, "CONTINUOUS_HOLD", False
            ):
                self.reset()
                return MotionStep.stop(
                    "orbit_align_timeout",
                    failed=True,
                    debug={
                        "x_error_px": x_error,
                        "heading_error_rad": heading_error,
                    },
                )

            derivative = self._heading_derivative(
                heading_error,
                dt,
            )
            w = (
                self.config.ORBIT_ALIGN_KP * heading_error
                + self.config.ORBIT_ALIGN_KD * derivative
            )
            w = clamp(
                w,
                -self.config.ORBIT_ALIGN_MAX_W_RAD_S,
                self.config.ORBIT_ALIGN_MAX_W_RAD_S,
            )
            w = self._apply_align_stiction_compensation(
                w, heading_error
            )
            if x_ok:
                self.x_pid.reset()
                vx = 0.0
            else:
                vx = self.x_pid.update(x_error, dt)
                if abs(x_error) <= 12.0:
                    vx *= 0.4
            # 横向对位期间不改变与物体的前后距离。
            command = (vx, 0.0, w)
        else:
            derivative = self._heading_derivative(
                heading_error,
                dt,
            )
            w = (
                self.config.ORBIT_ALIGN_KP * heading_error
                + self.config.ORBIT_ALIGN_KD * derivative
            )
            w = clamp(
                w,
                -self.config.ORBIT_ALIGN_MAX_W_RAD_S,
                self.config.ORBIT_ALIGN_MAX_W_RAD_S,
            )
            w = self._apply_align_stiction_compensation(
                w, heading_error
            )
            # 贴近阶段以物体在画面中的杆前目标点为闭环目标：横移调 X，
            # 前后移动调 Y；航向仍只由 IMU 的目标航向控制。
            x_error = target_x - self.rod_target_x_px
            x_ok = (
                abs(x_error)
                <= self.config.ORBIT_FINAL_ALIGN_X_ERROR_PX
            )
            if x_ok:
                self.x_pid.reset()
                vx = 0.0
            else:
                vx = self.x_pid.update(x_error, dt)
                if abs(x_error) <= 15.0:
                    vx *= 0.7

            y_error = self.rod_target_y_px - target_y
            y_ok = (
                abs(y_error)
                <= self.config.ORBIT_FINAL_ALIGN_Y_ERROR_PX
            )
            if y_ok:
                self.y_pid.reset()
                vy = 0.0
            else:
                vy = self.y_pid.update(y_error, dt)

            # ToF 只防止继续向前顶得过近；允许后退恢复安全距离，且不作为完成条件。
            stop_distance = (
                self.config.ORBIT_CLOSE_IN_TENNIS_STOP_MM
                if class_id == 3
                else self.config.ORBIT_CLOSE_IN_STOP_MM
            )
            too_close = tof_found and distance <= stop_distance
            if too_close and vy > 0.0:
                vy = 0.0

            timed_out = (
                self.phase_elapsed_s
                >= self.config.ORBIT_CLOSE_IN_TIMEOUT_S
            )
            heading_ok = (
                abs(heading_error)
                <= self.config.ORBIT_STOP_ERROR_RAD
            )
            # 默认完成后退出；独立测试可开启 CONTINUOUS_HOLD，保留本阶段的
            # 原有 X/Y/航向 PID 持续保持最终对准，不影响主任务默认行为。
            continuous_hold = getattr(
                self.config, "CONTINUOUS_HOLD", False
            )
            if heading_ok and x_ok and y_ok:
                self.push_ready_elapsed_s += dt
            else:
                self.push_ready_elapsed_s = 0.0
            if (
                self.push_ready_elapsed_s + 1.0e-9
                >= self.config.PUSH_READY_STABLE_S
            ):
                self.active = False
                return MotionStep.stop(
                    "orbit_ready_for_push",
                    done=True,
                    debug={
                        "phase": PHASE_CLOSE_IN,
                        "rod_target_x_px": self.rod_target_x_px,
                        "rod_target_y_px": self.rod_target_y_px,
                        "tof_too_close": too_close,
                        "ready_elapsed_s": self.push_ready_elapsed_s,
                    },
                )
            if timed_out and not continuous_hold:
                self.reset()
                return MotionStep.stop(
                    "orbit_close_in_timeout",
                    failed=True,
                    debug={
                        "x_error_px": x_error,
                        "y_error_px": y_error,
                        "heading_error_rad": heading_error,
                        "tof_too_close": too_close,
                    },
                )
            command = (vx, vy, w)

        vx, vy, w = command
        if (
            tof_found
            and distance <= self.config.TOF_EMERGENCY_MM
        ):
            vy = min(
                vy,
                -self.config.TOF_EMERGENCY_RETREAT_SPEED_CM_S,
            )
        vx, vy = limit_vector(
            vx,
            vy,
            self.config.MAX_XY_SPEED_CM_S,
        )
        command = (vx, vy, w)
        if not all(finite(value) for value in command):
            self.reset()
            return MotionStep.stop("non_finite_command", failed=True)
        self.last_command = command
        return MotionStep(
            command,
            debug={
                "phase": self.phase,
                "heading_error_rad": heading_error,
                "direction": self.direction,
                "orbit_radius_mm": self.orbit_radius_mm,
                "rod_target_x_px": self.rod_target_x_px,
                "rod_x_error_px": target_x - self.rod_target_x_px,
                "rod_target_y_px": self.rod_target_y_px,
                "rod_y_error_px": self.rod_target_y_px - target_y,
            },
        )

"""car141929 接近逻辑的当前底盘接口版本。

控制公式、PID结构、目标丢失衰减、分类停止距离和绕行交接条件均来自
car141929/approach.py；只有输入输出接口和线速度比例做了适配。
"""

from control import MotionStep, PIDController, clamp, finite, limit_vector


def _target_value(target, key, default=None):
    if isinstance(target, dict):
        return target.get(key, default)
    return getattr(target, key, default)


def _legacy_config(config):
    return {
        "target_center_x": config.TARGET_CENTER_X_PX,
        "stop_y_threshold": config.STOP_Y_THRESHOLD_PX,
        "approach_speed": config.APPROACH_SPEED_CM_S,
        "approach_y_slow_start": config.APPROACH_Y_SLOW_START_PX,
        "slow_forward_x_error": config.SLOW_FORWARD_X_ERROR_PX,
        "min_approach_speed": config.MIN_APPROACH_SPEED_CM_S,
        "tennis_min_approach_speed": (
            config.TENNIS_MIN_APPROACH_SPEED_CM_S
        ),
        "tennis_approach_y_slow_start": (
            config.TENNIS_APPROACH_Y_SLOW_START_PX
        ),
        "approach_tof_profile_enabled": True,
        "approach_tof_slow_start_mm": config.TOF_SLOW_START_MM,
        "stop_distance_mm": config.STOP_DISTANCE_MM,
        "tennis_approach_stop_distance_mm": (
            config.TENNIS_STOP_DISTANCE_MM
        ),
        "orbit_min_radius_mm": config.ORBIT_MIN_RADIUS_MM,
    }


def _min_approach_speed(config, obj_class):
    if obj_class == 3:
        return config.get(
            "tennis_min_approach_speed",
            config["min_approach_speed"],
        )
    return config["min_approach_speed"]


def calc_visual_approach_vy(
    error_x,
    target_y,
    config,
    obj_class=0,
):
    approach_speed = config["approach_speed"]
    min_speed = _min_approach_speed(config, obj_class)
    slow_x = max(config["slow_forward_x_error"], 1)

    if abs(error_x) >= slow_x:
        vy = min_speed
    else:
        error_ratio = abs(error_x) / slow_x
        vy = approach_speed - (
            approach_speed - min_speed
        ) * error_ratio

    approach_y_slow_start = config.get(
        "approach_y_slow_start",
        120,
    )
    if obj_class == 3:
        approach_y_slow_start = config.get(
            "tennis_approach_y_slow_start",
            approach_y_slow_start,
        )

    if target_y >= approach_y_slow_start:
        y_span = max(
            config["stop_y_threshold"] - approach_y_slow_start,
            1,
        )
        y_ratio = clamp(
            (target_y - approach_y_slow_start) / y_span,
            0.0,
            1.0,
        )
        vy = vy - (vy - min_speed) * y_ratio
    return vy


def calc_tof_approach_vy(
    tof_found,
    tof_distance_mm,
    config,
    target_stop_dist,
    obj_class=0,
):
    approach_speed = config["approach_speed"]
    min_speed = _min_approach_speed(config, obj_class)
    stop_mm = target_stop_dist

    if (
        not config.get("approach_tof_profile_enabled", True)
        or not tof_found
    ):
        return approach_speed
    if tof_distance_mm <= stop_mm:
        return 0.0

    slow_start_mm = max(
        config.get("approach_tof_slow_start_mm", 360.0),
        stop_mm + 1.0,
    )
    if tof_distance_mm >= slow_start_mm:
        return approach_speed

    distance_ratio = clamp(
        (tof_distance_mm - stop_mm) / (slow_start_mm - stop_mm),
        0.0,
        1.0,
    )
    return min_speed + (
        approach_speed - min_speed
    ) * distance_ratio


def calc_approach_command(
    target_x,
    target_y,
    approach_w_pid,
    dt,
    config,
    tof_found=False,
    tof_distance_mm=0.0,
    target_stop_dist=None,
    obj_class=0,
):
    if target_stop_dist is None:
        target_stop_dist = config["stop_distance_mm"]

    error_x = target_x - config["target_center_x"]
    vy = calc_visual_approach_vy(
        error_x,
        target_y,
        config,
        obj_class,
    )
    vy_tof = calc_tof_approach_vy(
        tof_found,
        tof_distance_mm,
        config,
        target_stop_dist,
        obj_class,
    )
    vy = min(vy, vy_tof)
    w = approach_w_pid.update(-error_x, dt)
    return 0.0, vy, w


def resolve_orbit_radius_mm(measured_distance_mm, config):
    min_radius_mm = config.get(
        "orbit_min_radius_mm",
        config["stop_distance_mm"],
    )
    return max(float(measured_distance_mm), float(min_radius_mm))


class ApproachController:
    """旧 approach 状态逻辑在当前 MotionStep 接口下的适配器。"""

    def __init__(self, config):
        self.config = config
        self.legacy_config = _legacy_config(config)
        self.approach_w_pid = PIDController(
            config.PID_APPROACH_W_KP,
            config.PID_APPROACH_W_KI,
            config.PID_APPROACH_W_KD,
            output_limit=config.PID_APPROACH_W_OUTPUT_LIMIT,
            integral_limit=config.PID_APPROACH_W_I_LIMIT,
        )
        self.reset()

    def reset(self):
        self.approach_w_pid.reset()
        self.loss_elapsed_s = 0.0
        self.align_elapsed_s = 0.0
        self.last_command = (0.0, 0.0, 0.0)

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
        target_x = float(
            _target_value(
                target,
                "x",
                _target_value(target, "target_x", 0.0),
            )
        )
        target_y = float(
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
                _target_value(target, "target_class_id", 0),
            )
            or 0
        )
        if not finite(target_x) or not finite(target_y):
            return False
        return target_x, target_y, class_id

    def step(
        self,
        target,
        tof_distance_mm,
        dt=0.02,
    ):
        dt = clamp(float(dt), 0.001, 0.1)
        target_data = self._target_data(target)
        if target_data is False:
            self.reset()
            return MotionStep.stop("invalid_target", failed=True)
        if target_data is None:
            self.loss_elapsed_s += dt
            if self.loss_elapsed_s > self.config.TARGET_LOSS_DECAY_S:
                self.reset()
                return MotionStep.stop("spin_search", failed=True)
            decay = 1.0 - (
                self.loss_elapsed_s / self.config.TARGET_LOSS_DECAY_S
            )
            command = tuple(value * decay for value in self.last_command)
            return MotionStep(
                command,
                reason="target_lost_decelerating",
                debug={"loss_elapsed_s": self.loss_elapsed_s},
            )

        self.loss_elapsed_s = 0.0
        target_x, target_y, obj_class = target_data
        tof_found = self._valid_tof(tof_distance_mm)
        target_stop_dist = (
            self.config.TENNIS_STOP_DISTANCE_MM
            if obj_class == 3
            else self.config.STOP_DISTANCE_MM
        )
        # ball(class 3) 的 y>120 可提前结束靠近。
        # 独立测试可通过 VISUAL_STOP_ENABLED=False 关闭此规则，只以 ToF 交接。
        visual_stop_enabled = getattr(
            self.config, "VISUAL_STOP_ENABLED", True
        )
        visual_stop_thresholds = getattr(
            self.config,
            "VISUAL_STOP_Y_THRESHOLD_BY_CLASS",
            {3: 120.0},
        )
        visual_stop_y = visual_stop_thresholds.get(obj_class)
        visual_reached = (
            visual_stop_enabled
            and visual_stop_y is not None
            and target_y > float(visual_stop_y)
        )
        reached = (
            tof_found
            and float(tof_distance_mm) <= target_stop_dist
        ) or visual_reached

        if reached:
            error_x = (
                target_x - self.config.TARGET_CENTER_X_PX
            )
            aligned = (
                abs(error_x)
                <= self.config.TARGET_ALIGN_ERROR_PX
            )
            self.align_elapsed_s += dt
            timed_out = (
                self.align_elapsed_s
                > self.config.ALIGN_TIMEOUT_S
            )
            if not aligned and not timed_out:
                w = self.approach_w_pid.update(-error_x, dt)
                self.last_command = (0.0, 0.0, w)
                return MotionStep(
                    self.last_command,
                    reason="approach_final_align",
                    debug={"x_error_px": error_x},
                )

            measured = (
                float(tof_distance_mm)
                if tof_found
                else target_stop_dist
            )
            orbit_radius = resolve_orbit_radius_mm(
                measured,
                self.legacy_config,
            )
            debug = {
                "x_error_px": error_x,
                "orbit_radius_mm": orbit_radius,
                "orbit_target_y_px": target_y,
                "class_id": obj_class,
            }
            self.reset()
            return MotionStep.stop(
                "approach_reached",
                done=True,
                debug=debug,
            )

        self.align_elapsed_s = 0.0
        command = calc_approach_command(
            target_x,
            target_y,
            self.approach_w_pid,
            dt,
            self.legacy_config,
            tof_found,
            float(tof_distance_mm) if tof_found else 0.0,
            target_stop_dist,
            obj_class,
        )
        vx, vy, w = command
        if not tof_found:
            if target_y >= self.config.TOF_FALLBACK_STOP_Y_PX and vy > 0.0:
                vy = 0.0
            else:
                vy = min(vy, self.config.TOF_FALLBACK_SPEED_CM_S)
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
                "x_error_px": (
                    target_x - self.config.TARGET_CENTER_X_PX
                ),
                "target_stop_distance_mm": target_stop_dist,
                "class_id": obj_class,
            },
        )

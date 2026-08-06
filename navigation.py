"""主车坐标直线巡航与原地转向。

世界航向角遵循 odometry.py：0 rad 时车头指向世界 +X，逆时针为正。
车体系遵循 motor.py：+vx 向右、+vy 向前、+w 逆时针。
"""

import math

from control import (
    MotionStep,
    clamp,
    finite,
    limit_vector,
    normalize_angle,
)


def world_to_body(world_vx, world_vy, heading_rad):
    """将世界速度转换到 (+右, +前) 车体系。"""
    sin_heading = math.sin(heading_rad)
    cos_heading = math.cos(heading_rad)
    body_vx = world_vx * sin_heading - world_vy * cos_heading
    body_vy = world_vx * cos_heading + world_vy * sin_heading
    return body_vx, body_vy


def _waypoint_value(waypoint, key, index, default=None):
    if isinstance(waypoint, dict):
        return waypoint.get(key, default)
    if len(waypoint) > index:
        return waypoint[index]
    return default


class CoordinatePatrolController:
    """沿路点间的世界坐标直线运动，一次只负责当前一段。

    到达路点时返回 done=True。调用方完成需要的原地转向后，调用 advance()
    进入下一段，避免“到点、换向、继续平移”在同一个控制周期中互相覆盖。
    """

    def __init__(self, waypoints, config):
        self.config = config
        self.waypoints = list(waypoints)
        self.index = 0
        self.segment_start = None
        self.active = False

    def reset(self, x_cm, y_cm, index=0):
        self.index = int(index)
        self.segment_start = (float(x_cm), float(y_cm))
        self.active = bool(self.waypoints)

    def current_waypoint(self):
        if not self.waypoints:
            return None
        return self.waypoints[self.index % len(self.waypoints)]

    def advance(self, x_cm=None, y_cm=None):
        if not self.waypoints:
            self.active = False
            return
        reached = self.current_waypoint()
        reached_x = float(_waypoint_value(reached, "x", 0))
        reached_y = float(_waypoint_value(reached, "y", 1))
        self.segment_start = (
            reached_x if x_cm is None else float(x_cm),
            reached_y if y_cm is None else float(y_cm),
        )
        self.index = (self.index + 1) % len(self.waypoints)
        self.active = True

    def target_heading_rad(self, pose=None):
        waypoint = self.current_waypoint()
        if waypoint is None:
            return None
        heading_deg = _waypoint_value(
            waypoint, "heading_deg", 2, None
        )
        if heading_deg is not None:
            return math.radians(float(heading_deg))
        if pose is None:
            return None
        target_x = float(_waypoint_value(waypoint, "x", 0))
        target_y = float(_waypoint_value(waypoint, "y", 1))
        return math.atan2(target_y - pose[1], target_x - pose[0])

    def _along_speed(self, remaining_cm):
        cfg = self.config
        if remaining_cm > cfg.PATH_FAST_DISTANCE_CM:
            return cfg.PATH_FAST_SPEED_CM_S, "fast"
        if remaining_cm > cfg.PATH_SLOW_DISTANCE_CM:
            return cfg.PATH_MID_SPEED_CM_S, "mid"
        return cfg.PATH_SLOW_SPEED_CM_S, "slow"

    def step(self, pose, yaw_rate_rad_s=0.0):
        if not self.active or not self.waypoints:
            return MotionStep.stop("no_path", done=True)
        if self.segment_start is None:
            self.segment_start = (float(pose[0]), float(pose[1]))

        x_cm = float(pose[0])
        y_cm = float(pose[1])
        heading = float(pose[2])
        waypoint = self.current_waypoint()
        target_x = float(_waypoint_value(waypoint, "x", 0))
        target_y = float(_waypoint_value(waypoint, "y", 1))
        start_x, start_y = self.segment_start

        path_x = target_x - start_x
        path_y = target_y - start_y
        path_length = math.sqrt(path_x * path_x + path_y * path_y)
        target_dx = target_x - x_cm
        target_dy = target_y - y_cm
        direct_distance = math.sqrt(
            target_dx * target_dx + target_dy * target_dy
        )

        if path_length < 1.0e-6:
            return MotionStep.stop(
                "waypoint_reached",
                done=True,
                debug={"index": self.index, "distance_cm": direct_distance},
            )

        unit_x = path_x / path_length
        unit_y = path_y / path_length
        normal_x = -unit_y
        normal_y = unit_x
        relative_x = x_cm - start_x
        relative_y = y_cm - start_y
        along_position = relative_x * unit_x + relative_y * unit_y
        remaining = path_length - along_position
        cross_error = relative_x * normal_x + relative_y * normal_y

        reached = (
            direct_distance <= self.config.POSITION_TOLERANCE_CM
            or (
                remaining <= self.config.POSITION_TOLERANCE_CM
                and abs(cross_error)
                <= self.config.CROSS_TRACK_TOLERANCE_CM
            )
        )
        if reached:
            return MotionStep.stop(
                "waypoint_reached",
                done=True,
                debug={
                    "index": self.index,
                    "distance_cm": direct_distance,
                    "cross_error_cm": cross_error,
                },
            )

        along_speed, profile = self._along_speed(max(remaining, 0.0))
        cross_speed = clamp(
            -self.config.PATH_CROSS_KP * cross_error,
            -self.config.PATH_CROSS_MAX_SPEED_CM_S,
            self.config.PATH_CROSS_MAX_SPEED_CM_S,
        )
        world_vx = along_speed * unit_x + cross_speed * normal_x
        world_vy = along_speed * unit_y + cross_speed * normal_y
        world_vx, world_vy = limit_vector(
            world_vx, world_vy, self.config.PATH_MAX_SPEED_CM_S
        )
        body_vx, body_vy = world_to_body(world_vx, world_vy, heading)

        desired_heading = self.target_heading_rad(pose)
        heading_error = normalize_angle(desired_heading - heading)
        if (
            abs(heading_error)
            <= self.config.TRANSLATE_HEADING_DEADBAND_RAD
        ):
            w = 0.0
        else:
            w = (
                self.config.TRANSLATE_HEADING_KP * heading_error
                - self.config.TRANSLATE_HEADING_KD
                * float(yaw_rate_rad_s)
            )
            w = clamp(
                w,
                -self.config.TRANSLATE_MAX_W_RAD_S,
                self.config.TRANSLATE_MAX_W_RAD_S,
            )

        if not all(finite(value) for value in (body_vx, body_vy, w)):
            return MotionStep.stop("non_finite_command", failed=True)
        return MotionStep(
            (body_vx, body_vy, w),
            debug={
                "index": self.index,
                "profile": profile,
                "remaining_cm": remaining,
                "cross_error_cm": cross_error,
                "heading_error_rad": heading_error,
            },
        )


class HeadingTurnController:
    """原地转到世界绝对航向，并要求角度和角速度稳定后完成。"""

    def __init__(self, config):
        self.config = config
        self.target_heading_rad = None
        self.stable_time_s = 0.0

    def start(self, target_heading_rad):
        self.target_heading_rad = normalize_angle(float(target_heading_rad))
        self.stable_time_s = 0.0

    def reset(self):
        self.target_heading_rad = None
        self.stable_time_s = 0.0

    def step(self, heading_rad, yaw_rate_rad_s, dt):
        if self.target_heading_rad is None:
            return MotionStep.stop("turn_not_started", failed=True)
        dt = clamp(float(dt), 0.001, 0.1)
        error = normalize_angle(
            self.target_heading_rad - float(heading_rad)
        )
        yaw_rate = float(yaw_rate_rad_s)

        inside_angle = abs(error) <= self.config.TURN_TOLERANCE_RAD
        inside_rate = (
            abs(yaw_rate)
            <= self.config.TURN_YAW_RATE_TOLERANCE_RAD_S
        )
        if inside_angle and inside_rate:
            self.stable_time_s += dt
            if self.stable_time_s >= self.config.TURN_STABLE_TIME_S:
                return MotionStep.stop(
                    "heading_reached",
                    done=True,
                    debug={"heading_error_rad": error},
                )
            return MotionStep.stop(
                "heading_settling",
                debug={"heading_error_rad": error},
            )

        self.stable_time_s = 0.0
        abs_error = abs(error)
        if abs_error > self.config.TURN_FAST_ERROR_RAD:
            base_w = self.config.TURN_FAST_W_RAD_S
            profile = "fast"
        elif abs_error > self.config.TURN_MID_ERROR_RAD:
            base_w = self.config.TURN_MID_W_RAD_S
            profile = "mid"
        else:
            base_w = self.config.TURN_SLOW_W_RAD_S
            profile = "slow"
        direction = 1.0 if error > 0.0 else -1.0
        w = direction * base_w - self.config.TURN_DAMPING_KD * yaw_rate
        return MotionStep(
            (0.0, 0.0, w),
            debug={"profile": profile, "heading_error_rad": error},
        )


class ClockwiseTurnController:
    """Turn a prescribed amount clockwise; never choose the shorter path."""

    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        self.active = False
        self.last_heading_rad = None
        self.progress_rad = 0.0
        self.stable_time_s = 0.0

    def start(self, heading_rad):
        self.active = True
        self.last_heading_rad = float(heading_rad)
        self.progress_rad = 0.0
        self.stable_time_s = 0.0

    def step(self, heading_rad, yaw_rate_rad_s, dt, angle_rad=math.pi):
        if not self.active:
            return MotionStep.stop("clockwise_turn_not_started", failed=True)
        dt = clamp(float(dt), 0.001, 0.1)
        heading_delta = normalize_angle(float(heading_rad) - self.last_heading_rad)
        if heading_delta < 0.0:
            self.progress_rad += -heading_delta
        self.last_heading_rad = float(heading_rad)
        remaining = float(angle_rad) - self.progress_rad

        if remaining <= self.config.TURN_TOLERANCE_RAD:
            if abs(float(yaw_rate_rad_s)) <= self.config.TURN_YAW_RATE_TOLERANCE_RAD_S:
                self.stable_time_s += dt
                if self.stable_time_s >= self.config.TURN_STABLE_TIME_S:
                    self.active = False
                    return MotionStep.stop(
                        "clockwise_180_reached",
                        done=True,
                        debug={"turn_progress_rad": self.progress_rad},
                    )
                return MotionStep.stop(
                    "clockwise_180_settling",
                    debug={"turn_progress_rad": self.progress_rad},
                )
            self.stable_time_s = 0.0

        if remaining > self.config.TURN_FAST_ERROR_RAD:
            speed, profile = self.config.TURN_FAST_W_RAD_S, "fast"
        elif remaining > self.config.TURN_MID_ERROR_RAD:
            speed, profile = self.config.TURN_MID_W_RAD_S, "mid"
        else:
            speed, profile = self.config.TURN_SLOW_W_RAD_S, "slow"
        w = -speed - self.config.TURN_DAMPING_KD * float(yaw_rate_rad_s)
        return MotionStep(
            (0.0, 0.0, w),
            reason="clockwise_180_turning",
            debug={
                "profile": profile,
                "turn_progress_rad": self.progress_rad,
                "turn_remaining_rad": max(0.0, remaining),
            },
        )


class CounterclockwiseTurnController:
    """Turn a prescribed amount counterclockwise; never choose the shorter path."""

    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        self.active = False
        self.last_heading_rad = None
        self.progress_rad = 0.0
        self.stable_time_s = 0.0

    def start(self, heading_rad):
        self.active = True
        self.last_heading_rad = float(heading_rad)
        self.progress_rad = 0.0
        self.stable_time_s = 0.0

    def step(self, heading_rad, yaw_rate_rad_s, dt, angle_rad=math.pi):
        if not self.active:
            return MotionStep.stop(
                "counterclockwise_turn_not_started", failed=True
            )
        dt = clamp(float(dt), 0.001, 0.1)
        heading_delta = normalize_angle(float(heading_rad) - self.last_heading_rad)
        if heading_delta > 0.0:
            self.progress_rad += heading_delta
        self.last_heading_rad = float(heading_rad)
        remaining = float(angle_rad) - self.progress_rad

        if remaining <= self.config.TURN_TOLERANCE_RAD:
            if abs(float(yaw_rate_rad_s)) <= self.config.TURN_YAW_RATE_TOLERANCE_RAD_S:
                self.stable_time_s += dt
                if self.stable_time_s >= self.config.TURN_STABLE_TIME_S:
                    self.active = False
                    return MotionStep.stop(
                        "counterclockwise_180_reached",
                        done=True,
                        debug={"turn_progress_rad": self.progress_rad},
                    )
                return MotionStep.stop(
                    "counterclockwise_180_settling",
                    debug={"turn_progress_rad": self.progress_rad},
                )
            self.stable_time_s = 0.0

        if remaining > self.config.TURN_FAST_ERROR_RAD:
            speed, profile = self.config.TURN_FAST_W_RAD_S, "fast"
        elif remaining > self.config.TURN_MID_ERROR_RAD:
            speed, profile = self.config.TURN_MID_W_RAD_S, "mid"
        else:
            speed, profile = self.config.TURN_SLOW_W_RAD_S, "slow"
        w = speed - self.config.TURN_DAMPING_KD * float(yaw_rate_rad_s)
        return MotionStep(
            (0.0, 0.0, w),
            reason="counterclockwise_180_turning",
            debug={
                "profile": profile,
                "turn_progress_rad": self.progress_rad,
                "turn_remaining_rad": max(0.0, remaining),
            },
        )

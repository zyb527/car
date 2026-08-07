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


DEFAULT_TURN_MAX_TIME_S = 8.0


def _turn_max_time_s(config):
    """Return the shared safety limit for navigation-only in-place turns."""
    return max(
        0.001,
        float(getattr(config, "TURN_MAX_TIME_S", DEFAULT_TURN_MAX_TIME_S)),
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
    """原地转到世界绝对航向，并在角度和角速度均到位后完成。"""

    def __init__(self, config):
        self.config = config
        self.max_turn_time_s = _turn_max_time_s(config)
        self.target_heading_rad = None
        self.elapsed_s = 0.0

    def start(self, target_heading_rad):
        self.target_heading_rad = normalize_angle(float(target_heading_rad))
        self.elapsed_s = 0.0

    def reset(self):
        self.target_heading_rad = None
        self.elapsed_s = 0.0

    def step(self, heading_rad, yaw_rate_rad_s, dt):
        if self.target_heading_rad is None:
            return MotionStep.stop("turn_not_started", failed=True)
        self.elapsed_s += max(0.0, float(dt))
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
            return MotionStep.stop(
                "heading_reached",
                done=True,
                debug={
                    "heading_error_rad": error,
                    "suppress_feedforward_w": True,
                },
            )

        if self.elapsed_s >= self.max_turn_time_s:
            return MotionStep.stop(
                "heading_turn_timeout",
                failed=True,
                debug={
                    "heading_error_rad": error,
                    "turn_elapsed_s": self.elapsed_s,
                    "turn_max_time_s": self.max_turn_time_s,
                },
            )

        abs_error = abs(error)
        if abs_error > self.config.TURN_FAST_ERROR_RAD:
            base_w = self.config.TURN_FAST_W_RAD_S
            profile = "fast"
        elif abs_error > self.config.TURN_MID_ERROR_RAD:
            base_w = min(
                self.config.TURN_MID_KP * abs_error,
                self.config.TURN_MID_W_RAD_S,
            )
            profile = "mid"
        else:
            base_w = self.config.TURN_SLOW_KP * abs_error
            profile = "slow"
        direction = 1.0 if error > 0.0 else -1.0
        w = direction * base_w - self.config.TURN_DAMPING_KD * yaw_rate
        return MotionStep(
            (0.0, 0.0, w),
            debug={
                "profile": profile,
                "heading_error_rad": error,
                "suppress_feedforward_w": profile == "slow",
            },
        )


class ClockwiseTurnController:
    """Turn a prescribed amount clockwise; never choose the shorter path."""

    def __init__(self, config):
        self.config = config
        self.max_turn_time_s = _turn_max_time_s(config)
        self.reset()

    def reset(self):
        self.active = False
        self.last_heading_rad = None
        self.progress_rad = 0.0
        self.elapsed_s = 0.0

    def start(self, heading_rad):
        self.active = True
        self.last_heading_rad = float(heading_rad)
        self.progress_rad = 0.0
        self.elapsed_s = 0.0

    def step(self, heading_rad, yaw_rate_rad_s, dt, angle_rad=math.pi):
        if not self.active:
            return MotionStep.stop("clockwise_turn_not_started", failed=True)
        self.elapsed_s += max(0.0, float(dt))
        heading_delta = normalize_angle(float(heading_rad) - self.last_heading_rad)
        if heading_delta < 0.0:
            self.progress_rad += -heading_delta
        self.last_heading_rad = float(heading_rad)
        remaining = float(angle_rad) - self.progress_rad

        if remaining <= self.config.TURN_TOLERANCE_RAD:
            if abs(float(yaw_rate_rad_s)) <= self.config.TURN_YAW_RATE_TOLERANCE_RAD_S:
                self.active = False
                return MotionStep.stop(
                    "clockwise_180_reached",
                    done=True,
                    debug={
                        "turn_progress_rad": self.progress_rad,
                        "suppress_feedforward_w": True,
                    },
                )

        if self.elapsed_s >= self.max_turn_time_s:
            return MotionStep.stop(
                "clockwise_turn_timeout",
                failed=True,
                debug={
                    "turn_progress_rad": self.progress_rad,
                    "turn_elapsed_s": self.elapsed_s,
                    "turn_max_time_s": self.max_turn_time_s,
                },
            )

        if remaining > self.config.TURN_FAST_ERROR_RAD:
            speed, profile = self.config.TURN_FAST_W_RAD_S, "fast"
        elif remaining > self.config.TURN_MID_ERROR_RAD:
            speed = min(
                self.config.TURN_MID_KP * remaining,
                self.config.TURN_MID_W_RAD_S,
            )
            profile = "mid"
        else:
            speed = self.config.TURN_SLOW_KP * max(0.0, remaining)
            profile = "slow"
        w = -speed - self.config.TURN_DAMPING_KD * float(yaw_rate_rad_s)
        return MotionStep(
            (0.0, 0.0, w),
            reason="clockwise_180_turning",
            debug={
                "profile": profile,
                "turn_progress_rad": self.progress_rad,
                "turn_remaining_rad": max(0.0, remaining),
                "suppress_feedforward_w": profile == "slow",
            },
        )


class CounterclockwiseTurnController:
    """Turn a prescribed amount counterclockwise; never choose the shorter path."""

    def __init__(self, config):
        self.config = config
        self.max_turn_time_s = _turn_max_time_s(config)
        self.reset()

    def reset(self):
        self.active = False
        self.last_heading_rad = None
        self.progress_rad = 0.0
        self.elapsed_s = 0.0

    def start(self, heading_rad):
        self.active = True
        self.last_heading_rad = float(heading_rad)
        self.progress_rad = 0.0
        self.elapsed_s = 0.0

    def step(self, heading_rad, yaw_rate_rad_s, dt, angle_rad=math.pi):
        if not self.active:
            return MotionStep.stop(
                "counterclockwise_turn_not_started", failed=True
            )
        self.elapsed_s += max(0.0, float(dt))
        heading_delta = normalize_angle(float(heading_rad) - self.last_heading_rad)
        if heading_delta > 0.0:
            self.progress_rad += heading_delta
        self.last_heading_rad = float(heading_rad)
        remaining = float(angle_rad) - self.progress_rad

        if remaining <= self.config.TURN_TOLERANCE_RAD:
            if abs(float(yaw_rate_rad_s)) <= self.config.TURN_YAW_RATE_TOLERANCE_RAD_S:
                self.active = False
                return MotionStep.stop(
                    "counterclockwise_180_reached",
                    done=True,
                    debug={
                        "turn_progress_rad": self.progress_rad,
                        "suppress_feedforward_w": True,
                    },
                )

        if self.elapsed_s >= self.max_turn_time_s:
            return MotionStep.stop(
                "counterclockwise_turn_timeout",
                failed=True,
                debug={
                    "turn_progress_rad": self.progress_rad,
                    "turn_elapsed_s": self.elapsed_s,
                    "turn_max_time_s": self.max_turn_time_s,
                },
            )

        if remaining > self.config.TURN_FAST_ERROR_RAD:
            speed, profile = self.config.TURN_FAST_W_RAD_S, "fast"
        elif remaining > self.config.TURN_MID_ERROR_RAD:
            speed = min(
                self.config.TURN_MID_KP * remaining,
                self.config.TURN_MID_W_RAD_S,
            )
            profile = "mid"
        else:
            speed = self.config.TURN_SLOW_KP * max(0.0, remaining)
            profile = "slow"
        w = speed - self.config.TURN_DAMPING_KD * float(yaw_rate_rad_s)
        return MotionStep(
            (0.0, 0.0, w),
            reason="counterclockwise_180_turning",
            debug={
                "profile": profile,
                "turn_progress_rad": self.progress_rad,
                "turn_remaining_rad": max(0.0, remaining),
                "suppress_feedforward_w": profile == "slow",
            },
        )


class ApproachLossSearchState:
    """Motion-only phases for the Approach-loss six-waypoint search."""

    TURN = "TURN"
    PRETURN = "PRETURN"
    NAVIGATE = "NAVIGATE"


class ApproachLossSearchController:
    """Turn once, then repeatedly patrol the nearest ordered search waypoint.

    This controller deliberately has no vision or task-state knowledge. Callers
    may interrupt any phase when they acquire a visual target.
    """

    def __init__(self, waypoints, config, turn_angle_rad, search_w_rad_s):
        self.waypoints = tuple(waypoints)
        self.config = config
        self.turn_angle_rad = float(turn_angle_rad)
        self.search_w_rad_s = abs(float(search_w_rad_s))
        self.turn = CounterclockwiseTurnController(config)
        self.nav_turn = HeadingTurnController(config)
        self.reset()

    def reset(self):
        self.active = False
        self.state = None
        self.patrol = None
        self.turn.reset()
        self.nav_turn.reset()

    def start(self, pose):
        self.reset()
        if not self.waypoints:
            return MotionStep.stop(
                "approach_search_waypoints_empty", failed=True
            )
        self.active = True
        self.state = ApproachLossSearchState.TURN
        self.turn.start(pose[2])
        return MotionStep.stop("approach_search_started")

    def _nearest_waypoint_index(self, pose):
        nearest_index = 0
        nearest_distance_sq = None
        for index, waypoint in enumerate(self.waypoints):
            dx = float(waypoint[0]) - float(pose[0])
            dy = float(waypoint[1]) - float(pose[1])
            distance_sq = dx * dx + dy * dy
            if nearest_distance_sq is None or distance_sq < nearest_distance_sq:
                nearest_index = index
                nearest_distance_sq = distance_sq
        return nearest_index

    def _start_nearest_waypoint(self, pose):
        nearest_index = self._nearest_waypoint_index(pose)
        self.patrol = CoordinatePatrolController(self.waypoints, self.config)
        self.patrol.reset(pose[0], pose[1], nearest_index)
        target_heading = self.patrol.target_heading_rad(pose)
        if target_heading is None:
            return MotionStep.stop(
                "approach_search_waypoint_heading_missing", failed=True
            )
        self.nav_turn.start(target_heading)
        self.state = ApproachLossSearchState.PRETURN
        return MotionStep.stop(
            "approach_search_full_turn_complete_starting_nearest_waypoint_turn",
            debug={
                "search_phase": self.state,
                "nearest_waypoint_index": nearest_index,
                "nearest_waypoint": self.patrol.current_waypoint(),
                "hard_stop": True,
            },
        )

    def step(self, pose, yaw_rate_rad_s, dt):
        if not self.active:
            return MotionStep.stop("approach_search_not_started", failed=True)

        if self.state == ApproachLossSearchState.TURN:
            result = self.turn.step(
                pose[2],
                yaw_rate_rad_s,
                dt,
                angle_rad=self.turn_angle_rad,
            )
            if result.failed:
                return result
            if result.done:
                return self._start_nearest_waypoint(pose)
            debug = dict(result.debug)
            debug.update({
                "search_phase": self.state,
                "search_all_classes": True,
            })
            # Keep the pre-refactor Approach search speed; the underlying turn
            # controller may still slow down near 2*pi for a stable stop.
            command_w = clamp(result.command[2], 0.0, self.search_w_rad_s)
            return MotionStep(
                (0.0, 0.0, command_w),
                reason="approach_spin_search",
                debug=debug,
            )

        if self.state == ApproachLossSearchState.PRETURN:
            result = self.nav_turn.step(pose[2], yaw_rate_rad_s, dt)
            if result.failed:
                return result
            if result.done:
                self.state = ApproachLossSearchState.NAVIGATE
                return MotionStep.stop(
                    "approach_search_heading_reached_starting_patrol",
                    debug={
                        "search_phase": self.state,
                        "suppress_feedforward_w": True,
                    },
                )
            return result

        if self.state == ApproachLossSearchState.NAVIGATE:
            if self.patrol is None:
                return MotionStep.stop(
                    "approach_search_patrol_missing", failed=True
                )
            result = self.patrol.step(pose, yaw_rate_rad_s)
            if result.failed:
                return result
            if result.done:
                reached_index = self.patrol.index
                self.patrol.advance(pose[0], pose[1])
                next_heading = self.patrol.target_heading_rad(pose)
                if next_heading is None:
                    return MotionStep.stop(
                        "approach_search_waypoint_heading_missing", failed=True
                    )
                self.nav_turn.start(next_heading)
                self.state = ApproachLossSearchState.PRETURN
                return MotionStep.stop(
                    "approach_search_waypoint_reached_starting_next_turn",
                    debug={
                        "search_phase": self.state,
                        "reached_waypoint_index": reached_index,
                        "next_waypoint_index": self.patrol.index,
                        "next_waypoint": self.patrol.current_waypoint(),
                        "hard_stop": True,
                    },
                )
            return result

        return MotionStep.stop("unknown_approach_search_state", failed=True)


class PostPushPointSearchState:
    """Motion-only phases after a Push return waypoint is reached."""

    PRETURN = "PRETURN"
    WAIT = "WAIT"
    FORWARD = "FORWARD"
    COMPLETE = "COMPLETE"


class PostPushPointSearchController:
    """Align, wait for vision, then drive straight with a distance limit.

    Vision ownership stays with the top-level task controller so any object can
    interrupt the wait or forward phase.
    """

    def __init__(
        self,
        config,
        wait_s,
        forward_speed_cm_s,
        forward_max_distance_cm,
    ):
        self.config = config
        self.wait_s = max(0.0, float(wait_s))
        self.forward_speed_cm_s = float(forward_speed_cm_s)
        self.forward_max_distance_cm = max(
            0.0, float(forward_max_distance_cm)
        )
        self.heading_turn = HeadingTurnController(config)
        self.reset()

    def reset(self):
        self.active = False
        self.state = None
        self.target_heading_rad = None
        self.wait_elapsed_s = 0.0
        self.forward_start = None
        self.heading_turn.reset()

    def start(self, pose, heading_deg):
        self.reset()
        self.active = True
        self.state = PostPushPointSearchState.PRETURN
        self.target_heading_rad = math.radians(float(heading_deg))
        self.heading_turn.start(self.target_heading_rad)
        return MotionStep.stop("post_push_point_search_started")

    def _forward_heading_command(self, heading_rad, yaw_rate_rad_s):
        error = normalize_angle(self.target_heading_rad - float(heading_rad))
        if abs(error) <= self.config.TRANSLATE_HEADING_DEADBAND_RAD:
            return 0.0, error
        w = (
            self.config.TRANSLATE_HEADING_KP * error
            - self.config.TRANSLATE_HEADING_KD * float(yaw_rate_rad_s)
        )
        return clamp(
            w,
            -self.config.TRANSLATE_MAX_W_RAD_S,
            self.config.TRANSLATE_MAX_W_RAD_S,
        ), error

    def _forward_progress_cm(self, pose):
        dx = float(pose[0]) - self.forward_start[0]
        dy = float(pose[1]) - self.forward_start[1]
        return dx * math.cos(self.target_heading_rad) + dy * math.sin(
            self.target_heading_rad
        )

    def step(self, pose, yaw_rate_rad_s, dt):
        if not self.active:
            return MotionStep.stop(
                "post_push_point_search_not_started", failed=True
            )

        if self.state == PostPushPointSearchState.PRETURN:
            result = self.heading_turn.step(pose[2], yaw_rate_rad_s, dt)
            if result.failed:
                return result
            if result.done:
                self.state = PostPushPointSearchState.WAIT
                self.wait_elapsed_s = 0.0
                return MotionStep.stop(
                    "post_push_point_heading_reached_waiting_for_target",
                    debug={"post_push_search_phase": self.state},
                )
            return result

        if self.state == PostPushPointSearchState.WAIT:
            self.wait_elapsed_s += max(0.0, float(dt))
            if self.wait_elapsed_s >= self.wait_s:
                self.state = PostPushPointSearchState.FORWARD
                self.forward_start = (float(pose[0]), float(pose[1]))
                return MotionStep.stop(
                    "post_push_point_wait_complete_starting_forward",
                    debug={"post_push_search_phase": self.state},
                )
            return MotionStep.stop(
                "post_push_point_waiting_for_target",
                debug={
                    "post_push_search_phase": self.state,
                    "wait_elapsed_s": self.wait_elapsed_s,
                },
            )

        if self.state == PostPushPointSearchState.FORWARD:
            progress_cm = self._forward_progress_cm(pose)
            if progress_cm >= self.forward_max_distance_cm:
                self.active = False
                self.state = PostPushPointSearchState.COMPLETE
                return MotionStep.stop(
                    "post_push_point_forward_distance_complete",
                    done=True,
                    debug={
                        "post_push_search_phase": self.state,
                        "forward_progress_cm": progress_cm,
                    },
                )
            w, heading_error = self._forward_heading_command(
                pose[2], yaw_rate_rad_s
            )
            return MotionStep(
                (0.0, self.forward_speed_cm_s, w),
                reason="post_push_point_forward_search",
                debug={
                    "post_push_search_phase": self.state,
                    "forward_progress_cm": progress_cm,
                    "heading_error_rad": heading_error,
                },
            )

        return MotionStep.stop(
            "unknown_post_push_point_search_state", failed=True
        )

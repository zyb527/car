"""主车上层动作共用的轻量控制工具。"""

import math


def clamp(value, lower, upper):
    if value < lower:
        return lower
    if value > upper:
        return upper
    return value


def finite(value):
    return value == value and -1.0e30 < value < 1.0e30


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def limit_vector(x, y, maximum):
    magnitude = math.sqrt(x * x + y * y)
    if magnitude <= maximum or magnitude <= 1.0e-12:
        return x, y
    scale = maximum / magnitude
    return x * scale, y * scale


class MotionStep:
    """一次动作计算结果。

    command 始终是车体系 (vx, vy, w)。failed=True 时调用方必须停车并
    进入安全状态，不能沿用上一周期命令。
    """

    def __init__(
        self,
        command=(0.0, 0.0, 0.0),
        done=False,
        failed=False,
        reason="running",
        debug=None,
    ):
        self.command = (
            float(command[0]),
            float(command[1]),
            float(command[2]),
        )
        self.done = bool(done)
        self.failed = bool(failed)
        self.reason = reason
        self.debug = debug or {}

    @classmethod
    def stop(cls, reason, done=False, failed=False, debug=None):
        return cls((0.0, 0.0, 0.0), done, failed, reason, debug)


class PIDController:
    """带输出限幅和条件积分的 PID，兼容 MicroPython。"""

    def __init__(
        self,
        kp,
        ki=0.0,
        kd=0.0,
        output_limit=None,
        integral_limit=None,
    ):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.last_error = None

    def reset(self):
        self.integral = 0.0
        self.last_error = None

    def update(self, error, dt):
        error = float(error)
        dt = clamp(float(dt), 0.001, 0.1)
        derivative = 0.0
        if self.last_error is not None:
            derivative = (error - self.last_error) / dt

        candidate_integral = self.integral
        if self.ki != 0.0:
            candidate_integral += error * dt
            if self.integral_limit is not None:
                limit = abs(float(self.integral_limit))
                candidate_integral = clamp(
                    candidate_integral, -limit, limit
                )

        raw = (
            self.kp * error
            + self.ki * candidate_integral
            + self.kd * derivative
        )
        output = raw
        if self.output_limit is not None:
            limit = abs(float(self.output_limit))
            output = clamp(raw, -limit, limit)

        # 输出饱和且误差继续把输出推向饱和时，拒绝本次积分。
        pushing_positive = raw > output and error > 0.0
        pushing_negative = raw < output and error < 0.0
        if not pushing_positive and not pushing_negative:
            self.integral = candidate_integral
        self.last_error = error
        return output


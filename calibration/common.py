"""车载标定共用的安全、计时和日志辅助函数。"""

import math
import time

from calibration_store import ticks_ms


def ticks_diff(new_value, old_value):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(new_value, old_value)
    return new_value - old_value


def sleep_ms(milliseconds):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(milliseconds)
    else:
        time.sleep(milliseconds / 1000.0)


class StatusLED:
    """尽力驱动 C4 指示灯；未安装 LED 时标定仍可正常进行。"""

    def __init__(self, pin_name="C4"):
        self.pin = None
        try:
            from machine import Pin

            self.pin = Pin(pin_name, Pin.OUT, value=True)
        except Exception:
            self.pin = None

    def toggle(self):
        if self.pin is not None:
            self.pin.toggle()

    def set_idle(self):
        if self.pin is not None:
            self.pin.value(True)

    def set_active(self):
        if self.pin is not None:
            self.pin.value(False)


def automatic_countdown(message, delay_ms=5000):
    """在给操作者留出清场时间时保持电机停止。"""
    print(message)
    print("Automatic start in {} ms; power off to abort.".format(delay_ms))
    led = StatusLED()
    led.set_idle()
    start = ticks_ms()
    last_toggle = start
    while ticks_diff(ticks_ms(), start) < delay_ms:
        now = ticks_ms()
        if ticks_diff(now, last_toggle) >= 500:
            led.toggle()
            last_toggle = now
        sleep_ms(20)
    led.set_active()
    return led


def body_speed_from_state(state, kinematics, radius_cm, rotation_gain):
    wheels = state["wheel_speeds"]
    return kinematics.wheels_to_body(
        wheels[0], wheels[1], wheels[2], radius_cm, rotation_gain
    )


def log_sample(log, stage, motor, odometry=None, extra=None):
    state = motor.get_state()
    values = {
        "stage": stage,
        "target_body": state["target_body"],
        "limited_body": state["limited_body"],
        "target_wheels": state["target_wheels"],
        "wheel_speeds": state["wheel_speeds"],
        "encoder_counts": state["encoder_counts"],
        "encoder_totals": state["encoder_totals"],
        "duty": state["duty"],
        "s_curve_enabled": state["s_curve_enabled"],
        "open_loop_calibration": state["open_loop_calibration"],
    }
    if odometry is not None:
        odometry_state = odometry.get_state()
        values["pose"] = (
            odometry_state["x_cm"],
            odometry_state["y_cm"],
            odometry_state["heading_rad"],
        )
        values["heading_unwrapped_rad"] = odometry_state[
            "heading_unwrapped_rad"
        ]
        values["yaw_rate_rad_s"] = odometry_state["yaw_rate_rad_s"]
    if extra is not None:
        for key in extra:
            values[key] = extra[key]
    log.write("sample", **values)
    return state


def mean(values):
    return sum(values) / float(len(values)) if values else 0.0


def linear_fit(x_values, y_values):
    """返回 y = slope*x + intercept 的斜率和截距。"""
    count = len(x_values)
    if count < 2:
        return 0.0, 0.0
    mean_x = mean(x_values)
    mean_y = mean(y_values)
    denominator = 0.0
    numerator = 0.0
    for index in range(count):
        dx = x_values[index] - mean_x
        denominator += dx * dx
        numerator += dx * (y_values[index] - mean_y)
    if denominator <= 1.0e-12:
        return 0.0, mean_y
    slope = numerator / denominator
    return slope, mean_y - slope * mean_x


def vector_magnitude(x, y):
    return math.sqrt(x * x + y * y)

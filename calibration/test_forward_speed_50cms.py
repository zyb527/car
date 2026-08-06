"""脱机 50 cm/s 定速实测：前进与右移各保持 1 秒。

两段测试都使用 ``MotorSystem.command()``，明确绕过 S 曲线。每段先以
50 cm/s 预热 1 秒，待轮速 PI 收敛后才点亮 C4 指示灯并开始计时；指示灯
亮起到熄灭的 1 秒才是需要用卷尺测量的区间。脚本不创建无线模块、不发送
任何无线前馈，适合主车脱机标定。
"""

import math
import time

from motor import MotorSystem
from odometry import OdometrySystem


TARGET_SPEED_CM_S = 50.0
WARMUP_DURATION_MS = 1000
MEASURE_DURATION_MS = 1000
REST_DURATION_MS = 2000
CONTROL_REFRESH_MS = 5
AUTO_START_DELAY_MS = 5000

# 用于在取得卷尺结果后换算当前迁移比例的建议值。
CURRENT_BODY_COMMAND_SPEED_SCALE = 50.0 / 57.0

PROFILES = (
    ("FORWARD", 0.0, TARGET_SPEED_CM_S),
    ("RIGHT", TARGET_SPEED_CM_S, 0.0),
)


def ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.monotonic() * 1000.0)


def ticks_diff(new_value, old_value):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(new_value, old_value)
    return new_value - old_value


def sleep_ms(milliseconds):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(int(milliseconds))
    else:
        time.sleep(milliseconds / 1000.0)


class StatusLED:
    """尽力使用 C4 灯标记测量区间；没有 LED 时仍可看串口标记。"""

    def __init__(self):
        self.pin = None
        try:
            from machine import Pin

            self.pin = Pin("C4", Pin.OUT, value=True)
        except Exception:
            pass

    def set_idle(self):
        if self.pin is not None:
            self.pin.value(True)

    def set_active(self):
        if self.pin is not None:
            self.pin.value(False)

    def toggle(self):
        if self.pin is not None:
            self.pin.toggle()


def automatic_countdown(message, delay_ms):
    print(message)
    print("Automatic start in {} ms; power off to abort.".format(delay_ms))
    led = StatusLED()
    led.set_idle()
    start_ms = ticks_ms()
    last_toggle_ms = start_ms
    while ticks_diff(ticks_ms(), start_ms) < delay_ms:
        now_ms = ticks_ms()
        if ticks_diff(now_ms, last_toggle_ms) >= 500:
            led.toggle()
            last_toggle_ms = now_ms
        sleep_ms(20)
    led.set_idle()
    return led


def suggested_body_command_speed_scale(actual_distance_cm):
    """输入测量窗口内的一秒位移（cm），返回建议的统一迁移比例。"""
    actual_distance_cm = float(actual_distance_cm)
    if actual_distance_cm <= 0.0:
        raise ValueError("actual distance must be positive")
    actual_speed_cm_s = (
        actual_distance_cm * 1000.0 / MEASURE_DURATION_MS
    )
    return (
        CURRENT_BODY_COMMAND_SPEED_SCALE
        * TARGET_SPEED_CM_S
        / actual_speed_cm_s
    )


def _hold_command(motor, vx, vy, duration_ms):
    start_ms = ticks_ms()
    last_refresh_ms = start_ms - CONTROL_REFRESH_MS
    while ticks_diff(ticks_ms(), start_ms) < duration_ms:
        now_ms = ticks_ms()
        if ticks_diff(now_ms, last_refresh_ms) >= CONTROL_REFRESH_MS:
            motor.command(vx, vy, 0.0)
            last_refresh_ms = now_ms
        sleep_ms(1)
    return start_ms, ticks_ms()


def _axis_displacement_cm(start_pose, end_pose, axis):
    """把世界坐标位移投影回测试开始时的车体前进/右移轴。"""
    start_x, start_y, heading_rad = start_pose
    dx = end_pose[0] - start_x
    dy = end_pose[1] - start_y
    if axis == "FORWARD":
        return (
            dx * math.cos(heading_rad)
            + dy * math.sin(heading_rad)
        )
    return dx * math.sin(heading_rad) - dy * math.cos(heading_rad)


def run_profile(motor, odometry, led, profile):
    label, vx, vy = profile
    print("{}_WARMUP_START".format(label))
    led.set_idle()
    _hold_command(motor, vx, vy, WARMUP_DURATION_MS)

    # 预热结束后清零里程计位置；不改航向，避免影响零角速度保持。
    odometry.reset_position(0.0, 0.0)
    start_pose = odometry.get_pose()
    print("{}_MEASURE_START".format(label))
    led.set_active()
    start_ms, end_ms = _hold_command(
        motor, vx, vy, MEASURE_DURATION_MS
    )
    motor.hard_stop()
    led.set_idle()
    end_pose = odometry.get_pose()
    odometry_distance_cm = _axis_displacement_cm(
        start_pose, end_pose, label
    )
    print(
        "{}_MEASURE_END elapsed_ms={} odometry_axis_cm={:.2f}"
        .format(label, ticks_diff(end_ms, start_ms), odometry_distance_cm)
    )
    print(
        "Measure {} physical displacement while C4 is active for 1 s."
        .format(label)
    )


def main():
    odometry = OdometrySystem()
    motor = MotorSystem(odometry=odometry)
    led = None
    try:
        motor.start()
        motor.hard_stop()
        led = automatic_countdown(
            "Clear the area. Forward and right 50 cm/s tests will run offline.",
            AUTO_START_DELAY_MS,
        )

        for index, profile in enumerate(PROFILES):
            run_profile(motor, odometry, led, profile)
            if index + 1 < len(PROFILES):
                print("REST_BEFORE_NEXT_TEST")
                rest_start_ms = ticks_ms()
                while (
                    ticks_diff(ticks_ms(), rest_start_ms)
                    < REST_DURATION_MS
                ):
                    sleep_ms(10)

        print("Use the physical one-second distances D_forward and D_right:")
        print("forward command scale = current_scale * 50 / D_forward")
        print("right command scale   = current_scale * 50 / D_right")
        print("Do not merge a large forward/right difference into one scale.")
    finally:
        motor.hard_stop()
        motor.stop()
        if led is not None:
            led.set_idle()


if __name__ == "__main__":
    main()

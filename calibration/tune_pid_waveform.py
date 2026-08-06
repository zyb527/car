"""适用于 Thonny 的闭环 PI 波形测试。

每行波形数据包含六个纯数字列：
target_wheel_1 measured_wheel_1 target_wheel_2 measured_wheel_2
target_wheel_3 measured_wheel_3

请在倒计时结束前打开 Thonny 绘图器。本测试使用 command()，因此波形显示的
是未经过 S 曲线整形的轮速 PI 响应。请在无障碍地面运行并留足线缆余量；断电
即可中止。
"""

import math

from motor import MotorSystem
from calibration_store import ticks_ms
from common import automatic_countdown, sleep_ms, ticks_diff


# 可选："sine_forward"、"sine_right"、"step_forward" 或 "step_right"
TEST_MODE = "step_right"
AUTO_START_DELAY_MS = 3000
TEST_DURATION_MS = 13000
SINE_AMPLITUDE_CM_S = 300.0
SINE_PERIOD_MS = 3000
STEP_SPEED_CM_S = 100.0
STEP_HOLD_MS = 2000
PRINT_PERIOD_MS = 20
# 不依赖板端 motor.py 中可能残留的旧 120 cm/s 默认值。
# 与正常底盘当前的车体速度上限保持一致；STEP_SPEED_CM_S 更大时仍会限到此值。
TEST_MAX_XY_SPEED_CM_S = 800.0
# 串口绘图和文件写入可能暂时阻塞 Python 主循环；仅本人工看护的标定脚本使用
# 较长的看门狗时间。
WAVEFORM_WATCHDOG_TIMEOUT_MS = 500


def body_command(elapsed_ms):
    if TEST_MODE == "sine_forward":
        phase = 2.0 * math.pi * elapsed_ms / SINE_PERIOD_MS
        return 0.0, SINE_AMPLITUDE_CM_S * math.sin(phase), 0.0
    if TEST_MODE == "sine_right":
        phase = 2.0 * math.pi * elapsed_ms / SINE_PERIOD_MS
        return SINE_AMPLITUDE_CM_S * math.sin(phase), 0.0, 0.0
    if TEST_MODE == "step_forward":
        phase = (elapsed_ms // STEP_HOLD_MS) % 4
        if phase == 0:
            return 0.0, 0.0, 0.0
        if phase == 1:
            return 0.0, STEP_SPEED_CM_S, 0.0
        if phase == 2:
            return 0.0, 0.0, 0.0
        return 0.0, -STEP_SPEED_CM_S, 0.0
    if TEST_MODE == "step_right":
        phase = (elapsed_ms // STEP_HOLD_MS) % 4
        if phase == 0:
            return 0.0, 0.0, 0.0
        if phase == 1:
            return STEP_SPEED_CM_S, 0.0, 0.0
        if phase == 2:
            return 0.0, 0.0, 0.0
        return -STEP_SPEED_CM_S, 0.0, 0.0
    raise ValueError("Unknown TEST_MODE: " + TEST_MODE)


def print_waveform(state):
    target = state["target_wheels"]
    measured = state["wheel_speeds"]
    print(
        "{:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f}".format(
            target[0],
            measured[0],
            target[1],
            measured[1],
            target[2],
            measured[2],
        )
    )


def main():
    if TEST_MODE not in (
        "sine_forward", "sine_right", "step_forward", "step_right"
    ):
        raise ValueError(
            "Set TEST_MODE to sine_forward, sine_right, step_forward or step_right"
        )

    motor = MotorSystem(odometry=None)
    try:
        motor.config.watchdog_timeout_ms = WAVEFORM_WATCHDOG_TIMEOUT_MS
        motor.config.max_xy_speed_cm_s = TEST_MAX_XY_SPEED_CM_S
        motor.limiter.max_xy_speed = TEST_MAX_XY_SPEED_CM_S
        motor.start()
        motor.hard_stop()
        automatic_countdown(
            "Open Thonny Plotter; PID waveform starts automatically.",
            AUTO_START_DELAY_MS,
        )
        start = ticks_ms()
        last_print = start - PRINT_PERIOD_MS
        while ticks_diff(ticks_ms(), start) < TEST_DURATION_MS:
            now = ticks_ms()
            vx, vy, w = body_command(ticks_diff(now, start))
            motor.command(vx, vy, w)
            if ticks_diff(now, last_print) >= PRINT_PERIOD_MS:
                state = motor.get_state()
                print_waveform(state)
                last_print = now
            sleep_ms(5)
    finally:
        motor.hard_stop()
        motor.stop()


if __name__ == "__main__":
    main()

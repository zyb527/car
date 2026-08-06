"""Thonny Plotter 用的单轮闭环阶跃波形测试。

每行仅输出两列纯数字：目标轮速、实测轮速。打开 Thonny Plotter 后，
将看到两条清晰的曲线。测试时必须架空全部车轮。
"""

from motor import ChassisKinematics, MotorSystem
from calibration_store import ticks_ms
from common import automatic_countdown, sleep_ms, ticks_diff


# 从 0、1、2 中选择要测试的单个轮子。
TEST_WHEEL_INDEX = 0
# 单轮阶跃目标（cm/s）；先用 5、10、15 等低速查看补偿是否过强。
STEP_SPEED_CM_S =20.0
# 波形依次为：静止 -> 正向 -> 静止 -> 反向 -> 静止。
STEP_HOLD_MS = 1800
PRINT_PERIOD_MS = 20
AUTO_START_DELAY_MS = 3000
# 仅人工观察的测试，避免串口打印偶发卡顿触发看门狗。
WAVEFORM_WATCHDOG_TIMEOUT_MS = 500


def target_for_phase(elapsed_ms):
    phase = (elapsed_ms // STEP_HOLD_MS) % 5
    if phase == 1:
        return STEP_SPEED_CM_S
    if phase == 3:
        return -STEP_SPEED_CM_S
    return 0.0


def command_single_wheel(motor, speed_cm_s):
    wheels = [0.0, 0.0, 0.0]
    wheels[TEST_WHEEL_INDEX] = float(speed_cm_s)
    vx, vy, w = ChassisKinematics.wheels_to_body(
        wheels[0],
        wheels[1],
        wheels[2],
        motor.config.robot_radius_cm,
        motor.config.rotation_gain,
    )
    # 绕过 motor.command() 的全车平移速度缩放，直接将其作为底层数学目标
    motor._open_loop_calibration = False
    motor._use_s_curve = False
    target = motor._make_feasible_body_target(vx, vy, w)
    motor._activate_target(target)


def main():
    if TEST_WHEEL_INDEX not in (0, 1, 2):
        raise ValueError("TEST_WHEEL_INDEX must be 0, 1, or 2")

    motor = MotorSystem(odometry=None)
    try:
        motor.config.watchdog_timeout_ms = WAVEFORM_WATCHDOG_TIMEOUT_MS
        motor.start()
        motor.hard_stop()
        automatic_countdown(
            "Raise all wheels; open Thonny Plotter. Two traces start automatically.",
            AUTO_START_DELAY_MS,
        )
        start = ticks_ms()
        last_print = start - PRINT_PERIOD_MS
        # 5 个阶段，完整显示一次正反向阶跃和停车响应。
        while ticks_diff(ticks_ms(), start) < STEP_HOLD_MS * 5:
            now = ticks_ms()
            target = target_for_phase(ticks_diff(now, start))
            command_single_wheel(motor, target)
            if ticks_diff(now, last_print) >= PRINT_PERIOD_MS:
                measured = motor.get_state()["wheel_speeds"][TEST_WHEEL_INDEX]
                # 不要在这一行添加标签、轮号或字典；Plotter 只需要两列数字。
                print("{:.3f} {:.3f}".format(target, measured))
                last_print = now
            sleep_ms(5)
    finally:
        motor.hard_stop()
        motor.stop()


if __name__ == "__main__":
    main()

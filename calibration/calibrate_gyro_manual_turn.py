"""手动转一整圈，标定 IMU 偏航轴比例。

使用方法：
1. 将车放在平整地面，保持静止上电；
2. 倒计时结束后，沿 EXPECTED_TURN_DEG 指定方向缓慢、连续地手动转准确一圈；
3. 松手并保持静止。脚本会在检测到转动已停止后打印建议比例。

`heading_rad` 被限制在 [-180°, +180°]，本脚本必须使用连续的
`heading_unwrapped_rad`，否则转满一圈会错误地显示为约 0°。
"""

import math

from motor import MotorSystem
from odometry import OdometrySystem
from calibration_store import ticks_ms
from common import automatic_countdown, sleep_ms, ticks_diff


# 逆时针为 +360；若手动顺时针转一整圈，改为 -360。
EXPECTED_TURN_DEG = 360.0
AUTO_START_DELAY_MS = 3000
TIMEOUT_MS = 30000

# 至少转过该角度后才允许“停稳”完成，避免刚起步或中途短暂停就结束。
MINIMUM_COMPLETION_DEG = 270.0
MOVING_YAW_RATE_RAD_S = 0.08
SETTLE_MS = 1200


def main():
    odometry = OdometrySystem()
    motor = MotorSystem(odometry=odometry)
    try:
        # start() 会初始化并校准 IMU；整个过程中不发送任何电机速度指令。
        motor.start()
        motor.hard_stop()
        automatic_countdown(
            "Keep the car still. Manual gyro-turn test starts automatically.",
            AUTO_START_DELAY_MS,
        )

        odometry.set_pose(0.0, 0.0, 0.0)
        start = ticks_ms()
        last_moving = start
        started = False
        print("Turn the car exactly {:.0f} deg, then release it and keep still.".format(
            EXPECTED_TURN_DEG
        ))

        while ticks_diff(ticks_ms(), start) < TIMEOUT_MS:
            now = ticks_ms()
            state = odometry.get_state()
            turn_deg = state["heading_unwrapped_rad"] * 180.0 / math.pi
            yaw_rate = state["yaw_rate_rad_s"]

            if not started and (
                abs(turn_deg) >= 10.0
                or abs(yaw_rate) >= MOVING_YAW_RATE_RAD_S
            ):
                started = True
                last_moving = now

            if started and abs(yaw_rate) >= MOVING_YAW_RATE_RAD_S:
                last_moving = now

            if (
                started
                and abs(turn_deg) >= MINIMUM_COMPLETION_DEG
                and ticks_diff(now, last_moving) >= SETTLE_MS
            ):
                break
            sleep_ms(20)
        else:
            raise RuntimeError("manual turn timed out")

        state = odometry.get_state()
        reported_deg = state["heading_unwrapped_rad"] * 180.0 / math.pi
        if abs(reported_deg) <= 1.0e-6:
            raise RuntimeError("reported turn is too small")

        scale_factor = EXPECTED_TURN_DEG / reported_deg
        yaw_raw_axis = odometry.config.axis_indices[2]
        old_scales = list(odometry.config.gyro_scale_raw)
        new_scales = list(old_scales)
        new_scales[yaw_raw_axis] *= scale_factor

        print("reported_turn_deg={:.3f}".format(reported_deg))
        print("expected_turn_deg={:.3f}".format(EXPECTED_TURN_DEG))
        print("gyro_scale_factor={:.6f}".format(scale_factor))
        print("yaw_raw_axis_index={}".format(yaw_raw_axis))
        print("current_gyro_scale_raw={}".format(tuple(old_scales)))
        print("suggested_gyro_scale_raw={}".format(tuple(new_scales)))
    finally:
        motor.hard_stop()
        motor.stop()


if __name__ == "__main__":
    main()

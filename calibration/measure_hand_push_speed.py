"""手推底盘速度观测。

脚本启动编码器采样，但始终保持三个电机 PWM 为 0。用手推动或旋转底盘时，
每 50 ms 打印三个轮速以及由三轮正运动学计算出的车体速度。

坐标约定：
    vx：向右为正，cm/s
    vy：向前为正，cm/s
    w ：逆时针为正，rad/s

按 Ctrl-C 结束测试。
"""

import math
import time

from motor import ChassisKinematics, MotorSystem


PRINT_PERIOD_MS = 800


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
        time.sleep_ms(milliseconds)
    else:
        time.sleep(milliseconds / 1000.0)


def main():
    motor = MotorSystem(odometry=None)

    try:
        motor.start()
        motor.hard_stop()
        motor.reset_encoder_totals()

        start_ms = ticks_ms()
        last_print_ms = start_ms - PRINT_PERIOD_MS

        print("Hand-push speed measurement started; motor duty is zero.")
        print(
            "t_ms,c1,c2,c3,"
            "wheel1_cm_s,wheel2_cm_s,wheel3_cm_s,"
            "vx_cm_s,vy_cm_s,linear_cm_s,w_rad_s"
        )

        while True:
            now_ms = ticks_ms()
            if ticks_diff(now_ms, last_print_ms) >= PRINT_PERIOD_MS:
                state = motor.get_state()
                wheel_1, wheel_2, wheel_3 = state["wheel_speeds"]
                count_1, count_2, count_3 = state["encoder_counts"]

                vx, vy, w = ChassisKinematics.wheels_to_body(
                    wheel_1,
                    wheel_2,
                    wheel_3,
                    motor.config.robot_radius_cm,
                    motor.config.rotation_gain,
                )
                linear_speed = math.sqrt(vx * vx + vy * vy)

                print(
                    "{},{},{},{},{:.3f},{:.3f},{:.3f},"
                    "{:.3f},{:.3f},{:.3f},{:.4f}".format(
                        ticks_diff(now_ms, start_ms),
                        count_1,
                        count_2,
                        count_3,
                        wheel_1,
                        wheel_2,
                        wheel_3,
                        vx,
                        vy,
                        linear_speed,
                        w,
                    )
                )
                last_print_ms = now_ms

            sleep_ms(5)
    except KeyboardInterrupt:
        print("Hand-push speed measurement stopped.")
    finally:
        motor.hard_stop()
        motor.stop()


if __name__ == "__main__":
    main()

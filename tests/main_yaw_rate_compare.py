"""主车 2 rad/s 绝对角速度录像标定测试。

使用方法：
1. 在主车顶部贴一条清晰的方向标记，固定手机俯拍。
2. 程序用 motor.command() 驱动主车逆时针旋转，不使用 move()。
3. 先预转稳定，随后 C4 灯点亮并开始正式计时。
4. 按 2 rad/s 转完理论 5 圈所需的 15.708 s 后，立即下发零命令并熄灯。
5. 从视频统计亮灯期间的实际圈数（可包含小数圈）：

       actual_w = 2*pi*actual_turns / MEASURE_DURATION_S

如果角速度准确，亮灯期间应正好转 5 圈。
"""

import math
import time

try:
    from machine import Pin
except ImportError:
    Pin = None

from control import normalize_angle
from motor import ChassisKinematics, MotorSystem
from odometry import OdometrySystem


COMMAND_W_RAD_S = 2.0
THEORETICAL_TURNS = 5.0
MEASURE_DURATION_S = 2.0 * math.pi * THEORETICAL_TURNS / abs(
    COMMAND_W_RAD_S
)
CONTROL_PERIOD_MS = 20
STARTUP_HOLD_S = 2.0
PRESPIN_S = 1.5
# 到点先用零 command 主动制动，同时继续观察停止后的角速度残留。
BRAKE_OBSERVE_S = 1.0


def _ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.monotonic() * 1000.0)


def _ticks_diff(new_value, old_value):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(new_value, old_value)
    return new_value - old_value


def _sleep_ms(milliseconds):
    milliseconds = max(0, int(milliseconds))
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(milliseconds)
    else:
        time.sleep(milliseconds / 1000.0)


def _finite(value):
    return value == value and -1.0e6 < value < 1.0e6


def _format_value(value):
    return "nan" if value is None or not _finite(value) else "{:.5f}".format(value)


def _encoder_yaw_rate(motor):
    """尽量读取编码器角速度；旧版板端 motor 无接口时返回 None。"""
    try:
        if hasattr(motor, "get_wheel_speeds"):
            wheels = motor.get_wheel_speeds()
        elif hasattr(motor, "get_state"):
            wheels = motor.get_state().get("wheel_speeds")
        elif hasattr(motor, "get_debug_state"):
            wheels = motor.get_debug_state().get("wheel_speeds")
        else:
            return None
    except Exception:
        return None
    if wheels is None or len(wheels) != 3:
        return None
    try:
        return ChassisKinematics.wheels_to_body(
            float(wheels[0]),
            float(wheels[1]),
            float(wheels[2]),
            motor.config.robot_radius_cm,
            motor.config.rotation_gain,
        )[2]
    except Exception:
        return None


def _sample(motor, odometry, stage, command_w, start_ms, last_sample):
    now_ms = _ticks_ms()
    elapsed_ms = _ticks_diff(now_ms, last_sample["ms"])
    if elapsed_ms < CONTROL_PERIOD_MS:
        return False

    dt = max(0.001, min(elapsed_ms / 1000.0, 0.1))
    last_sample["ms"] = now_ms
    motor.command(0.0, 0.0, float(command_w))

    state = odometry.get_state()
    heading = float(odometry.get_pose()[2])
    try:
        imu_w = float(state.get("yaw_rate_rad_s"))
    except (TypeError, ValueError):
        imu_w = None
    encoder_w = _encoder_yaw_rate(motor)
    heading_delta = normalize_angle(heading - last_sample["heading"])
    last_sample["heading"] = heading
    heading_diff_w = heading_delta / dt
    last_sample["unwrapped_heading"] += heading_delta

    print(
        "{:.3f},{},{:.5f},{},{},{},{:.3f}".format(
            _ticks_diff(now_ms, start_ms) / 1000.0,
            stage,
            float(command_w),
            _format_value(imu_w),
            _format_value(encoder_w),
            _format_value(heading_diff_w),
            math.degrees(heading),
        )
    )
    return True


def _run_timed_command(motor, odometry, stage, command_w, duration_s):
    start_ms = _ticks_ms()
    last_sample = {
        "ms": start_ms - CONTROL_PERIOD_MS,
        "heading": float(odometry.get_pose()[2]),
        "unwrapped_heading": 0.0,
    }
    duration_ms = int(round(float(duration_s) * 1000.0))
    while _ticks_diff(_ticks_ms(), start_ms) < duration_ms:
        if not _sample(
            motor,
            odometry,
            stage,
            command_w,
            start_ms,
            last_sample,
        ):
            _sleep_ms(1)
    return last_sample["unwrapped_heading"]


def main():
    motor = None
    led_c4 = None
    print("time_s,stage,w_cmd,imu_w,encoder_w,heading_diff_w,heading_deg")
    try:
        if Pin is not None:
            led_c4 = Pin("C4", Pin.OUT)
            # C4 为低电平点亮；正式计时前保持熄灭。
            led_c4.value(1)

        odometry = OdometrySystem()
        motor = MotorSystem(odometry=odometry)
        motor.start()
        motor.hard_stop()

        print("正在标定IMU，请保持主车静止 {} s".format(STARTUP_HOLD_S))
        time.sleep(STARTUP_HOLD_S)
        odometry.set_pose(0.0, 0.0, 0.0)

        print(
            "预转 {:.1f} s，命令角速度 {:.3f} rad/s".format(
                PRESPIN_S,
                COMMAND_W_RAD_S,
            )
        )
        _run_timed_command(
            motor,
            odometry,
            "prespin",
            COMMAND_W_RAD_S,
            PRESPIN_S,
        )

        if led_c4 is not None:
            led_c4.value(0)
        print(
            "MEASURE_START,duration_s={:.6f},expected_turns={:.3f}".format(
                MEASURE_DURATION_S,
                THEORETICAL_TURNS,
            )
        )
        measured_heading = _run_timed_command(
            motor,
            odometry,
            "measure",
            COMMAND_W_RAD_S,
            MEASURE_DURATION_S,
        )

        # 时间到：先下发零目标，再熄灭计时灯。
        motor.command(0.0, 0.0, 0.0)
        if led_c4 is not None:
            led_c4.value(1)
        print("MEASURE_STOP,zero_command_sent")

        internal_turns = measured_heading / (2.0 * math.pi)
        internal_average_w = measured_heading / MEASURE_DURATION_S
        print(
            "INTERNAL_SUMMARY,duration_s={:.6f},heading_turns={:.5f},heading_w={:.5f}".format(
                MEASURE_DURATION_S,
                internal_turns,
                internal_average_w,
            )
        )

        _run_timed_command(
            motor,
            odometry,
            "brake",
            0.0,
            BRAKE_OBSERVE_S,
        )
        motor.hard_stop()
        print(
            "测试完成：视频中亮灯期间若不是 {:.3f} 圈，请按 2*pi*圈数/{:.6f} 计算真实角速度。".format(
                THEORETICAL_TURNS,
                MEASURE_DURATION_S,
            )
        )
    except KeyboardInterrupt:
        print("测试被手动中断")
    except Exception as error:
        print("角速度测试异常: {}".format(repr(error)))
        try:
            import sys
            sys.print_exception(error)
        except Exception:
            pass
    finally:
        if led_c4 is not None:
            led_c4.value(1)
        if motor is not None:
            motor.hard_stop()
            motor.stop()


if __name__ == "__main__":
    main()

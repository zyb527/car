"""测量 S 曲线的加速度、减速度、加加速度及编码器制动距离。"""

import math

from motor import ChassisKinematics, MotorSystem
from odometry import OdometrySystem
from calibration_store import CalibrationLog, DEFAULT_LOG_PATH, ticks_ms
from common import (
    automatic_countdown,
    sleep_ms,
    ticks_diff,
)


# 只测试前进：避免在还未确认最高安全速度前自动执行倒车、横移和旋转。
# 每档都会从静止开始并自动软刹车。若某档已出现明显打滑、跑偏或振荡，
# 不要继续提高档位；关闭电源中止并在日志中保留已完成档位的数据。
PROFILE_HOLD_MS = 4500
INTER_PROFILE_PAUSE_MS = 1500

# 固定四种工况的终点速度，只改变加速度或 jerk。
TRANSLATION_TARGET_CM_S = 150.0
ROTATION_TARGET_RAD_S = 2.0
# car141929 的线速度使用 6000 pulse/m，当前三轮约为 21100 pulse/m；
# 其 1650 cm/s² / 21000 cm/s³ 约等于当前单位的 470 / 6000。
# 因此围绕该换算值扫描，而非直接照抄旧工程数值。
XY_ACCEL_REFERENCE = 960.0
XY_JERK_REFERENCE = 12000.0
W_ACCEL_REFERENCE = 17.0
W_JERK_REFERENCE = 260.0
XY_ACCEL_LEVELS = (720.0, 960.0, 1200.0, 1440.0)
XY_JERK_LEVELS = (9000.0, 12000.0, 15000.0, 18000.0)
W_ACCEL_LEVELS = (12.0, 17.0, 22.0, 28.0)
W_JERK_LEVELS = (160.0, 260.0, 360.0, 480.0)


def make_profile(label, vx, vy, w, xy_accel, xy_jerk, w_accel, w_jerk):
    return (
        label, vx, vy, w, PROFILE_HOLD_MS,
        xy_accel, xy_jerk, w_accel, w_jerk,
    )


def translation_scan(prefix, vx, vy):
    profiles = []
    for accel in XY_ACCEL_LEVELS:
        profiles.append(make_profile(
            prefix + "_accel_{:g}".format(accel), vx, vy, 0.0,
            accel, XY_JERK_REFERENCE, W_ACCEL_REFERENCE, W_JERK_REFERENCE,
        ))
    for jerk in XY_JERK_LEVELS:
        profiles.append(make_profile(
            prefix + "_jerk_{:g}".format(jerk), vx, vy, 0.0,
            XY_ACCEL_REFERENCE, jerk, W_ACCEL_REFERENCE, W_JERK_REFERENCE,
        ))
    return profiles


def rotation_scan(prefix, vx, vy, w):
    profiles = []
    for accel in W_ACCEL_LEVELS:
        profiles.append(make_profile(
            prefix + "_w_accel_{:g}".format(accel), vx, vy, w,
            XY_ACCEL_REFERENCE, XY_JERK_REFERENCE, accel, W_JERK_REFERENCE,
        ))
    for jerk in W_JERK_LEVELS:
        profiles.append(make_profile(
            prefix + "_w_jerk_{:g}".format(jerk), vx, vy, w,
            XY_ACCEL_REFERENCE, XY_JERK_REFERENCE, W_ACCEL_REFERENCE, jerk,
        ))
    return profiles


# 前移、侧移、原地旋转，以及前移同时旋转。组合运动分别扫描平移和
# 旋转的加速度/jerk，所以能分辨是哪个维度先导致车体不稳定。
PROFILES = []
PROFILES += translation_scan("forward", 0.0, TRANSLATION_TARGET_CM_S)
PROFILES += translation_scan("right", TRANSLATION_TARGET_CM_S, 0.0)
PROFILES += rotation_scan("rotate_ccw", 0.0, 0.0, ROTATION_TARGET_RAD_S)
PROFILES += translation_scan(
    "forward_turn_xy", 0.0, TRANSLATION_TARGET_CM_S
)
for index in range(len(PROFILES) - 8, len(PROFILES)):
    profile = PROFILES[index]
    PROFILES[index] = (
        profile[0], profile[1], profile[2], ROTATION_TARGET_RAD_S,
        profile[4], profile[5], profile[6], profile[7], profile[8],
    )
PROFILES += rotation_scan(
    "forward_turn_w", 0.0, TRANSLATION_TARGET_CM_S, ROTATION_TARGET_RAD_S
)

# 此值仅在本标定脚本运行期间生效，不会写回 motor.py。
# 轮速保护仍保持在 motor.py 的 150 cm/s。对纯前进而言，轮速保护将
# 180 cm/s 及以上的请求自动缩到约 173 cm/s 的车体速度；所以后续档位
# 能直接显示“当前轮速保护下的车体上限”，而不会解除轮速保护硬冲。
TEST_MAX_XY_SPEED_CM_S = 800.0
SAMPLE_PERIOD_MS = 50
STOP_TIMEOUT_MS = 4000
AUTO_START_DELAY_MS = 7000
LOG_PATH = DEFAULT_LOG_PATH
# 标定时 JSON 写入 Flash 偶尔会阻塞 200 ms 左右；保留正常程序的 150 ms
# 看门狗，只在这份有人值守的标定脚本中临时放宽，避免记录动作触发硬停。
CALIBRATION_WATCHDOG_TIMEOUT_MS = 500


def scalar_along_profile(body_value, target):
    vx, vy, w = body_value
    target_vx, target_vy, target_w = target
    translation = math.sqrt(target_vx * target_vx + target_vy * target_vy)
    if translation > 1.0e-9:
        return (vx * target_vx + vy * target_vy) / translation
    sign = 1.0 if target_w >= 0.0 else -1.0
    return w * sign


def log_motion_sample(log, stage, motor):
    """只保存计算加速度/jerk 所需字段，减少 Flash 日志占用。"""
    state = motor.get_state()
    log.write(
        "motion_sample",
        stage=stage,
        target_body=state["target_body"],
        limited_body=state["limited_body"],
        wheel_speeds=state["wheel_speeds"],
    )
    return state


def summarize_series(points):
    if len(points) < 3:
        return {
            "max_speed": 0.0,
            "max_accel": 0.0,
            "max_decel": 0.0,
            "max_jerk": 0.0,
        }
    max_speed = max(abs(point[1]) for point in points)
    accelerations = []
    for index in range(1, len(points)):
        dt = (points[index][0] - points[index - 1][0]) / 1000.0
        if dt > 0.0:
            accelerations.append(
                (
                    points[index][0],
                    (points[index][1] - points[index - 1][1]) / dt,
                )
            )
    positive = [value for _, value in accelerations if value >= 0.0]
    negative = [-value for _, value in accelerations if value < 0.0]
    jerks = []
    for index in range(1, len(accelerations)):
        dt = (
            accelerations[index][0] - accelerations[index - 1][0]
        ) / 1000.0
        if dt > 0.0:
            jerks.append(
                abs(
                    accelerations[index][1] - accelerations[index - 1][1]
                ) / dt
            )
    return {
        "max_speed": max_speed,
        "max_accel": max(positive) if positive else 0.0,
        "max_decel": max(negative) if negative else 0.0,
        "max_jerk": max(jerks) if jerks else 0.0,
    }


def run_profile(motor, odometry, log, profile):
    (
        label, vx, vy, w, hold_ms, xy_accel, xy_jerk, w_accel, w_jerk,
    ) = profile
    motor.config.xy_accel_up_cm_s2 = xy_accel
    motor.config.xy_jerk_cm_s3 = xy_jerk
    motor.config.w_accel_up_rad_s2 = w_accel
    motor.config.w_jerk_rad_s3 = w_jerk
    motor.limiter.xy_accel_up = xy_accel
    motor.limiter.xy_jerk = xy_jerk
    motor.limiter.w_accel_up = w_accel
    motor.limiter.w_jerk = w_jerk
    target = (vx, vy, w)
    command_series = []
    measured_series = []
    command_rotation_series = []
    measured_rotation_series = []
    maximum_wheel_speed = 0.0
    start = ticks_ms()
    last_sample = start - SAMPLE_PERIOD_MS

    while ticks_diff(ticks_ms(), start) < hold_ms:
        motor.move(vx, vy, w)
        now = ticks_ms()
        if ticks_diff(now, last_sample) >= SAMPLE_PERIOD_MS:
            state = log_motion_sample(log, label, motor)
            elapsed = ticks_diff(now, start)
            measured_body = ChassisKinematics.wheels_to_body(
                state["wheel_speeds"][0],
                state["wheel_speeds"][1],
                state["wheel_speeds"][2],
                motor.config.robot_radius_cm,
                motor.config.rotation_gain,
            )
            command_series.append(
                (elapsed, scalar_along_profile(state["limited_body"], target))
            )
            measured_series.append(
                (elapsed, scalar_along_profile(measured_body, target))
            )
            command_rotation_series.append((elapsed, state["limited_body"][2]))
            measured_rotation_series.append((elapsed, measured_body[2]))
            maximum_wheel_speed = max(
                maximum_wheel_speed,
                max(abs(value) for value in state["wheel_speeds"]),
            )
            last_sample = now
        sleep_ms(5)

    stop_pose = odometry.get_pose()
    stop_heading_unwrapped_rad = odometry.get_state()[
        "heading_unwrapped_rad"
    ]
    stop_start = ticks_ms()
    motor.soft_stop()
    while motor.get_state()["motion_active"]:
        now = ticks_ms()
        if ticks_diff(now, stop_start) >= STOP_TIMEOUT_MS:
            motor.hard_stop()
            break
        if ticks_diff(now, last_sample) >= SAMPLE_PERIOD_MS:
            state = log_motion_sample(log, label + "_soft_stop", motor)
            elapsed = ticks_diff(now, start)
            measured_body = ChassisKinematics.wheels_to_body(
                state["wheel_speeds"][0],
                state["wheel_speeds"][1],
                state["wheel_speeds"][2],
                motor.config.robot_radius_cm,
                motor.config.rotation_gain,
            )
            command_series.append(
                (elapsed, scalar_along_profile(state["limited_body"], target))
            )
            measured_series.append(
                (elapsed, scalar_along_profile(measured_body, target))
            )
            command_rotation_series.append((elapsed, state["limited_body"][2]))
            measured_rotation_series.append((elapsed, measured_body[2]))
            maximum_wheel_speed = max(
                maximum_wheel_speed,
                max(abs(value) for value in state["wheel_speeds"]),
            )
            last_sample = now
        sleep_ms(5)

    final_pose = odometry.get_pose()
    final_heading_unwrapped_rad = odometry.get_state()[
        "heading_unwrapped_rad"
    ]
    braking_distance = math.sqrt(
        (final_pose[0] - stop_pose[0]) ** 2
        + (final_pose[1] - stop_pose[1]) ** 2
    )
    summary = {
        "stage": label,
        "body_target": target,
        "configured_xy_accel_up_cm_s2": xy_accel,
        "configured_xy_jerk_cm_s3": xy_jerk,
        "configured_w_accel_up_rad_s2": w_accel,
        "configured_w_jerk_rad_s3": w_jerk,
        "command_profile": summarize_series(command_series),
        "measured_profile": summarize_series(measured_series),
        "command_rotation_profile": summarize_series(command_rotation_series),
        "measured_rotation_profile": summarize_series(measured_rotation_series),
        "max_measured_wheel_speed_cm_s": maximum_wheel_speed,
        "encoder_braking_distance_cm": braking_distance,
        "braking_heading_change_rad": (
            final_heading_unwrapped_rad - stop_heading_unwrapped_rad
        ),
    }
    log.write("motion_stage_summary", **summary)
    return summary


def main():
    log = CalibrationLog("motion_profile", LOG_PATH)
    odometry = OdometrySystem()
    motor = MotorSystem(odometry=odometry)
    status = "complete"
    error_text = None
    try:
        motor.config.watchdog_timeout_ms = CALIBRATION_WATCHDOG_TIMEOUT_MS
        motor.config.max_xy_speed_cm_s = TEST_MAX_XY_SPEED_CM_S
        motor.limiter.max_xy_speed = TEST_MAX_XY_SPEED_CM_S
        motor.start()
        motor.hard_stop()
        led = automatic_countdown(
            "Clear the test area before automatic motion-profile tests.",
            AUTO_START_DELAY_MS,
        )
        for profile in PROFILES:
            odometry.set_pose(0.0, 0.0, 0.0)
            summary = run_profile(motor, odometry, log, profile)
            sleep_ms(INTER_PROFILE_PAUSE_MS)
    except Exception as error:
        status = "error"
        error_text = repr(error)
        print(error_text)
        raise
    finally:
        motor.hard_stop()
        motor.stop()
        if "led" in locals():
            led.set_idle()
        log.close(status, error_text)


if __name__ == "__main__":
    main()

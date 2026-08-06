"""连续提速的车体最高速度测试。"""

import math

from motor import ChassisKinematics, MotorSystem
from odometry import OdometrySystem
from calibration_store import CalibrationLog, DEFAULT_LOG_PATH, ticks_ms
from common import automatic_countdown, log_sample, sleep_ms, ticks_diff


# 仅前进。速度请求以 RAMP_RATE_CM_S2 连续增长，而非分档阶跃。
RAMP_RATE_CM_S2 = 100.0
MAX_REQUEST_SPEED_CM_S = 900.0
MAX_SPEED_HOLD_MS = 3000
SAMPLE_PERIOD_MS = 20
AUTO_START_DELAY_MS = 7000
CALIBRATION_WATCHDOG_TIMEOUT_MS = 500

# 只放宽本脚本的车体请求上限；轮速上限仍由 motor.py 决定。
TEST_MAX_XY_SPEED_CM_S = MAX_REQUEST_SPEED_CM_S
LOG_PATH = DEFAULT_LOG_PATH


def degrees(value_rad):
    return value_rad * 180.0 / math.pi


def actual_forward_speed(motor):
    state = motor.get_state()
    _, forward_speed, _ = ChassisKinematics.wheels_to_body(
        state["wheel_speeds"][0],
        state["wheel_speeds"][1],
        state["wheel_speeds"][2],
        motor.config.robot_radius_cm,
        motor.config.rotation_gain,
    )
    return forward_speed


def main():
    log = CalibrationLog("body_speed_limit", LOG_PATH)
    odometry = OdometrySystem()
    motor = MotorSystem(odometry=odometry)
    status = "complete"
    error_text = None
    reason = "max_request_complete"
    maximum_actual_speed = 0.0
    led = None
    try:
        motor.config.watchdog_timeout_ms = CALIBRATION_WATCHDOG_TIMEOUT_MS
        motor.config.max_xy_speed_cm_s = TEST_MAX_XY_SPEED_CM_S
        motor.limiter.max_xy_speed = TEST_MAX_XY_SPEED_CM_S
        motor.start()
        motor.hard_stop()
        led = automatic_countdown(
            "Clear the test area. The vehicle will continuously accelerate forward.",
            AUTO_START_DELAY_MS,
        )
        odometry.set_pose(0.0, 0.0, 0.0)
        initial_state = odometry.get_state()
        initial_heading_rad = initial_state["heading_unwrapped_rad"]
        initial_y_cm = initial_state["y_cm"]
        start = ticks_ms()
        last_sample = start - SAMPLE_PERIOD_MS

        while True:
            now = ticks_ms()
            elapsed_ms = ticks_diff(now, start)
            requested_speed = min(
                MAX_REQUEST_SPEED_CM_S,
                RAMP_RATE_CM_S2 * elapsed_ms / 1000.0,
            )
            motor.move(0.0, requested_speed, 0.0)
            if ticks_diff(now, last_sample) >= SAMPLE_PERIOD_MS:
                forward_speed = actual_forward_speed(motor)
                maximum_actual_speed = max(
                    maximum_actual_speed, abs(forward_speed)
                )
                log_sample(
                    log,
                    "continuous_forward_ramp",
                    motor,
                    odometry,
                    {
                        "requested_forward_speed_cm_s": requested_speed,
                        "actual_forward_speed_cm_s": forward_speed,
                    },
                )
                last_sample = now
            if (
                requested_speed >= MAX_REQUEST_SPEED_CM_S
                and elapsed_ms >= (
                    MAX_REQUEST_SPEED_CM_S / RAMP_RATE_CM_S2 * 1000.0
                    + MAX_SPEED_HOLD_MS
                )
            ):
                break
            sleep_ms(5)
        if reason == "max_request_complete":
            motor.soft_stop()
            while motor.get_state()["motion_active"]:
                sleep_ms(5)
        final_state = odometry.get_state()
        summary = {
            "stop_reason": reason,
            "max_actual_forward_speed_cm_s": maximum_actual_speed,
            "final_roll_deg": degrees(final_state["roll_rad"]),
            "final_pitch_deg": degrees(final_state["pitch_rad"]),
            "final_heading_change_deg": degrees(
                final_state["heading_unwrapped_rad"] - initial_heading_rad
            ),
            "final_lateral_drift_cm": final_state["y_cm"] - initial_y_cm,
            "ramp_rate_cm_s2": RAMP_RATE_CM_S2,
            "wheel_speed_limit_cm_s": motor.config.max_wheel_speed_cm_s,
        }
        log.write("body_speed_limit_summary", **summary)
        print(summary)
    except Exception as error:
        status = "error"
        error_text = repr(error)
        raise
    finally:
        motor.hard_stop()
        motor.stop()
        if led is not None:
            led.set_idle()
        log.close(status, error_text)


if __name__ == "__main__":
    main()

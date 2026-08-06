"""地面负载 PWM 扫描，用于测量死区、前馈和轮速极限。

三个轮子接收相同占空比，因此车辆会原地旋转。倒计时后自动开始；断电即可
中止。仅在检查完上一轮结果后才可提高 DUTY_LEVELS。
"""

from motor import MotorSystem
from odometry import OdometrySystem
from calibration_store import CalibrationLog, DEFAULT_LOG_PATH, ticks_ms
from common import (
    automatic_countdown,
    linear_fit,
    log_sample,
    mean,
    sleep_ms,
    ticks_diff,
)


DUTY_LEVELS = (150, 220, 300, 400, 550, 750, 1000, 1400, 1900, 2500)
LEVEL_DURATION_MS = 1200
REST_DURATION_MS = 500
SAMPLE_PERIOD_MS = 40
STEADY_FRACTION = 0.45
MOVING_THRESHOLD_CM_S = 2.0
AUTO_START_DELAY_MS = 7000
LOG_PATH = DEFAULT_LOG_PATH


def rest(motor, duration_ms):
    motor.hard_stop()
    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < duration_ms:
        sleep_ms(10)


def run_level(motor, odometry, log, duty):
    stage = "open_loop_{:+d}".format(duty)
    start = ticks_ms()
    last_sample = start - SAMPLE_PERIOD_MS
    samples = [[], [], []]
    while ticks_diff(ticks_ms(), start) < LEVEL_DURATION_MS:
        motor.calibration_duty(duty, duty, duty)
        now = ticks_ms()
        if ticks_diff(now, last_sample) >= SAMPLE_PERIOD_MS:
            state = log_sample(
                log, stage, motor, odometry, {"applied_duty": duty}
            )
            elapsed = ticks_diff(now, start)
            if elapsed >= int(LEVEL_DURATION_MS * (1.0 - STEADY_FRACTION)):
                for index in range(3):
                    samples[index].append(state["wheel_speeds"][index])
            last_sample = now
        sleep_ms(10)
    motor.hard_stop()
    steady = [mean(values) for values in samples]
    log.write(
        "open_loop_level",
        stage=stage,
        applied_duty=duty,
        steady_wheel_speeds=steady,
    )
    return steady


def main():
    log = CalibrationLog("open_loop_wheel", LOG_PATH)
    odometry = OdometrySystem()
    motor = MotorSystem(odometry=odometry)
    status = "complete"
    error_text = None
    try:
        motor.start()
        motor.hard_stop()
        led = automatic_countdown(
            "Clear the ground area before the automatic PWM sweep.",
            AUTO_START_DELAY_MS,
        )

        results = []
        for direction in (1, -1):
            for magnitude in DUTY_LEVELS:
                duty = direction * magnitude
                steady = run_level(motor, odometry, log, duty)
                results.append((duty, steady))
                rest(motor, REST_DURATION_MS)

        stiction_positive = [None, None, None]
        stiction_negative = [None, None, None]
        suggested_stiction = [None, None, None]
        feedforward_positive = [None, None, None]
        feedforward_negative = [None, None, None]
        suggested_feedforward = [None, None, None]
        fitted_intercept = [None, None, None]
        max_measured_speed = [0.0, 0.0, 0.0]
        for wheel in range(3):
            speed_values = [[], []]
            duty_values = [[], []]
            for duty, speeds in results:
                speed = abs(speeds[wheel])
                max_measured_speed[wheel] = max(
                    max_measured_speed[wheel], speed
                )
                if speed >= MOVING_THRESHOLD_CM_S:
                    direction_index = 0 if duty > 0 else 1
                    speed_values[direction_index].append(speed)
                    duty_values[direction_index].append(abs(duty))
                    if duty > 0 and stiction_positive[wheel] is None:
                        stiction_positive[wheel] = abs(duty)
                    if duty < 0 and stiction_negative[wheel] is None:
                        stiction_negative[wheel] = abs(duty)

            positive_fit = linear_fit(speed_values[0], duty_values[0])
            negative_fit = linear_fit(speed_values[1], duty_values[1])
            feedforward_positive[wheel] = positive_fit[0]
            feedforward_negative[wheel] = negative_fit[0]
            available_stiction = [
                value
                for value in (
                    stiction_positive[wheel],
                    stiction_negative[wheel],
                )
                if value is not None
            ]
            suggested_stiction[wheel] = (
                max(available_stiction) if available_stiction else None
            )
            available_slopes = [
                value
                for value in (positive_fit[0], negative_fit[0])
                if value > 0.0
            ]
            suggested_feedforward[wheel] = (
                sum(available_slopes) / len(available_slopes)
                if available_slopes
                else None
            )
            available_intercepts = [
                value
                for value in (positive_fit[1], negative_fit[1])
                if value > 0.0
            ]
            fitted_intercept[wheel] = (
                sum(available_intercepts) / len(available_intercepts)
                if available_intercepts
                else None
            )

        log.write(
            "open_loop_summary",
            duty_levels=DUTY_LEVELS,
            moving_threshold_cm_s=MOVING_THRESHOLD_CM_S,
            stiction_duty_positive=stiction_positive,
            stiction_duty_negative=stiction_negative,
            suggested_stiction_duty=suggested_stiction,
            feedforward_positive=feedforward_positive,
            feedforward_negative=feedforward_negative,
            suggested_feedforward=suggested_feedforward,
            fitted_intercept_duty=fitted_intercept,
            max_measured_wheel_speed_cm_s=max_measured_speed,
        )
        print("stiction:", suggested_stiction)
        print("feedforward:", suggested_feedforward)
        print("max wheel speed:", max_measured_speed)
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

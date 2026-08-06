"""通过手动移动完整底盘测量轮子每米脉冲数。

测试在倒计时后自动开始，并在固定时间窗口内记录数据。将车恰好移动配置的
距离后保持不动，直至窗口结束。MOVEMENT_MODE="forward" 与
MOVEMENT_MODE="right" 各运行一次。
"""

from motor import MotorSystem, SQRT3_OVER_2
from calibration_store import CalibrationLog, DEFAULT_LOG_PATH, ticks_ms
from common import automatic_countdown, log_sample, sleep_ms, ticks_diff


MOVEMENT_MODE = "forward"  # "forward" or "right"; one direction per power-up
KNOWN_FORWARD_CM = 100.0
KNOWN_RIGHT_CM = 100.0
AUTO_START_DELAY_MS = 1000
MEASUREMENT_WINDOW_MS = 10000
LOG_PATH = DEFAULT_LOG_PATH


def measure_stage(motor, log, name):
    motor.hard_stop()
    motor.reset_encoder_totals()
    start = ticks_ms()
    last_log = start
    log.write("stage_start", stage=name)

    while ticks_diff(ticks_ms(), start) < MEASUREMENT_WINDOW_MS:
        now = ticks_ms()
        if ticks_diff(now, last_log) >= 50:
            log_sample(log, name, motor)
            last_log = now
        sleep_ms(10)

    totals = motor.get_encoder_counts()[1]
    log.write("stage_end", stage=name, encoder_totals=totals)
    return totals


def pulses_estimate(count, wheel_distance_cm):
    if abs(wheel_distance_cm) < 1.0e-9:
        return None
    return abs(float(count)) * 100.0 / abs(float(wheel_distance_cm))


def main():
    log = CalibrationLog("pulses_per_meter", LOG_PATH)
    motor = MotorSystem(odometry=None)
    status = "complete"
    error_text = None
    try:
        motor.start()
        motor.hard_stop()
        if MOVEMENT_MODE not in ("forward", "right"):
            raise ValueError("MOVEMENT_MODE must be 'forward' or 'right'")
        automatic_countdown(
            "Prepare to move the car {} manually by the known distance.".format(
                MOVEMENT_MODE
            ),
            AUTO_START_DELAY_MS,
        )
        counts = measure_stage(motor, log, "manual_" + MOVEMENT_MODE)

        if MOVEMENT_MODE == "forward":
            known_distance = KNOWN_FORWARD_CM
            wheel_distances = (
                -SQRT3_OVER_2 * known_distance,
                SQRT3_OVER_2 * known_distance,
                0.0,
            )
        else:
            known_distance = KNOWN_RIGHT_CM
            wheel_distances = (
                -0.5 * known_distance,
                -0.5 * known_distance,
                known_distance,
            )
        estimates = [
            pulses_estimate(counts[index], wheel_distances[index])
            for index in range(3)
        ]
        log.write(
            "pulses_trial",
            movement_mode=MOVEMENT_MODE,
            known_distance_cm=known_distance,
            encoder_totals=counts,
            expected_wheel_distances_cm=wheel_distances,
            pulses_per_meter_estimates=estimates,
        )
        log.write(
            "pulses_summary",
            movement_mode=MOVEMENT_MODE,
            suggested_pulses_per_meter=estimates,
        )
        print("Suggested pulses_per_meter from this direction:", estimates)
    except Exception as error:
        status = "error"
        error_text = repr(error)
        print(error_text)
        raise
    finally:
        motor.hard_stop()
        motor.stop()
        log.close(status, error_text)


if __name__ == "__main__":
    main()

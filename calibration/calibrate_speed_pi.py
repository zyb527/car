"""用于最终逐轮 PI 调参的闭环阶跃测试。

本测试有意使用 motor.command()，因此测得的瞬态响应属于轮速控制器而非
S 曲线。只有将每米脉冲数、死区和前馈填入 MotorConfig 后才可运行。
"""

from motor import MotorSystem
from odometry import OdometrySystem
from calibration_store import CalibrationLog, DEFAULT_LOG_PATH, ticks_ms
from common import (
    automatic_countdown,
    log_sample,
    mean,
    sleep_ms,
    ticks_diff,
)


# 标签、vx（cm/s）、vy（cm/s）、w（rad/s）、持续时间（ms）
PROFILES = (
    ("forward_30", 0.0, 30.0, 0.0, 1800),
    ("forward_60", 0.0, 60.0, 0.0, 1800),
    ("reverse_40", 0.0, -40.0, 0.0, 1800),
    ("right_35", 35.0, 0.0, 0.0, 1800),
    ("left_35", -35.0, 0.0, 0.0, 1800),
    ("rotate_ccw", 0.0, 0.0, 1.2, 1800),
    ("rotate_cw", 0.0, 0.0, -1.2, 1800),
)
REST_DURATION_MS = 700
SAMPLE_PERIOD_MS = 20
AUTO_START_DELAY_MS = 7000
LOG_PATH = DEFAULT_LOG_PATH


def wheel_metrics(samples, wheel_index):
    if not samples:
        return None
    targets = [item[1][wheel_index] for item in samples]
    measured = [item[2][wheel_index] for item in samples]
    target = mean(targets[-max(1, len(targets) // 4):])
    if abs(target) < 1.0:
        return None

    sign = 1.0 if target > 0.0 else -1.0
    normalized = [value * sign for value in measured]
    target_abs = abs(target)
    rise_time_ms = None
    for elapsed_ms, _, speeds in samples:
        if speeds[wheel_index] * sign >= 0.9 * target_abs:
            rise_time_ms = elapsed_ms
            break
    peak = max(normalized)
    tail = normalized[-max(1, len(normalized) // 4):]
    steady = mean(tail)
    return {
        "target_cm_s": target,
        "rise_time_ms": rise_time_ms,
        "overshoot_percent": max(0.0, (peak / target_abs - 1.0) * 100.0),
        "steady_error_cm_s": target_abs - steady,
        "steady_error_percent": (target_abs - steady) * 100.0 / target_abs,
        "peak_cm_s": peak,
    }


def run_stage(motor, odometry, log, profile):
    label, vx, vy, w, duration_ms = profile
    start = ticks_ms()
    last_sample = start - SAMPLE_PERIOD_MS
    samples = []
    while ticks_diff(ticks_ms(), start) < duration_ms:
        motor.command(vx, vy, w)
        now = ticks_ms()
        if ticks_diff(now, last_sample) >= SAMPLE_PERIOD_MS:
            state = log_sample(log, label, motor, odometry)
            samples.append(
                (
                    ticks_diff(now, start),
                    state["target_wheels"],
                    state["wheel_speeds"],
                )
            )
            last_sample = now
        sleep_ms(5)

    motor.hard_stop()
    metrics = [wheel_metrics(samples, index) for index in range(3)]
    log.write(
        "pi_stage_summary",
        stage=label,
        body_command=(vx, vy, w),
        wheel_metrics=metrics,
    )
    return metrics


def main():
    log = CalibrationLog("speed_pi", LOG_PATH)
    odometry = OdometrySystem()
    motor = MotorSystem(odometry=odometry)
    status = "complete"
    error_text = None
    try:
        motor.start()
        motor.hard_stop()
        led = automatic_countdown(
            "Clear the test area before automatic PI step tests.",
            AUTO_START_DELAY_MS,
        )
        for profile in PROFILES:
            metrics = run_stage(motor, odometry, log, profile)
            print(profile[0], metrics)
            start = ticks_ms()
            while ticks_diff(ticks_ms(), start) < REST_DURATION_MS:
                sleep_ms(10)
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

"""自动单方向里程计与旋转测试。

本车没有 GPIO 启动/停止开关。选择一个 TEST_MODE 后上电，等待倒计时并让
距离/角度测试完成。直线模式会在估计距离达到 TARGET_DISTANCE_CM 时停车；
旋转模式保持固定时长。之后测量实际结果，并记录以更新下一轮比例参数。
"""

from motor import MotorSystem
from odometry import OdometrySystem, normalize_angle
from calibration_store import CalibrationLog, DEFAULT_LOG_PATH, ticks_ms
from common import (
    automatic_countdown,
    log_sample,
    sleep_ms,
    ticks_diff,
)


TEST_MODE = "forward"  # "forward", "right" or "rotate"; one per power-up
COMMANDS = {
    "forward": (0.0, 50.0, 0.0),
    "right": (120.0, 0.0, 0.0),
    "rotate": (0.0, 0.0, 0.7),
}
# 直线航向保持：w = -KP * 航向误差 - KD * 偏航角速度。
# 首次实车测试仍出现 8--9 度偏航后已提高。
HEADING_HOLD_KP = 6.0
HEADING_HOLD_KD = 0.35
HEADING_HOLD_MAX_W_RAD_S = 1.2
# 最终停车阶段允许存在小的残余姿态误差，而非对抗静摩擦保持底盘不动直至
# STOP_TIMEOUT_MS 超时。
STOP_HEADING_TOLERANCE_RAD = 0.08
TARGET_DISTANCE_CM = 100.0
# 修正后的前移在 100 cm 触发后仍滑行 16.6 cm。提前开始软停车；右移参数
# 暂不调整，直到其行驶得更直。
STOP_EARLY_DISTANCE_CM = {
    "forward": 17.0,
# 当前右移速度为 120 cm/s，首次测试在触发后滑行约 43 cm（原始里程计）。
# 配合 1.255 横移比例，修正距离的提前停车余量为 55 cm。
    "right": 55.0,
}
MAX_RUN_DURATION_MS = {
    "forward": 8000,
    "right": 8000,
    "rotate": 8500,
}
AUTO_START_DELAY_MS = 3000
SAMPLE_PERIOD_MS = 40
STOP_TIMEOUT_MS = 4000
LOG_PATH = DEFAULT_LOG_PATH


def heading_hold_command(odometry, heading_target, rotate_mode, fallback_w):
    """返回主动行驶及制动阶段使用的（航向误差，w 指令）。"""
    if rotate_mode:
        return 0.0, fallback_w
    heading_state = odometry.get_state()
    heading_error_rad = normalize_angle(
        heading_state["heading_rad"] - heading_target
    )
    heading_hold_w = -(
        HEADING_HOLD_KP * heading_error_rad
        + HEADING_HOLD_KD * heading_state["yaw_rate_rad_s"]
    )
    heading_hold_w = max(
        -HEADING_HOLD_MAX_W_RAD_S,
        min(HEADING_HOLD_MAX_W_RAD_S, heading_hold_w),
    )
    return heading_error_rad, heading_hold_w


def run_trial(motor, odometry, log, mode):
    command = COMMANDS[mode]
    max_duration_ms = MAX_RUN_DURATION_MS[mode]
    odometry.set_pose(0.0, 0.0, 0.0)
    start = ticks_ms()
    last_time = start
    last_sample = start - SAMPLE_PERIOD_MS
    integrated_command_w = 0.0
    heading_target = odometry.get_pose()[2]
    maximum_heading_error_rad = 0.0
    log.write(
        "stage_start",
        stage="odometry_" + mode,
        body_command=command,
        target_distance_cm=(TARGET_DISTANCE_CM if mode != "rotate" else None),
        stop_trigger_distance_cm=(
            TARGET_DISTANCE_CM - STOP_EARLY_DISTANCE_CM.get(mode, 0.0)
            if mode != "rotate"
            else None
        ),
        max_duration_ms=max_duration_ms,
        heading_target_rad=heading_target if mode != "rotate" else None,
    )

    reached_target = False
    while ticks_diff(ticks_ms(), start) < max_duration_ms:
        if mode == "forward":
            progress_cm = abs(odometry.get_pose()[0])
        elif mode == "right":
            progress_cm = abs(odometry.get_pose()[1])
        else:
            progress_cm = 0.0
        stop_trigger_cm = TARGET_DISTANCE_CM - STOP_EARLY_DISTANCE_CM.get(
            mode, 0.0
        )
        if mode != "rotate" and progress_cm >= stop_trigger_cm:
            reached_target = True
            break
        heading_error_rad, heading_hold_w = heading_hold_command(
            odometry, heading_target, mode == "rotate", command[2]
        )
        maximum_heading_error_rad = max(
            maximum_heading_error_rad, abs(heading_error_rad)
        )
        motor.move(command[0], command[1], heading_hold_w)
        now = ticks_ms()
        dt = max(0.0, ticks_diff(now, last_time) / 1000.0)
        integrated_command_w += motor.get_limited_command()[2] * dt
        last_time = now
        if ticks_diff(now, last_sample) >= SAMPLE_PERIOD_MS:
            log_sample(
                log,
                "odometry_" + mode,
                motor,
                odometry,
                {
                    "integrated_command_w_rad": integrated_command_w,
                    "heading_target_rad": heading_target,
                    "heading_error_rad": heading_error_rad,
                    "heading_hold_w_rad_s": heading_hold_w,
                },
            )
            last_sample = now
        sleep_ms(5)

    # 此处不可调用 soft_stop()：它会将 w 缓慢降至零，恰好在滑行可能使底盘
    # 转动的阶段放弃航向保持。应持续发送零平移加 IMU 航向修正。
    stop_start = ticks_ms()
    while True:
        now = ticks_ms()
        if ticks_diff(now, stop_start) >= STOP_TIMEOUT_MS:
            motor.hard_stop()
            break
        heading_error_rad, heading_hold_w = heading_hold_command(
            odometry, heading_target, mode == "rotate", 0.0
        )
        maximum_heading_error_rad = max(
            maximum_heading_error_rad, abs(heading_error_rad)
        )
        motor.move(0.0, 0.0, heading_hold_w)
        dt = max(0.0, ticks_diff(now, last_time) / 1000.0)
        integrated_command_w += motor.get_limited_command()[2] * dt
        last_time = now
        state = motor.get_state()
        limited = state["limited_body"]
        stopped = (
            max(abs(speed) for speed in state["wheel_speeds"])
            < motor.config.stopped_speed_cm_s
            and abs(limited[0]) < 0.5
            and abs(limited[1]) < 0.5
            and abs(heading_error_rad) < STOP_HEADING_TOLERANCE_RAD
        )
        if stopped:
            motor.hard_stop()
            break
        if ticks_diff(now, last_sample) >= SAMPLE_PERIOD_MS:
            log_sample(
                log,
                "odometry_" + mode + "_soft_stop",
                motor,
                odometry,
                {
                    "integrated_command_w_rad": integrated_command_w,
                    "heading_target_rad": heading_target,
                    "heading_error_rad": heading_error_rad,
                    "heading_hold_w_rad_s": heading_hold_w,
                },
            )
            last_sample = now
        sleep_ms(5)

    # 让编码器积分包含剩余的机械滑行。
    sleep_ms(500)
    state = odometry.get_state()
    trial = {
        "mode": mode,
        "body_command": command,
        "run_duration_ms": ticks_diff(ticks_ms(), start),
        "target_distance_cm": TARGET_DISTANCE_CM if mode != "rotate" else None,
        "stop_trigger_distance_cm": (
            TARGET_DISTANCE_CM - STOP_EARLY_DISTANCE_CM.get(mode, 0.0)
            if mode != "rotate"
            else None
        ),
        "reached_target_distance": reached_target if mode != "rotate" else None,
        "reported_x_cm": state["x_cm"],
        "reported_y_cm": state["y_cm"],
        "reported_heading_rad": state["heading_rad"],
        "reported_heading_unwrapped_rad": state["heading_unwrapped_rad"],
        "integrated_command_angle_rad": integrated_command_w,
        "heading_target_rad": heading_target if mode != "rotate" else None,
        "maximum_heading_error_rad": (
            maximum_heading_error_rad if mode != "rotate" else None
        ),
        "heading_hold_kp": HEADING_HOLD_KP if mode != "rotate" else None,
        "heading_hold_kd": HEADING_HOLD_KD if mode != "rotate" else None,
        "current_forward_distance_scale": (
            odometry.config.forward_distance_scale
        ),
        "current_lateral_distance_scale": (
            odometry.config.lateral_distance_scale
        ),
        "current_gyro_scale_raw": odometry.config.gyro_scale_raw,
        "yaw_raw_axis_index": odometry.config.axis_indices[2],
        "current_rotation_gain": motor.config.rotation_gain,
    }
    log.write("odometry_trial", **trial)
    return trial


def main():
    log = CalibrationLog("odometry_scale_" + TEST_MODE, LOG_PATH)
    odometry = OdometrySystem()
    motor = MotorSystem(odometry=odometry)
    status = "complete"
    error_text = None
    try:
        if TEST_MODE not in COMMANDS:
            raise ValueError("TEST_MODE must be forward, right or rotate")
        motor.start()
        motor.hard_stop()
        led = automatic_countdown(
            "Automatic {} odometry trial will start.".format(TEST_MODE),
            AUTO_START_DELAY_MS,
        )
        trial = run_trial(motor, odometry, log, TEST_MODE)
        print(trial)
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

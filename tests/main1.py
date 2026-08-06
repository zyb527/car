"""主车底盘基础动作测试：前后、左右横移、顺逆时针原地自转。

不启用摄像头、ToF 或正式任务状态机。运行期间主车仍以 10 ms 节拍向
辅助车发送实际底盘命令；任意异常或 Ctrl-C 都会让两车停止。
"""

import time
import math

from main_config import MissionConfig
from motor import MotorSystem
from odometry import OdometrySystem
from wireless_feedforward import FeedforwardSender


# move()/command() 的 vx、vy 单位为实际 cm/s；w 单位为 rad/s。
LINEAR_SPEED_CM_S = 100.0
ROTATE_SPEED_RAD_S = 0.8
STAGE_DURATION_MS = 2000
PAUSE_DURATION_MS = 1000
START_DELAY_MS = 3000

STAGES = (
    ("FORWARD", (0.0, LINEAR_SPEED_CM_S, 0.0)),
    ("BACKWARD", (0.0, -LINEAR_SPEED_CM_S, 0.0)),
    ("RIGHT", (LINEAR_SPEED_CM_S, 0.0, 0.0)),
    ("LEFT", (-LINEAR_SPEED_CM_S, 0.0, 0.0)),
    (
        "RIGHT_FORWARD",
        (
            LINEAR_SPEED_CM_S / math.sqrt(2.0),
            LINEAR_SPEED_CM_S / math.sqrt(2.0),
            0.0,
        ),
    ),
    ("CLOCKWISE", (0.0, 0.0, -ROTATE_SPEED_RAD_S)),
    ("COUNTERCLOCKWISE", (0.0, 0.0, ROTATE_SPEED_RAD_S)),
)


def _ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.monotonic() * 1000.0)


def _ticks_diff(new_value, old_value):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(new_value, old_value)
    return new_value - old_value


def _sleep_ms(milliseconds):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(int(milliseconds))
    else:
        time.sleep(milliseconds / 1000.0)


def _send_zero_frames(sender, count=5):
    for _ in range(count):
        try:
            sender.send(0.0, 0.0, 0.0)
        except Exception:
            return
        _sleep_ms(MissionConfig.FEEDFORWARD_TX_PERIOD_MS)


def _is_translation_without_rotation(command):
    """任务要求平移且明确给出 w=0 时返回 True。"""
    vx = float(command[0])
    vy = float(command[1])
    w = float(command[2])
    return w == 0.0 and (vx != 0.0 or vy != 0.0)


def _run_interval(motor, sender, command, duration_ms, label):
    start_ms = _ticks_ms()
    last_control_ms = start_ms - MissionConfig.CONTROL_PERIOD_MS
    last_tx_ms = start_ms - MissionConfig.FEEDFORWARD_TX_PERIOD_MS
    straight_without_w = _is_translation_without_rotation(command)
    print(label, command, "duration_ms=", duration_ms)

    while _ticks_diff(_ticks_ms(), start_ms) < duration_ms:
        now_ms = _ticks_ms()
        if (
            _ticks_diff(now_ms, last_control_ms)
            >= MissionConfig.CONTROL_PERIOD_MS
        ):
            motor.move(command[0], command[1], command[2])
            last_control_ms = now_ms
        if (
            _ticks_diff(now_ms, last_tx_ms)
            >= MissionConfig.FEEDFORWARD_TX_PERIOD_MS
        ):
            sender.send_motor_command(
                motor,
                straight_without_w=straight_without_w,
            )
            last_tx_ms = now_ms
        _sleep_ms(1)


def main():
    motor = None
    sender = None
    try:
        odometry = OdometrySystem()
        motor = MotorSystem(odometry=odometry)
        sender = FeedforwardSender()
        motor.start()
        motor.hard_stop()

        # 必须在 IMU 初始化完成后归零，避免测试航向受初始化时序影响。
        odometry.set_pose(0.0, 0.0, 0.0)
        print("main1: clear the area; starting in", START_DELAY_MS, "ms")
        _sleep_ms(START_DELAY_MS)

        for index, (label, command) in enumerate(STAGES):
            _run_interval(motor, sender, command, STAGE_DURATION_MS, label)
            motor.hard_stop()
            _send_zero_frames(sender)
            if index + 1 < len(STAGES):
                print("PAUSE", PAUSE_DURATION_MS, "ms")
                _sleep_ms(PAUSE_DURATION_MS)

        print("main1 complete")
    except KeyboardInterrupt:
        print("main1 stopped")
    except Exception as error:
        print("main1 error:", repr(error))
        try:
            import sys

            sys.print_exception(error)
        except Exception:
            pass
    finally:
        if motor is not None:
            motor.hard_stop()
        if sender is not None:
            _send_zero_frames(sender)
        if motor is not None:
            motor.stop()


if __name__ == "__main__":
    main()

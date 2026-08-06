"""主车固定半径连续绕行测试。

把目标物体放在主车正前方约 300 mm 处。程序识别到目标后，以主车旋转
中心到物体 300 mm 为固定半径连续绕行，并持续向辅助车发送实际底盘前馈。

本测试不导入、不初始化 ToF。摄像头负责目标横向居中，同时锁定首次识别
目标时的图像 y 坐标，用作无 ToF 条件下的径向漂移修正参考。目标丢失或
摄像头超时后立即停车；重新识别目标后会在当前位置重新锁定并继续测试。
Ctrl-C 可随时结束，两车都会收到停车命令。
"""

import time

from control import MotionStep, PIDController, finite
from main_config import MissionConfig, OrbitConfig
from motor import MotorSystem
from odometry import OdometrySystem
from orbit import calc_orbit_command, get_orbit_direction
from vision import VisionReceiver
from wireless_feedforward import FeedforwardSender


class FixedRadiusOrbitConfig(OrbitConfig):
    """只作用于 main_orbit.py 的测试参数。"""

    # 这里的半径直接定义为物体到主车旋转中心的距离。
    ORBIT_RADIUS_MM = 300.0
    ORBIT_DIRECTION = "left"
    ORBIT_ROTATION_SPEED_RAD_S = 1.6
    TOF_CENTER_OFFSET_MM = 0.0

    # 当前底盘 move() 的线速度单位已经是 cm/s。测试按 v=wR 生成切向
    # 速度，不沿用旧底盘的 8.1 倍旋转换算。
    LEGACY_ROTATION_GAIN = 1.0
    ORBIT_RADIUS_TO_SPEED_SCALE = 1.0
    LINEAR_SPEED_SCALE = 1.0

    # 本测试没有 ToF 数据，也不启用依赖 ToF 的半径安全带。
    ORBIT_TOF_WEIGHT = 0.0
    ORBIT_BAND_VY_ENABLED = False


START_DELAY_MS = MissionConfig.START_DELAY_MS
DEBUG_PRINT_PERIOD_MS = 500


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


def _target_xy(target):
    if target is None or not bool(target.get("found", False)):
        return None
    try:
        x = float(target.get("x"))
        y = float(target.get("y"))
    except (TypeError, ValueError):
        return None
    if not finite(x) or not finite(y):
        return None
    return x, y


class FixedRadiusOrbitController:
    """不依赖硬件的连续绕行控制器，便于桌面验证。"""

    def __init__(self, config=FixedRadiusOrbitConfig):
        self.config = config
        self.direction = get_orbit_direction(config)
        self.camera_turn_pid = PIDController(
            config.PID_CAMERA_TURN_KP,
            config.PID_CAMERA_TURN_KI,
            config.PID_CAMERA_TURN_KD,
            output_limit=config.PID_CAMERA_TURN_OUTPUT_LIMIT,
            integral_limit=config.PID_CAMERA_TURN_I_LIMIT,
        )
        # calc_orbit_command 保留一个距离 PID 形参；这里传入恒为零的占位
        # 控制器，且 tof_found 始终为 False，不连接任何测距硬件或数据。
        self.disabled_distance_pid = PIDController(
            0.0,
            output_limit=0.0,
        )
        self.y_pid = PIDController(
            config.PID_ORBIT_Y_KP,
            config.PID_ORBIT_Y_KI,
            config.PID_ORBIT_Y_KD,
            output_limit=config.ORBIT_MAX_VY_CM_S,
            integral_limit=config.PID_ORBIT_Y_I_LIMIT,
        )
        self.orbit_target_y = None

    def reset(self):
        self.camera_turn_pid.reset()
        self.disabled_distance_pid.reset()
        self.y_pid.reset()
        self.orbit_target_y = None

    def step(self, target, dt=0.02):
        target_xy = _target_xy(target)
        if target_xy is None:
            self.reset()
            return MotionStep.stop("orbit_wait_target")

        target_x, target_y = target_xy
        if self.orbit_target_y is None:
            # 用户把物体放在约 300 mm 处后，以首次图像位置作为该半径的
            # 视觉参考。它只能抑制漂移，不能替代绝对距离测量。
            self.orbit_target_y = target_y

        command = calc_orbit_command(
            target_x,
            target_y,
            False,
            0.0,
            self.config.ORBIT_RADIUS_MM,
            self.orbit_target_y,
            self.camera_turn_pid,
            self.disabled_distance_pid,
            self.y_pid,
            dt,
            self.config,
            heading_error=None,
            direction=self.direction,
        )
        return MotionStep(
            command,
            reason="fixed_radius_orbit",
            debug={
                "direction": self.direction,
                "orbit_radius_mm": self.config.ORBIT_RADIUS_MM,
                "target_x_px": target_x,
                "target_y_px": target_y,
                "orbit_target_y_px": self.orbit_target_y,
            },
        )


def _send_zero_frames(sender, count=5):
    if sender is None:
        return
    for _ in range(int(count)):
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


def _send_motor_feedforward(sender, motor, task_command):
    """仅在任务要求无旋转平移时屏蔽主车本地航向修正。"""
    return sender.send_motor_command(
        motor,
        straight_without_w=_is_translation_without_rotation(task_command),
    )


def _startup_hold(sender):
    start_ms = _ticks_ms()
    last_tx_ms = start_ms - MissionConfig.FEEDFORWARD_TX_PERIOD_MS
    while _ticks_diff(_ticks_ms(), start_ms) < START_DELAY_MS:
        now_ms = _ticks_ms()
        if (
            _ticks_diff(now_ms, last_tx_ms)
            >= MissionConfig.FEEDFORWARD_TX_PERIOD_MS
        ):
            sender.send(0.0, 0.0, 0.0)
            last_tx_ms = now_ms
        _sleep_ms(1)


def main():
    motor = None
    sender = None
    try:
        # 故意不导入也不创建 ToFSensor。
        odometry = OdometrySystem()
        motor = MotorSystem(odometry=odometry)
        sender = FeedforwardSender()
        vision = VisionReceiver(
            uart_id=MissionConfig.MAIN_CAMERA_UART_ID,
            baud=MissionConfig.MAIN_CAMERA_BAUD,
            timeout_ms=MissionConfig.MAIN_CAMERA_TIMEOUT_MS,
        )
        controller = FixedRadiusOrbitController()

        motor.start()
        motor.hard_stop()
        odometry.set_pose(0.0, 0.0, 0.0)
        vision.set_target_filter(0)

        print(
            "main_orbit: place object 300 mm ahead; direction=",
            controller.direction,
            "start_delay_ms=",
            START_DELAY_MS,
        )
        _startup_hold(sender)
        print("main_orbit: waiting for visual target")

        now_ms = _ticks_ms()
        last_control_ms = now_ms - MissionConfig.CONTROL_PERIOD_MS
        last_tx_ms = now_ms - MissionConfig.FEEDFORWARD_TX_PERIOD_MS
        last_debug_ms = now_ms - DEBUG_PRINT_PERIOD_MS
        last_reason = None
        feedforward_task_command = (0.0, 0.0, 0.0)

        while True:
            now_ms = _ticks_ms()
            result = None

            if (
                _ticks_diff(now_ms, last_control_ms)
                >= MissionConfig.CONTROL_PERIOD_MS
            ):
                dt_ms = _ticks_diff(now_ms, last_control_ms)
                last_control_ms = now_ms
                dt = max(0.001, min(dt_ms / 1000.0, 0.1))

                vision.poll(now_ms)
                target, _ = vision.get_data()
                result = controller.step(target, dt=dt)

                if result.reason == "fixed_radius_orbit":
                    motor.move(*result.command)
                    feedforward_task_command = result.command
                else:
                    motor.hard_stop()
                    feedforward_task_command = (0.0, 0.0, 0.0)

                if result.reason != last_reason:
                    print("main_orbit state:", result.reason)
                    last_reason = result.reason

                if (
                    result.reason == "fixed_radius_orbit"
                    and _ticks_diff(now_ms, last_debug_ms)
                    >= DEBUG_PRINT_PERIOD_MS
                ):
                    print(
                        "orbit target=",
                        result.debug["target_x_px"],
                        result.debug["target_y_px"],
                        "locked_y=",
                        result.debug["orbit_target_y_px"],
                        "command=",
                        result.command,
                    )
                    last_debug_ms = now_ms

            if (
                _ticks_diff(now_ms, last_tx_ms)
                >= MissionConfig.FEEDFORWARD_TX_PERIOD_MS
            ):
                _send_motor_feedforward(
                    sender,
                    motor,
                    feedforward_task_command,
                )
                last_tx_ms = now_ms

            _sleep_ms(1)

    except KeyboardInterrupt:
        print("main_orbit stopped")
    except Exception as error:
        print("main_orbit error:", repr(error))
        try:
            import sys

            sys.print_exception(error)
        except Exception:
            pass
    finally:
        if motor is not None:
            motor.hard_stop()
        _send_zero_frames(sender)
        if motor is not None:
            motor.stop()


if __name__ == "__main__":
    main()

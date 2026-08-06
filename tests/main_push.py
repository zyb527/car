"""主车推行与避障的双车跟随测试入口。

将待推物体放在两车推杆前方，主摄能够同时识别该目标及危险障碍/黄线。
识别到目标后，程序按 PushController 的正式推行、避障、返航和黄线停车逻辑
运行一次。主车每 10 ms 向辅助车发送已限幅的实际速度前馈。

本测试不导入、不初始化 ToF：当前 PushController 的 tof 形参尚未参与推行
或避障决策。目标丢失、黄线结束、推行超时、异常或 Ctrl-C 都会停车。
"""

import math
import time

from control import MotionStep, normalize_angle
from main_config import MissionConfig, PushConfig
from motor import MotorSystem
from odometry import OdometrySystem
from push import PushController, State
from vision import VisionReceiver
from wireless_feedforward import FeedforwardSender


WAIT_TARGET = "WAIT_TARGET"
PUSHING = "PUSHING"
FINISHED = "FINISHED"
FAULT = "FAULT"

START_DELAY_MS = MissionConfig.START_DELAY_MS
DEBUG_PRINT_PERIOD_MS = 250


class PushAvoidanceTestConfig(PushConfig):
    """main_push 专用的确定性 45°避障动作测试参数。"""

    FORCED_AVOIDANCE_ENABLED = True
    FORCED_AVOIDANCE_DELAY_S = 0.7
    FORCED_AVOIDANCE_ANGLE_RAD = math.radians(45.0)
    FORCED_AVOIDANCE_REACHED_TOLERANCE_RAD = math.radians(2.0)
    # 到达45°后立即转回，不保留正式避障中的 0.5 s 清障等待。
    AVOID_CLEAR_HOLD_S = 0.0
    # 给“推行 -> 45° -> 回正”完整测试留足时间；回正后会主动结束，
    # 不会实际运行到这个超时。
    PUSH_DURATION_S = 15.0


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


def _target_class_id(target):
    if target is None or not bool(target.get("found", False)):
        return 0
    try:
        return int(target.get("class_id", 0) or 0)
    except (TypeError, ValueError):
        return 0


def _target_available(target):
    return target is not None and bool(target.get("found", False))


def _target_xy(target):
    if not _target_available(target):
        return None
    try:
        return float(target.get("x")), float(target.get("y"))
    except (TypeError, ValueError):
        return None


class InitialTargetPushConfig:
    """只覆盖本次推行的目标像素，其余参数继承基础配置。"""

    def __init__(self, base_config, target_x_px, target_y_px):
        self._base_config = base_config
        self.TARGET_CENTER_X_PX = float(target_x_px)
        self.TARGET_Y_PX = float(target_y_px)

    def __getattr__(self, name):
        return getattr(self._base_config, name)


def _is_translation_without_rotation(command):
    """任务要求平移且明确给出 w=0 时返回 True。"""
    vx = float(command[0])
    vy = float(command[1])
    w = float(command[2])
    return w == 0.0 and (vx != 0.0 or vy != 0.0)


def _send_motor_feedforward(sender, motor, task_command):
    """发送主车实际速度；兼容旧版无线发送器。"""
    straight_without_w = _is_translation_without_rotation(task_command)
    try:
        return sender.send_motor_command(
            motor,
            straight_without_w=straight_without_w,
        )
    except TypeError:
        # 旧版 send_motor_command 只有 motor 参数。保留实际 vx/vy；直推时
        # 仍屏蔽主车本地航向保持 w，避障转向时发送实际 w。
        if hasattr(motor, "get_limited_physical_command"):
            command = motor.get_limited_physical_command()
        else:
            command = motor.get_limited_command()
        if straight_without_w:
            command = (command[0], command[1], 0.0)
        sender.send(command[0], command[1], command[2])
        return command


class PushTestSession:
    """把一次正式 PushController 运行包装为安全、可测试的会话。"""

    def __init__(self, config=PushConfig):
        self.base_config = config
        self.config = config
        self.controller = PushController(config)
        self.state = WAIT_TARGET
        self.last_reason = "push_wait_target"
        self.test_elapsed_s = 0.0
        self.forced_phase = "DISABLED"
        self.original_heading_rad = 0.0
        self.initial_target_x_px = None
        self.initial_target_y_px = None

    def reset(self):
        self.config = self.base_config
        self.controller = PushController(self.config)
        self.state = WAIT_TARGET
        self.last_reason = "push_wait_target"
        self.test_elapsed_s = 0.0
        self.forced_phase = "DISABLED"
        self.original_heading_rad = 0.0
        self.initial_target_x_px = None
        self.initial_target_y_px = None

    def _yellow_hazard(self, hazard):
        if hazard is None or not bool(hazard.get("found", False)):
            return None
        try:
            kind = int(hazard.get("type", hazard.get("hazard_type", 0)) or 0)
        except (TypeError, ValueError):
            return None
        if kind == int(getattr(self.config, "HAZARD_YELLOW", 2)):
            return hazard
        return None

    def _forced_hazard(self, hazard, heading_rad):
        """为动作回归测试生成一次近距离45°避障事件。"""
        if not bool(getattr(self.config, "FORCED_AVOIDANCE_ENABLED", False)):
            return hazard

        yellow_hazard = self._yellow_hazard(hazard)
        if yellow_hazard is not None:
            return yellow_hazard

        if self.forced_phase == "STRAIGHT":
            if self.test_elapsed_s >= float(
                self.config.FORCED_AVOIDANCE_DELAY_S
            ):
                self.forced_phase = "OUTBOUND"
            else:
                # 忽略普通视觉障碍，确保测试动作由固定0.7秒时刻触发。
                return None

        if self.forced_phase == "OUTBOUND":
            delta = abs(
                normalize_angle(float(heading_rad) - self.original_heading_rad)
            )
            reached = delta >= (
                float(self.config.FORCED_AVOIDANCE_ANGLE_RAD)
                - float(self.config.FORCED_AVOIDANCE_REACHED_TOLERANCE_RAD)
            )
            if reached:
                self.forced_phase = "RETURN"
                # 给控制器一个明确的“已清除障碍”帧，触发回正状态。
                return {"found": False, "type": 0, "x": 0.0, "y": 0.0}

            center_x = float(self.config.AVOID_CENTER_X_PX)
            deadband = float(self.config.AVOID_CENTER_DEADBAND_PX)
            near_y = float(self.config.AVOID_Y_NEAR_PX)
            # 障碍在右侧 -> 按现有逻辑向左避让；y 处在 near 档，避障角为45°。
            return {
                "found": True,
                "type": int(self.config.HAZARD_OBSTACLE),
                "x": center_x + deadband + 1.0,
                "y": near_y + 4.0,
            }

        if self.forced_phase == "RETURN":
            return {"found": False, "type": 0, "x": 0.0, "y": 0.0}
        return None

    def step(self, target, hazard, heading_rad, dt=0.02):
        if self.state == WAIT_TARGET:
            initial_target = _target_xy(target)
            if initial_target is None:
                return MotionStep.stop("push_wait_target")
            self.initial_target_x_px = initial_target[0]
            self.initial_target_y_px = initial_target[1]
            self.config = InitialTargetPushConfig(
                self.base_config,
                self.initial_target_x_px,
                self.initial_target_y_px,
            )
            self.controller = PushController(self.config)
            self.controller.start(
                float(heading_rad),
                class_id=_target_class_id(target),
            )
            self.state = PUSHING
            self.original_heading_rad = float(heading_rad)
            self.forced_phase = (
                "STRAIGHT"
                if bool(
                    getattr(self.config, "FORCED_AVOIDANCE_ENABLED", False)
                )
                else "DISABLED"
            )

        if self.state == PUSHING:
            self.test_elapsed_s += float(dt)
            controller_hazard = self._forced_hazard(hazard, heading_rad)
            # PushController 当前没有使用 tof；明确传 None，避免初始化硬件。
            result = self.controller.step(
                target,
                None,
                float(heading_rad),
                hazard=controller_hazard,
                dt=float(dt),
            )
            self.last_reason = result.reason
            if result.failed:
                self.state = FAULT
            elif result.done:
                self.state = FINISHED
            elif (
                self.forced_phase == "RETURN"
                and self.controller.state == State.PUSH_NORMAL
            ):
                self.forced_phase = "COMPLETE"
                self.state = FINISHED
                return MotionStep.stop(
                    "push_forced_avoidance_complete",
                    done=True,
                    debug={
                        "forced_avoidance_angle_deg": 45.0,
                        "elapsed_s": self.test_elapsed_s,
                    },
                )
            return result

        if self.state == FINISHED:
            return MotionStep.stop("push_test_finished", done=True)
        return MotionStep.stop("push_test_fault", failed=True)


def _send_zero_frames(sender, count=5):
    if sender is None:
        return
    for _ in range(int(count)):
        try:
            sender.send(0.0, 0.0, 0.0)
        except Exception:
            return
        _sleep_ms(MissionConfig.FEEDFORWARD_TX_PERIOD_MS)


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
        session = PushTestSession(PushAvoidanceTestConfig)

        motor.start()
        motor.hard_stop()
        odometry.set_pose(0.0, 0.0, 0.0)
        vision.set_target_filter(0)

        print(
            "main_push: place object at the push rods; waiting",
            START_DELAY_MS,
            "ms before target acquisition",
        )
        _startup_hold(sender)
        print("main_push: waiting for visual target")

        now_ms = _ticks_ms()
        last_control_ms = now_ms - MissionConfig.CONTROL_PERIOD_MS
        last_tx_ms = now_ms - MissionConfig.FEEDFORWARD_TX_PERIOD_MS
        last_debug_ms = now_ms - DEBUG_PRINT_PERIOD_MS
        last_reason = None
        feedforward_task_command = (0.0, 0.0, 0.0)

        while True:
            now_ms = _ticks_ms()
            if (
                _ticks_diff(now_ms, last_control_ms)
                >= MissionConfig.CONTROL_PERIOD_MS
            ):
                dt_ms = _ticks_diff(now_ms, last_control_ms)
                last_control_ms = now_ms
                dt = max(0.001, min(dt_ms / 1000.0, 0.1))

                vision.poll(now_ms)
                target, hazard = vision.get_data()
                heading_rad = odometry.get_pose()[2]
                result = session.step(target, hazard, heading_rad, dt=dt)

                if result.failed or result.done:
                    motor.hard_stop()
                    feedforward_task_command = (0.0, 0.0, 0.0)
                elif session.state == PUSHING:
                    motor.move(*result.command)
                    feedforward_task_command = result.command
                else:
                    motor.hard_stop()
                    feedforward_task_command = (0.0, 0.0, 0.0)

                if result.reason != last_reason:
                    print(
                        "main_push state:",
                        session.state,
                        "reason:",
                        result.reason,
                    )
                    last_reason = result.reason

                if (
                    session.state == PUSHING
                    and _ticks_diff(now_ms, last_debug_ms)
                    >= DEBUG_PRINT_PERIOD_MS
                ):
                    print(
                        "push command=",
                        result.command,
                        "phase=",
                        session.forced_phase,
                        "initial_target=",
                        session.initial_target_x_px,
                        session.initial_target_y_px,
                        "target=",
                        target,
                        "hazard=",
                        hazard,
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

            if session.state in (FINISHED, FAULT):
                break
            _sleep_ms(1)

    except KeyboardInterrupt:
        print("main_push stopped")
    except Exception as error:
        print("main_push error:", repr(error))
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

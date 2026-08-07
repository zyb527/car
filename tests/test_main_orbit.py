"""正式 OrbitController 的主车实车调参测试。

把目标放在主车前方准备入轨的位置。本文件直接使用正式 orbit.py 中的
OrbitController，并按正式主流程传入视觉、ToF、IMU 航向和角速度，完整
运行 ORBITING、ALIGN、CLOSE_IN 三个阶段。
"""

import math
import os
import sys
import time


try:
    PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if PROJECT_DIR not in sys.path:
        sys.path.insert(0, PROJECT_DIR)
except Exception:
    if ".." not in sys.path:
        sys.path.append("..")
    if "/" not in sys.path:
        sys.path.append("/")

from main_config import MissionConfig, OrbitConfig
from motor import MotorSystem
from odometry import OdometrySystem
from orbit import OrbitController
from tof import ToFSensor
from vision import VisionReceiver
from wireless_feedforward import FeedforwardSender


# ================= 可以在这里临时修改全部 Orbit 参数进行测试 =================

# 基础比例与视觉目标位置
OrbitConfig.LINEAR_SPEED_SCALE = 0.5
OrbitConfig.TARGET_CENTER_X_PX = 160.0
OrbitConfig.ORBIT_ROD_TARGET_X_PX = 65.0
OrbitConfig.ORBIT_ROD_TARGET_Y_PX = 140.0
OrbitConfig.CAMERA_TURN_DEAD_BAND_X_PX = 15.0
OrbitConfig.ORBIT_Y_DEAD_BAND_PX = 15.0

# 绕行方向、半径与速度
OrbitConfig.ORBIT_DIRECTION = "left"
OrbitConfig.ORBIT_MIN_RADIUS_MM = 150.0
OrbitConfig.TOF_CENTER_OFFSET_MM = 20.0
OrbitConfig.ORBIT_MAX_VX_CM_S = 100.0
OrbitConfig.ORBIT_MAX_VY_CM_S = 100.0
OrbitConfig.ORBIT_MAX_W_RAD_S = 4.0
OrbitConfig.ORBIT_ROTATION_SPEED_RAD_S = 2.0
OrbitConfig.ORBIT_CAMERA_W_WEIGHT = 0.65
OrbitConfig.ORBIT_W_SCALE = 1.0
OrbitConfig.ORBIT_TOF_WEIGHT = 1.0
OrbitConfig.ORBIT_BAND_VY_ENABLED = True

# 阶段切换、完成误差与超时
OrbitConfig.ORBIT_STOP_ERROR_RAD = math.radians(2.0)
OrbitConfig.ORBIT_ENTER_ALIGN_ERROR_RAD = math.radians(5.0)
OrbitConfig.ORBIT_STOP_X_ERROR_PX = 15.0
OrbitConfig.ORBIT_FINAL_ALIGN_X_ERROR_PX = 15.0
OrbitConfig.ORBIT_FINAL_ALIGN_Y_ERROR_PX = 15.0
OrbitConfig.ORBIT_ALIGN_TIMEOUT_S = 1.0
OrbitConfig.ORBIT_CLOSE_IN_TIMEOUT_S = 1.0
OrbitConfig.ORBIT_SLOW_DOWN_START_RAD = math.radians(30.0)
OrbitConfig.ORBIT_SLOW_DOWN_MIN_SCALE = 0.32

# ALIGN / CLOSE_IN 阶段的航向保持
OrbitConfig.ORBIT_ALIGN_KP = 0.55
OrbitConfig.ORBIT_ALIGN_KD = 0.032
OrbitConfig.ORBIT_ALIGN_MAX_W_RAD_S = 3.0
OrbitConfig.ORBIT_ALIGN_MIN_W_RAD_S = 0.45
OrbitConfig.ORBIT_ALIGN_MIN_W_ERROR_RAD = math.radians(2.0)

# CLOSE_IN 的 ToF 安全距离
OrbitConfig.ORBIT_CLOSE_IN_TENNIS_STOP_MM = 120.0
OrbitConfig.ORBIT_CLOSE_IN_STOP_MM = 120.0
OrbitConfig.TOF_VALID_MIN_MM = 20.0
OrbitConfig.TOF_VALID_MAX_MM = 1500.0
OrbitConfig.TOF_EMERGENCY_MM = 120.0
OrbitConfig.TOF_EMERGENCY_RELEASE_MM = 160.0
OrbitConfig.TOF_EMERGENCY_RETREAT_SPEED_CM_S = 20.0

# 总体输出、稳定判定与测试保持行为
OrbitConfig.MAX_XY_SPEED_CM_S = 100.0
OrbitConfig.PUSH_READY_STABLE_S = 0.20
OrbitConfig.CONTINUOUS_HOLD = True
OrbitConfig.TARGET_LOSS_DECAY_S = 0.4

# 绕行阶段：视觉 X 偏差到角速度的 PID
OrbitConfig.PID_CAMERA_TURN_KP = 0.0145
OrbitConfig.PID_CAMERA_TURN_KI = 0.0
OrbitConfig.PID_CAMERA_TURN_KD = 0.0004
OrbitConfig.PID_CAMERA_TURN_I_LIMIT = 150.0
OrbitConfig.PID_CAMERA_TURN_OUTPUT_LIMIT = 1.0

# 绕行阶段：ToF 半径误差到径向速度的 PID
OrbitConfig.PID_ORBIT_TOF_KP = 0.25
OrbitConfig.PID_ORBIT_TOF_KI = 0.0
OrbitConfig.PID_ORBIT_TOF_KD = 0.05
OrbitConfig.PID_ORBIT_TOF_I_LIMIT = 300.0

# 绕行及 CLOSE_IN：视觉 Y 偏差到前后速度的 PID
OrbitConfig.PID_ORBIT_Y_KP = 0.6
OrbitConfig.PID_ORBIT_Y_KI = 0.0
OrbitConfig.PID_ORBIT_Y_KD = 0.05
OrbitConfig.PID_ORBIT_Y_I_LIMIT = 200.0

# ALIGN / CLOSE_IN：视觉 X 偏差到横移速度的 PID
OrbitConfig.PID_X_KP = 0.335
OrbitConfig.PID_X_KI = 0.015
OrbitConfig.PID_X_KD = 0.1
OrbitConfig.PID_X_I_LIMIT = 100.0

# =====================================================================

# None 表示自动锁定第一个合法目标；也可指定 1~5。
TEST_CLASS_ID = None
TEST_DURATION_S = 30.0
START_DELAY_S = 2.0
DEBUG_PRINT_PERIOD_MS = 100


def _ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.monotonic() * 1000.0)


def _ticks_diff(new_value, old_value):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(new_value, old_value)
    return new_value - old_value


def _valid_target(target, locked_class_id):
    if target is None or not bool(target.get("found", False)):
        return False
    try:
        class_id = int(target.get("class_id", 0))
        float(target.get("x"))
        float(target.get("y"))
    except (TypeError, ValueError):
        return False
    return (
        class_id in MissionConfig.TARGET_CLASS_IDS
        and (locked_class_id in (None, 0) or class_id == locked_class_id)
    )


def _valid_entry_tof(distance_mm):
    if distance_mm is None:
        return False
    try:
        distance = float(distance_mm)
    except (TypeError, ValueError):
        return False
    return (
        OrbitConfig.TOF_VALID_MIN_MM
        <= distance
        <= OrbitConfig.TOF_VALID_MAX_MM
    )


def main():
    print("=== 启动正式 OrbitController 实车调参测试 ===")
    print("请把目标放在预期入轨距离，测试会冻结首次有效 ToF 和目标 Y。")

    motor = None
    sender = None
    vision = None
    try:
        odometry = OdometrySystem()
        motor = MotorSystem(odometry=odometry)
        controller = OrbitController(OrbitConfig)
        vision = VisionReceiver(
            uart_id=MissionConfig.MAIN_CAMERA_UART_ID,
            baud=MissionConfig.MAIN_CAMERA_BAUD,
            timeout_ms=MissionConfig.MAIN_CAMERA_TIMEOUT_MS,
        )
        tof_sensor = ToFSensor(
            timeout_ms=MissionConfig.TOF_TIMEOUT_MS,
            valid_min_mm=MissionConfig.TOF_VALID_MIN_MM,
            valid_max_mm=MissionConfig.TOF_VALID_MAX_MM,
        )
        if MissionConfig.FEEDFORWARD_ENABLED:
            sender = FeedforwardSender(
                period_ms=MissionConfig.FEEDFORWARD_TX_PERIOD_MS
            )

        motor.start()
        motor.hard_stop()

        locked_class_id = (
            None if TEST_CLASS_ID is None else int(TEST_CLASS_ID)
        )
        if locked_class_id is None:
            vision.unlock_target()
        else:
            if locked_class_id not in MissionConfig.TARGET_CLASS_IDS:
                raise ValueError("TEST_CLASS_ID must be None or one of 1..5")
            vision.set_target_filter(locked_class_id)

        print("保持车辆静止，{:.1f} 秒后开始读取目标...".format(START_DELAY_S))
        time.sleep(START_DELAY_S)

        started = False
        start_ms = _ticks_ms()
        last_control_ms = start_ms
        last_debug_ms = start_ms - DEBUG_PRINT_PERIOD_MS
        last_reason = None

        while True:
            now_ms = _ticks_ms()
            if _ticks_diff(now_ms, start_ms) > int(TEST_DURATION_S * 1000.0):
                print("Orbit 测试超时。")
                motor.hard_stop()
                break

            dt_ms = _ticks_diff(now_ms, last_control_ms)
            if dt_ms < MissionConfig.CONTROL_PERIOD_MS:
                if sender is not None:
                    sender.send_motor_command_if_due(motor, now_ms)
                time.sleep(0.001)
                continue

            dt = max(0.001, min(dt_ms / 1000.0, 0.1))
            last_control_ms = now_ms
            vision.poll(now_ms)
            target, _ = vision.get_data()
            tof_distance_mm = tof_sensor.update(now_ms)

            if locked_class_id is None and _valid_target(target, None):
                event = vision.lock_target(
                    target,
                    MissionConfig.TARGET_CLASS_IDS,
                )
                if event is not None:
                    locked_class_id = int(event["class_id"])
                    print("已锁定类别 {}，等待过滤后的新视觉帧。".format(locked_class_id))
                    target = None

            if not _valid_target(target, locked_class_id):
                target = None

            pose = odometry.get_pose()
            odometry_state = odometry.get_state()

            if not started:
                motor.hard_stop()
                if target is None or not _valid_entry_tof(tof_distance_mm):
                    continue
                class_id = int(target.get("class_id"))
                target_heading_rad = math.radians(
                    MissionConfig.CLASS_HEADING_DEG[class_id]
                )
                controller.start_from_approach(
                    pose[2],
                    target_heading_rad,
                    float(tof_distance_mm),
                    float(target.get("y")),
                    class_id=class_id,
                )
                started = True
                print(
                    "Orbit 开始: class={} current_heading_deg={:.1f} "
                    "target_heading_deg={:.1f} entry_tof_mm={:.1f} "
                    "target_y={:.1f}".format(
                        class_id,
                        math.degrees(pose[2]),
                        math.degrees(target_heading_rad),
                        float(tof_distance_mm),
                        float(target.get("y")),
                    )
                )

            result = controller.step(
                target,
                tof_distance_mm,
                pose[2],
                yaw_rate_rad_s=odometry_state["yaw_rate_rad_s"],
                dt=dt,
            )
            motor.apply_motion_step(result)

            if result.reason != last_reason:
                print(
                    "reason={} phase={} command=({:.2f},{:.2f},{:.3f})".format(
                        result.reason,
                        result.debug.get("phase", controller.phase),
                        result.command[0],
                        result.command[1],
                        result.command[2],
                    )
                )
                last_reason = result.reason

            if _ticks_diff(now_ms, last_debug_ms) >= DEBUG_PRINT_PERIOD_MS:
                print(
                    "phase={} heading_deg={:.1f} tof={} target={} "
                    "cmd=({:.2f},{:.2f},{:.3f})".format(
                        controller.phase,
                        math.degrees(pose[2]),
                        tof_distance_mm,
                        target,
                        result.command[0],
                        result.command[1],
                        result.command[2],
                    )
                )
                last_debug_ms = now_ms

            if sender is not None:
                sender.send_motor_command_if_due(motor, now_ms)

            if result.done:
                motor.hard_stop()
                print("Orbit 三阶段完成，已硬停。")
                break
            if result.failed:
                motor.hard_stop()
                print("Orbit 失败并硬停: {}".format(result.reason))
                break

    except KeyboardInterrupt:
        print("用户中断 Orbit 测试。")
    except Exception as error:
        print("Orbit 测试异常: {}".format(error))
        try:
            sys.print_exception(error)
        except Exception:
            pass
    finally:
        if motor is not None:
            motor.hard_stop()
            motor.stop()
        if sender is not None:
            try:
                sender.send_zero_frames()
            except Exception:
                pass
        if vision is not None:
            try:
                vision.unlock_target()
            except Exception:
                pass
        print("停止电机输出。")


if __name__ == "__main__":
    main()

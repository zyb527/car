"""正式 ApproachController + OrbitController 的主车实车调参测试。

把目标放在主车前方。本文件直接使用正式 approach.py 和 orbit.py 中的
控制器，先完成 APPROACH，再按正式主流程切入 ORBITING、ALIGN、CLOSE_IN。
C4 指示灯在整个测试期间熄灭，仅在 Orbit 成功完成时点亮。
"""

import math
import os
import sys
import time

try:
    from machine import Pin
except ImportError:
    Pin = None


try:
    PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if PROJECT_DIR not in sys.path:
        sys.path.insert(0, PROJECT_DIR)
except Exception:
    if ".." not in sys.path:
        sys.path.append("..")
    if "/" not in sys.path:
        sys.path.append("/")

from approach import ApproachController
from control import MotionStep
from main_config import ApproachConfig, MissionConfig, OrbitConfig
from motor import MotorSystem
from odometry import OdometrySystem
from orbit import OrbitController
from tof import ToFSensor
from vision import VisionReceiver
from wireless_feedforward import FeedforwardSender


# ================= 可以在这里临时修改全部 Approach 参数进行测试 =================

# 目标位置、减速与对准门限
ApproachConfig.TARGET_CENTER_X_PX = 160.0       # 摄像头视野中心 X 轴图像坐标（像素）
ApproachConfig.STOP_Y_THRESHOLD_PX = 110.0     # 逼近阶段停止时目标物在图像中的目标 Y 轴坐标（像素）
ApproachConfig.SLOW_FORWARD_X_ERROR_PX = 80.0 # 当 X 轴偏差大于该值时触发减速前行（像素）
ApproachConfig.APPROACH_Y_SLOW_START_PX = 50.0 # 图像 Y 轴方向接近目标时开始前向减速的距离（像素）
ApproachConfig.TENNIS_APPROACH_Y_SLOW_START_PX = 45.0 # 网球独立的图像 Y 减速起点（像素）

# 逼近速度与 ToF 停止门限
ApproachConfig.APPROACH_SPEED_CM_S = 100.0    # 逼近最大前进速度（厘米/秒）
ApproachConfig.MIN_APPROACH_SPEED_CM_S = 25.0 # 逼近最小前进速度（厘米/秒）
ApproachConfig.TENNIS_MIN_APPROACH_SPEED_CM_S = 15.0 # 网球逼近最小前进速度（厘米/秒）
ApproachConfig.TOF_SLOW_START_MM = 500.0      # ToF 传感器开始触发减速前行的距离门限（毫米）
ApproachConfig.STOP_DISTANCE_MM = 170.0       # 普通目标逼近停止 / 入轨切换 ToF 距离门限（毫米）
ApproachConfig.TENNIS_STOP_DISTANCE_MM = 200.0 # 网球目标逼近停止 ToF 距离门限（毫米）

# 视觉对准与控制超时
ApproachConfig.TARGET_ALIGN_ERROR_PX = 10.0  # 逼近阶段视觉横向对准误差允许门限（像素）
ApproachConfig.ALIGN_TIMEOUT_S = 1.0          # 逼近阶段视觉对准等待的最大超时时间（秒）
ApproachConfig.TARGET_LOSS_DECAY_S = 0.4      # 丢失视觉目标后速度平滑衰减归零时间（秒）
ApproachConfig.MAX_XY_SPEED_CM_S = 100.0      # 逼近阶段综合平移最大速度上限（厘米/秒）

# 逼近阶段角速度 PID
ApproachConfig.PID_APPROACH_W_KP = 0.012          # 逼近阶段角速度 PID 比例 P 增益
ApproachConfig.PID_APPROACH_W_KI = 0.0            # 逼近阶段角速度 PID 积分 I 增益
ApproachConfig.PID_APPROACH_W_KD = 0.0            # 逼近阶段角速度 PID 微分 D 增益
ApproachConfig.PID_APPROACH_W_OUTPUT_LIMIT = 3.0 # 逼近阶段角速度 PID 输出上限（弧度/秒）
ApproachConfig.PID_APPROACH_W_I_LIMIT = 100.0     # 逼近阶段角速度 PID 积分限幅

# ================= 可以在这里临时修改全部 Orbit 参数进行测试 =================

# 视觉目标位置
OrbitConfig.TARGET_CENTER_X_PX = 160.0          # 摄像头画面物理水平中心 X 坐标（像素，针对 320x240 分辨率画面）
OrbitConfig.ORBIT_ROD_TARGET_X_PX = 65.0        # 斜推杆正前方的目标 X 像素对准点（对位完成时目标落在此点正对斜推杆）
OrbitConfig.ORBIT_ROD_TARGET_Y_PX = 140.0       # 斜推杆正前方的目标 Y 像素对准点（对位与贴近阶段纵向深度控制基准）
OrbitConfig.CAMERA_TURN_DEAD_BAND_X_PX = 15.0  # 视觉转角辅助 PID 控制死区（|x_error| <= 15px 时忽略辅助转角，防止画面抖动）
OrbitConfig.ORBIT_Y_DEAD_BAND_PX = 15.0         # 纵向 Y 轴控制死区（|y_error| <= 15px 时停止前后速度调整）

# 绕行方向、半径与速度
OrbitConfig.ORBIT_DIRECTION = "left"             # 绕行方向（"left": 逆时针/向左绕行，"right": 顺时针/向右绕行）
OrbitConfig.ORBIT_MIN_RADIUS_MM = 150.0         # 绕行保持的标准基准物理半径（毫米，小车环绕目标旋转时的 ToF 锁定距离 15cm）
OrbitConfig.TOF_CENTER_OFFSET_MM = 20.0         # ToF 传感器物理偏移补偿（毫米，ToF 激光安装点相对小车旋转中心的物理偏置 2cm）
OrbitConfig.ORBIT_MAX_VX_CM_S = 100.0           # 绕行最大切向/前向速度限制（厘米/秒，绕行切向移动的最大前进速度）
OrbitConfig.ORBIT_MAX_VY_CM_S = 100.0           # 绕行最大径向/侧向速度限制（厘米/秒，绕行修正半径的最大径向平移速度）
OrbitConfig.ORBIT_MAX_W_RAD_S = 4.0             # 绕行阶段最大允许旋转角速度（弧度/秒，防止旋转过快导致摄像机模糊）
OrbitConfig.ORBIT_ROTATION_SPEED_RAD_S = 2.0   # 绕行阶段基准旋转角速度（弧度/秒，小车围绕目标旋转的标准角速度基准值）
OrbitConfig.ORBIT_CAMERA_W_WEIGHT = 0.65        # 视觉修正角速度在合成角速度中的权重（0.0~1.0，合成角速度 = W_base + 0.65 * W_camera）
OrbitConfig.ORBIT_W_SCALE = 1.0                 # 绕行角速度整体最终输出缩放比（1.0 表示 100% 原始输出）
OrbitConfig.ORBIT_TOF_WEIGHT = 1.0              # ToF 测距在半径修正中的控制权重
OrbitConfig.ORBIT_BAND_VY_ENABLED = True        # 绕行半径安全带 (Radius Band) 限速开关（开启后偏离标准半径过大时强制限速防脱轨）

# 阶段切换、完成误差与超时
OrbitConfig.ORBIT_STOP_ERROR_RAD = math.radians(2.0)         # 最终航向合格允许的最大角度误差门限（弧度，约 2.0 度）
OrbitConfig.ORBIT_ENTER_ALIGN_ERROR_RAD = math.radians(5.0)  # 切入推杆对位 (PHASE_ALIGN) 的角度误差门限（弧度，约 5.0 度）
OrbitConfig.ORBIT_STOP_X_ERROR_PX = 15.0                    # 横向对位合格允许的最大 X 轴像素偏差（像素，|target_x - 65| <= 15px）
OrbitConfig.ORBIT_FINAL_ALIGN_X_ERROR_PX = 15.0             # 贴近阶段 (CLOSE_IN) 的 X 轴允许像素误差门限（像素）
OrbitConfig.ORBIT_FINAL_ALIGN_Y_ERROR_PX = 15.0             # 贴近阶段 (CLOSE_IN) 的 Y 轴允许像素误差门限（像素）
OrbitConfig.ORBIT_ALIGN_TIMEOUT_S = 1.0                      # 推杆横向对位阶段超时限定时间（秒，受 CONTINUOUS_HOLD 控制）
OrbitConfig.ORBIT_CLOSE_IN_TIMEOUT_S = 1.0                   # 推杆逼近贴靠阶段超时限定时间（秒，受 CONTINUOUS_HOLD 控制）
OrbitConfig.ORBIT_SLOW_DOWN_START_RAD = math.radians(30.0)  # 接近绕行终点开始降旋转角速度的角度残差门限（弧度，约 30 度）
OrbitConfig.ORBIT_SLOW_DOWN_MIN_SCALE = 0.32                 # 绕行终点降速的最小角速度下限比例（基准角速度的 32%）

# ALIGN / CLOSE_IN 阶段的航向保持
OrbitConfig.ORBIT_ALIGN_KP = 0.52                            # 对位与贴靠阶段靠 IMU 维持目标航向角 proportional 增益 P
OrbitConfig.ORBIT_ALIGN_KD = 0.03                          # 对位与贴靠阶段靠 IMU 维持目标航向角 derivative 增益 D（抑制抖动）
OrbitConfig.ORBIT_ALIGN_MAX_W_RAD_S = 3.0                    # 姿态角保持阶段允许的最大修正角速度（弧度/秒）
OrbitConfig.ORBIT_ALIGN_MIN_W_RAD_S = 0.40                  # 克服电机静摩擦力的最小补偿旋转角速度（弧度/秒）
OrbitConfig.ORBIT_ALIGN_MIN_W_ERROR_RAD = math.radians(3.0)  # 触发静摩擦补偿的角度残差门限（弧度，偏差大于 2 度时生效）

# CLOSE_IN 的 ToF 安全距离
OrbitConfig.ORBIT_CLOSE_IN_TENNIS_STOP_MM = 120.0       # 网球（Class 3）贴近停止门限（毫米，向前贴近至 120mm 时停止）
OrbitConfig.ORBIT_CLOSE_IN_STOP_MM = 120.0              # 普通目标贴近停止门限（毫米，向前贴近至 120mm 时停止准备推入）
OrbitConfig.TOF_VALID_MIN_MM = 20.0                     # ToF 激光传感器有效读数最小下限门限（毫米，低于 20mm 视为失真）
OrbitConfig.TOF_VALID_MAX_MM = 1500.0                   # ToF 激光传感器有效读数最大上限门限（毫米，高于 1500mm 视为空旷）
OrbitConfig.TOF_EMERGENCY_MM = 120.0                    # 紧急后退防撞触发距离（毫米，距离目标低于 120mm 且继续前冲时紧急退避）
OrbitConfig.TOF_EMERGENCY_RELEASE_MM = 160.0            # 紧急后退防撞解除距离（毫米，退避至 160mm 以上时解除紧急状态）
OrbitConfig.TOF_EMERGENCY_RETREAT_SPEED_CM_S = 20.0    # 触发防撞紧急状态时的后退撤退速度（厘米/秒）

# 总体输出、稳定判定与测试保持行为
OrbitConfig.MAX_XY_SPEED_CM_S = 120.0      # 绕行模块综合平移合速度上限（厘米/秒，限制 sqrt(vx^2 + vy^2) <= 100）
OrbitConfig.PUSH_READY_STABLE_S = 0.20     # 进入 Push 状态前的稳定保持时间（秒，角度与 X/Y 轴偏差同时合格且保持 0.2s 才切入 Push）
OrbitConfig.CONTINUOUS_HOLD = True         # 持续微调保持模式（True: 忽视 1 秒超时报错，持续调节直到完成；False: 超时 1 秒报失败）
OrbitConfig.TARGET_LOSS_DECAY_S = 0.4      # 丢失视觉目标后速度平滑衰减归零的减速缓冲时间（秒）

# 绕行阶段：视觉 X 偏差到角速度的 PID
OrbitConfig.PID_CAMERA_TURN_KP = 0.0145            # 视觉转角辅助 PID 比例 P 增益
OrbitConfig.PID_CAMERA_TURN_KI = 0.0               # 视觉转角辅助 PID 积分 I 增益
OrbitConfig.PID_CAMERA_TURN_KD = 0.0004             # 视觉转角辅助 PID 微分 D 增益
OrbitConfig.PID_CAMERA_TURN_I_LIMIT = 150.0        # 视觉转角辅助 PID 积分限幅
OrbitConfig.PID_CAMERA_TURN_OUTPUT_LIMIT = 1.0     # 视觉转角辅助 PID 输出角速度上限（弧度/秒）

# 绕行阶段：ToF 半径误差到径向速度的 PID
OrbitConfig.PID_ORBIT_TOF_KP = 0.25                # ToF 测距半径修正 PID 比例 P 增益
OrbitConfig.PID_ORBIT_TOF_KI = 0.0                 # ToF 测距半径修正 PID 积分 I 增益
OrbitConfig.PID_ORBIT_TOF_KD = 0.025                # ToF 测距半径修正 PID 微分 D 增益
OrbitConfig.PID_ORBIT_TOF_I_LIMIT = 300.0          # ToF 测距半径修正 PID 积分限幅

# 绕行及 CLOSE_IN：视觉 Y 偏差到前后速度的 PID
OrbitConfig.PID_ORBIT_Y_KP = 0.6                   # 图像 Y 轴像素偏置修正 PID 比例 P 增益
OrbitConfig.PID_ORBIT_Y_KI = 0.0                   # 图像 Y 轴像素偏置修正 PID 积分 I 增益
OrbitConfig.PID_ORBIT_Y_KD = 0.05                  # 图像 Y 轴像素偏置修正 PID 微分 D 增益
OrbitConfig.PID_ORBIT_Y_I_LIMIT = 200.0            # 图像 Y 轴像素偏置修正 PID 积分限幅

# ALIGN / CLOSE_IN：视觉 X 偏差到横移速度的 PID
OrbitConfig.PID_X_KP = 0.335                       # 图像 X 轴横向平移修正 PID 比例 P 增益（控制横移对准 X=65px）
OrbitConfig.PID_X_KI = 0.015                       # 图像 X 轴横向平移修正 PID 积分 I 增益
OrbitConfig.PID_X_KD = 0.1                         # 图像 X 轴横向平移修正 PID 微分 D 增益
OrbitConfig.PID_X_I_LIMIT = 100.0                  # 图像 X 轴横向平移修正 PID 积分限幅

# =====================================================================

# 单项测试运行参数
TEST_CLASS_ID = None         # 单项测试锁定目标的类别 ID（None: 自动选择第一个合法目标；1~5: 指定物体类 ID）
TEST_DURATION_S = 30.0       # 单项绕行测试运行的最大时长（秒，超过 30 秒自动退出）
START_DELAY_S = 2.0          # 测试启动延时时间（秒，用于让数据稳定及人员撤离）
DEBUG_PRINT_PERIOD_MS = 100  # 串口/控制台调试日志打印刷新周期（毫秒，100ms 即 10Hz 刷新率）


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
    print("=== 启动正式 Approach + Orbit 实车调参测试 ===")
    print("请把目标放在主车前方；测试会先靠近，再冻结入轨 ToF 和目标 Y。")

    motor = None
    sender = None
    vision = None
    led_c4 = None
    orbit_succeeded = False
    try:
        if Pin is not None:
            led_c4 = Pin("C4", Pin.OUT)
            # C4 为低电平点亮；测试期间保持熄灭。
            led_c4.value(1)

        odometry = OdometrySystem()
        motor = MotorSystem(odometry=odometry)
        approach = ApproachController(ApproachConfig)
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

        stage = "WAIT_TARGET"
        orbit_class_id = None
        target_heading_rad = None
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

            if stage == "WAIT_TARGET":
                motor.hard_stop()
                if target is None:
                    continue
                orbit_class_id = int(target.get("class_id"))
                target_heading_rad = math.radians(
                    MissionConfig.CLASS_HEADING_DEG[orbit_class_id]
                )
                approach.reset()
                controller.reset()
                stage = "APPROACH"
                print(
                    "Approach 开始: class={} current_heading_deg={:.1f} "
                    "target_heading_deg={:.1f}".format(
                        orbit_class_id,
                        math.degrees(pose[2]),
                        math.degrees(target_heading_rad),
                    )
                )

            if stage == "APPROACH":
                result = approach.step(target, tof_distance_mm, dt=dt)
                if result.done:
                    entry_tof_mm = result.debug.get(
                        "orbit_radius_mm", tof_distance_mm
                    )
                    entry_target_y = result.debug.get("orbit_target_y_px")
                    if entry_target_y is None and target is not None:
                        entry_target_y = target.get("y")
                    if entry_tof_mm is None or entry_target_y is None:
                        result = MotionStep.stop(
                            "approach_done_without_orbit_entry",
                            failed=True,
                        )
                    else:
                        controller.start_from_approach(
                            pose[2],
                            target_heading_rad,
                            float(entry_tof_mm),
                            float(entry_target_y),
                            class_id=orbit_class_id,
                        )
                        stage = "ORBIT"
                        print(
                            "Orbit 开始: entry_tof_mm={:.1f} target_y={:.1f}".format(
                                float(entry_tof_mm),
                                float(entry_target_y),
                            )
                        )
                        result = MotionStep.stop(
                            "approach_to_orbit",
                            debug={
                                # 与主流程一致：入轨瞬间立即清零 Approach
                                # 的前移残余，避免 S 曲线拖慢切换。
                                "immediate_command": True,
                                "phase": controller.phase,
                                "entry_tof_mm": controller.entry_tof_mm,
                                "control_tof_mm": controller.control_tof_mm,
                                "entry_center_radius_mm": (
                                    controller.entry_center_radius_mm
                                ),
                                "orbit_center_radius_mm": (
                                    controller.control_center_radius_mm
                                ),
                            },
                        )
            else:
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
                    "reason={} stage={} phase={} command=({:.2f},{:.2f},{:.3f})".format(
                        result.reason,
                        stage,
                        result.debug.get("phase", controller.phase),
                        result.command[0],
                        result.command[1],
                        result.command[2],
                    )
                )
                last_reason = result.reason

            if _ticks_diff(now_ms, last_debug_ms) >= DEBUG_PRINT_PERIOD_MS:
                print(
                    "stage={} phase={} heading_deg={:.1f} tof={} target={} "
                    "cmd=({:.2f},{:.2f},{:.3f})".format(
                        stage,
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
                if stage == "ORBIT":
                    orbit_succeeded = True
                    if led_c4 is not None:
                        led_c4.value(0)
                    print("Orbit 三阶段完成，C4 已点亮，已硬停。")
                else:
                    print("Approach 完成，未进入 Orbit。")
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
        if led_c4 is not None and not orbit_succeeded:
            led_c4.value(1)
        print("停止电机输出。")


if __name__ == "__main__":
    main()





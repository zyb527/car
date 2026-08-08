"""
测试文件: 用于调试靠近（ApproachController）的参数以及无线前馈发送。
功能:
1. 初始化电机、里程计、视觉和 ToF 传感器。
2. 循环获取视觉目标和 ToF 测距。
3. 调用 ApproachController 计算靠近所需的速度。
4. 将速度下发给电机，并同时通过无线前馈发送出去。
5. 串口持续打印 target_y, tof_distance, vy_cmd，方便观察减速效果。
"""
import math
import time
import sys
import os

try:
    from machine import Pin
except ImportError:
    Pin = None

try:
    # 兼容 PC 端运行
    PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if PROJECT_DIR not in sys.path:
        sys.path.insert(0, PROJECT_DIR)
except Exception:
    # 兼容单片机 (MicroPython) 环境下 __file__ 或 os.path 不存在的情况
    if ".." not in sys.path:
        sys.path.append("..")
    if "/" not in sys.path:
        sys.path.append("/")

from motor import MotorSystem
from odometry import OdometrySystem
from approach import ApproachController
from main_config import ApproachConfig, MissionConfig
from wireless_feedforward import FeedforwardSender
from vision import VisionReceiver
from tof import ToFSensor

# ================= 可以在这里临时修改全部 Approach 参数进行测试 =================

# 视觉目标位置
ApproachConfig.TARGET_CENTER_X_PX = 160.0           # 希望目标位于画面中的 X 坐标（像素）
ApproachConfig.STOP_Y_THRESHOLD_PX = 110.0          # 允许视觉停车时的图像 Y 停止阈值（像素）
ApproachConfig.SLOW_FORWARD_X_ERROR_PX = 80.0       # X 偏差达到该值时，前进速度降到最小值（像素）
ApproachConfig.APPROACH_Y_SLOW_START_PX = 50.0      # 图像 Y 达到该值后开始减速（像素）
 
# 前进速度
ApproachConfig.APPROACH_SPEED_CM_S = 100.0          # 接近阶段最大前进速度（cm/s）
ApproachConfig.MIN_APPROACH_SPEED_CM_S = 30.0       # 接近阶段最小前进速度（cm/s）
ApproachConfig.MAX_XY_SPEED_CM_S = 100.0            # Approach 平面合速度上限（cm/s）

# ToF 距离与停车条件
ApproachConfig.TOF_SLOW_START_MM = 500.0            # ToF 距离低于该值后开始减速（毫米）
ApproachConfig.STOP_DISTANCE_MM = 170.0             # 普通物体停止距离（毫米）
ApproachConfig.TENNIS_STOP_DISTANCE_MM = 200.0      # 网球停止距离（毫米）
ApproachConfig.TOF_VALID_MIN_MM = 20.0              # ToF 有效距离下限（毫米）
ApproachConfig.TOF_VALID_MAX_MM = 1500.0            # ToF 有效距离上限（毫米）
ApproachConfig.TOF_FALLBACK_SPEED_CM_S = 30.0       # ToF 无效时允许的前进速度上限（cm/s）
ApproachConfig.TOF_FALLBACK_STOP_Y_PX = 120.0       # ToF 无效且图像 Y 达到该值时停止前进（像素）

# 对准、完成与异常处理
ApproachConfig.TARGET_ALIGN_ERROR_PX = 10.0         # 到达距离后允许的 X 对准误差（像素）
ApproachConfig.ALIGN_TIMEOUT_S = 1.0                # 最终纯旋转对准的最长时间（秒）
ApproachConfig.TARGET_LOSS_DECAY_S = 0.4            # 丢失目标后旧指令衰减到零的时间（秒）
ApproachConfig.ORBIT_MIN_RADIUS_MM = 0.0            # 交给 Orbit 的最小冻结半径（毫米）
ApproachConfig.VISUAL_STOP_ENABLED = True           # 是否允许仅凭图像 Y 判定 Approach 完成

# X 偏差到角速度 w 的 PID；Approach 不再输出横移 vx
ApproachConfig.PID_APPROACH_W_KP = 0.012           # 角速度 PID P 增益
ApproachConfig.PID_APPROACH_W_KI = 0.0          # 角速度 PID I 增益
ApproachConfig.PID_APPROACH_W_KD = 0.0          # 角速度 PID D 增益
ApproachConfig.PID_APPROACH_W_OUTPUT_LIMIT = 3.0    # 最大角速度输出（rad/s）
ApproachConfig.PID_APPROACH_W_I_LIMIT = 100.0       # 积分项限幅
# =====================================================================

TEST_DURATION_S = 20.0
# 你可以在这里指定想锁定的类别ID (例如 1=沙包, 3=网球)。设为 None 则会自动靠近它看到的第一个目标
TARGET_CLASS_ID = None

def _ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.monotonic() * 1000.0)

def _ticks_diff(new_value, old_value):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(new_value, old_value)
    return new_value - old_value

def main():
    print("=== 启动 Approach 参数与前馈 (硬件) 调参测试 ===")
    
    motor = None
    sender = None
    try:
        odometry = OdometrySystem()
        motor = MotorSystem(odometry=odometry)
        
        # 初始化真实传感器
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
        controller = ApproachController(ApproachConfig)
        
        # 初始化无线前馈
        if MissionConfig.FEEDFORWARD_ENABLED:
            sender = FeedforwardSender(period_ms=MissionConfig.FEEDFORWARD_TX_PERIOD_MS)
        
        motor.start()
        motor.hard_stop()
        
        vision.unlock_target()  # 确保摄像头没有残留的锁定滤镜
        print("请把小车正对目标，2秒后开始测试...")
        time.sleep(2.0)
        
        start_time = _ticks_ms()
        last_control_ms = start_time
        locked_class_id = TARGET_CLASS_ID
        
        print("开始接近测试，请在 Thonny 绘图器中查看波形")
        # 打印表头（用于绘图器的列名，按顺序分别对应数据）
        print("time_s, target_y, tof_mm, vy_cmd")
        
        while True:
            now_ms = _ticks_ms()
            dt_ms = _ticks_diff(now_ms, last_control_ms)
            
            # 20ms 控制周期
            if dt_ms >= 20:
                dt = dt_ms / 1000.0
                last_control_ms = now_ms
                
                run_time_s = _ticks_diff(now_ms, start_time) / 1000.0
                if run_time_s > TEST_DURATION_S:
                    print("测试时间结束。")
                    break
                
                # 1. 获取传感器数据
                vision.poll(now_ms)
                tof_distance_mm = tof_sensor.update(now_ms)
                
                target, _ = vision.get_data()
                
                # 自动锁定机制
                if target and target.get("found"):
                    cid = int(target.get("class_id", 0))
                    if locked_class_id is None and cid in MissionConfig.TARGET_CLASS_IDS:
                        locked_class_id = cid
                        vision.lock_target(target, MissionConfig.TARGET_CLASS_IDS)
                        print("自动锁定目标类别: {}".format(locked_class_id))
                    
                    if locked_class_id is not None and cid != locked_class_id:
                        target = {"found": False}
                
                # 2. 调用 Approach 控制器
                result = controller.step(target, tof_distance_mm, dt)

                # 本调参测试在 Approach 完成时立即硬停，便于实车比较
                # hard_stop 与 command(0, 0, 0) 的停车效果。
                if result.done:
                    result.debug["hard_stop"] = True
                
                # 3. 发送给电机 (兼容旧版 motor.mpy 固件)
                if hasattr(motor, "apply_motion_step"):
                    motor.apply_motion_step(result)
                else:
                    if result.failed or result.debug.get("hard_stop", False):
                        motor.hard_stop()
                    elif result.debug.get("immediate_command", False):
                        motor.command(*result.command)
                    else:
                        motor.move(*result.command)
                
                # 4. 串口输出，用于波形绘制
                target_y = target.get("y", 0.0) if target and target.get("found") else 0.0
                tof_disp = tof_distance_mm if tof_distance_mm is not None else 0.0
                vy_cmd = result.command[1] if result.command else 0.0
                
                print(" {:.2f}, {:.1f}, {:.1f}, {:.2f} ".format(
                    run_time_s, target_y, tof_disp, vy_cmd
                ))
                
                if result.done:
                    # 当 done 时说明判定为已经完成
                    print("测试提示: 靠近已判定为完成 (done=True)")
                    break
                
            # 无线前馈发送
            if sender is not None:
                sender.send_motor_command_if_due(motor, now_ms)
                
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("用户主动中断。")
    except Exception as e:
        print("发生异常: {}".format(e))
        try:
            import sys
            sys.print_exception(e)
        except Exception:
            pass
    finally:
        if motor:
            motor.hard_stop()
            motor.stop()
        if 'sender' in locals() and sender is not None:
            try:
                sender.send_zero_frames()
            except Exception:
                pass
        print("停止电机输出。")

if __name__ == "__main__":
    main()

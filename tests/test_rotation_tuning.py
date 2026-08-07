"""
测试文件: 用于调试原地自转（HeadingTurnController）的 PID 参数和角速度。
功能:
1. 初始化电机和里程计。
2. 设定一个目标航向角（如 90 度）。
3. 循环调用 HeadingTurnController 旋转至目标角。
4. 串口打印目标角度、实际角度和输出角速度，方便在 Thonny 的绘图器中观察是否有“摆头”现象或“不调整/不结束”的问题。
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
from navigation import HeadingTurnController
from main_config import NavigationConfig, MissionConfig
from wireless_feedforward import FeedforwardSender

# ================= 可以在这里临时修改自转相关参数进行测试 =================

NavigationConfig.TURN_FAST_ERROR_RAD = math.radians(25.0) # 快速自转误差门限
NavigationConfig.TURN_MID_ERROR_RAD = math.radians(5.0)   # 中速自转误差门限
NavigationConfig.TURN_FAST_W_RAD_S = 3.14                # 快速自转角速度
NavigationConfig.TURN_MID_KP = 5.0                        # 中速段 P 增益
NavigationConfig.TURN_MID_W_RAD_S = 1.50                  # 中速段 P 输出上限
NavigationConfig.TURN_SLOW_KP = 4.0                       # 慢速段 P 增益
NavigationConfig.TURN_DAMPING_KD = 0.10                   # 阻尼系数（角速度抑制）
NavigationConfig.TURN_TOLERANCE_RAD = math.radians(4.0)   # 角度容差门限
NavigationConfig.TURN_YAW_RATE_TOLERANCE_RAD_S = 0.12     # 停稳的角速度门限
# =====================================================================

TARGET_HEADING_DEG = 180.0
TEST_DURATION_S = 10.0

def _ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.monotonic() * 1000.0)

def _ticks_diff(new_value, old_value):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(new_value, old_value)
    return new_value - old_value

def main():
    print("=== 启动原地自转 (HeadingTurnController) 调参测试 ===")
    
    led_c4 = None
    turn_succeeded = False
    if Pin:
        try:
            led_c4 = Pin("C4", Pin.OUT)
            # C4 指示灯为低电平点亮；测试完成前保持熄灭。
            led_c4.value(1)
        except Exception as e:
            print("初始化 C4 灯失败:", e)

    motor = None
    try:
        odometry = OdometrySystem()
        motor = MotorSystem(odometry=odometry)
        
        # 初始化控制器
        turn_controller = HeadingTurnController(NavigationConfig)
        target_rad = math.radians(TARGET_HEADING_DEG)
        
        # 初始化无线前馈
        sender = None
        if MissionConfig.FEEDFORWARD_ENABLED:
            sender = FeedforwardSender(period_ms=MissionConfig.FEEDFORWARD_TX_PERIOD_MS)
        
        print("正在标定 IMU，请保持车辆静止；若被碰撞会自动重新采样...")
        motor.start()
        motor.hard_stop()
        
        print("请保持车辆静止，2秒后开始测试...")
        time.sleep(2.0)
        
        # 初始朝向设为 0 度
        odometry.set_pose(0.0, 0.0, 0.0)
        
        # 启动控制器
        turn_controller.start(target_rad)
        
        start_time = _ticks_ms()
        last_control_ms = start_time
        suppress_feedforward_w = False
        
        print("开始旋转测试，请在 Thonny 绘图器中查看波形")
        # 打印表头（用于绘图器的列名，按顺序分别对应数据）
        print("time_s, target_deg, actual_deg, w_cmd_x10")
        
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
                
                # 1. 获取当前状态
                pose = odometry.get_pose()
                current_heading_rad = pose[2]
                current_yaw_rate = odometry.get_state()["yaw_rate_rad_s"]
                
                # 2. 调用旋转控制器
                result = turn_controller.step(current_heading_rad, current_yaw_rate, dt)
                suppress_feedforward_w = bool(
                    result.debug.get("suppress_feedforward_w", False)
                )
                
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
                current_heading_deg = math.degrees(current_heading_rad)
                w_cmd = result.command[2] if result.command else 0.0
                
                # 角速度乘以 10 是为了在图表上和角度处于相近的数量级，方便同时观察
                print(" {:.2f}, {:.1f}, {:.1f}, {:.2f} ".format(
                    run_time_s, TARGET_HEADING_DEG, current_heading_deg, w_cmd * 10
                ))
                
                if result.done:
                    # 当 done 时说明判定为已经完成
                    print("测试提示: 转向已判定为完成 (done=True)")
                    turn_succeeded = True
                    if led_c4:
                        led_c4.value(0)
                    break
                
            if sender is not None:
                sender.send_motor_command_if_due(
                    motor,
                    now_ms,
                    straight_without_w=suppress_feedforward_w,
                )
                
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
        if led_c4 and not turn_succeeded:
            led_c4.value(1)
        print("停止电机输出。")

if __name__ == "__main__":
    main()

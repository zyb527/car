"""
测试文件: 用于调试电机底层 PID 参数，平滑变换速度。
功能: 
1. 采用正弦函数生成平滑变化的目标速度。
2. 将三个轮子的期望速度设为相同的平滑曲线（或单独测试某个轮子）。
3. 串口输出期望速度和实际速度，供上位机（如 Thonny、Vofa+）绘制波形进行 PID 调参。
"""

import math
import time
from machine import Pin
# 底层速度控制库及底盘参数
import speedcontrol

# 测试参数配置
MAX_SPEED_CM_S = 500.0  # 正弦峰值速度 (cm/s)
SINE_PERIOD_S = 4.0     # 正弦周期 (秒) - 周期越长，速度变化越平滑
TEST_DURATION_S = 20.0  # 总测试时长 (秒)

def main():
    print("=== 启动平滑速度 PID 调参测试 ===")

    # 初始化底层环境
    print("初始化运动控制器...")
    motion = speedcontrol.SpeedController()
    
    start_time = time.ticks_ms()
    last_update = start_time

    print("开始输出平滑速度波形...")
    print("请打开上位机或 Thonny 绘图器查看 (期望V1, 实际V1, 期望V2, 实际V2, 期望V3, 实际V3)")

    try:
        while True:
            current_time = time.ticks_ms()
            dt_ms = time.ticks_diff(current_time, last_update)
            
            # 恢复到高频控制：现在底层刷新是 20ms 了
            if dt_ms >= 20: 
                last_update = current_time
                
                # 计算运行时间
                run_time_s = time.ticks_diff(current_time, start_time) / 1000.0

                if run_time_s <= TEST_DURATION_S:
                    # 使用正弦波生成平滑的速度曲线
                    # 公式: v = v_max * sin(2 * pi * t / T)
                    target_speed = MAX_SPEED_CM_S * math.sin(2.0 * math.pi * run_time_s / SINE_PERIOD_S)
                    
                    # 悬空调参：必须只让一个轮子转！否则哪怕悬空，三个轮子加减速的微小差异也会带着整个底盘扭动，导致万向轮因为干涉读出负值
                    exp_w1 = target_speed
                    exp_w2 = target_speed
                    exp_w3 = target_speed    

                    # 发送目标速度 给底层
                    motion.speed_control(exp_w1, exp_w2, exp_w3, stop_flag=0)

                    # 获取实际滤波后速度
                    v1, v2, v3 = motion.speed_filtered

                    # 串口输出，增加打印当前时间 (秒)
                    # 可以在 Thonny -> 视图 -> 绘图器 中直观看到波形跟随情况，格式：时间, 期望V1, 实测V1
                    print("   {:.1f},    {:.1f} ,{:.1f},    {:.1f} ,{:.1f},    {:.1f} ".format(
                         exp_w1, v1,exp_w2, v2,exp_w3, v3
                    ))
                else:
                    break

        print("测试结束。")

    except KeyboardInterrupt:
        print("用户中断。")
    finally:
        # 测试结束或异常退出时，确保电机停转
        motion.speed_control(0, 0, 0, stop_flag=1)
        print("停止电机输出。")

if __name__ == "__main__":
    main()




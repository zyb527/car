"""辅助车无线前馈加视觉修正的并排跟随配置。

默认值只用于首次架空和低速调试。带“现场标定项”的参数必须在最终
安装姿态下重新确认，不能直接作为比赛参数。
"""

# 红外跟随总开关。False 时完全跳过摄像头读取和视觉控制，
# 只使用无线前馈；前馈丢失时直接硬停车，不退回纯视觉。
INFRARED_FOLLOW_ENABLED = True

# 主控连接跟随摄像头的 UART。
UART_ID = 5
BAUD = 230400
UART_BUFFER_MAX_BYTES = 256
UART_TIMEOUT_MS = 0
UART_TIMEOUT_CHAR_MS = 0
# 单次只读取当前已经到达的字节，并限制本轮最大搬运量；不能使用无长度
# uart.read()，否则部分固件会持续等待后续字符直到默认超时。
UART_READ_MAX_BYTES = 256

# 主循环和数据新鲜度
CONTROL_PERIOD_MS = 10
CAMERA_TIMEOUT_MS = 150
REACQUIRE_FRAMES = 1
# 无线仍新鲜时，视觉短暂丢失不把上一帧修正瞬间切成零，而是在该时间内
# 线性衰减。纯视觉模式丢失仍立即硬停车。
VISUAL_LOSS_DECAY_MS = 250
PI_DT_MIN_S = 0.005
PI_DT_MAX_S = 0.100

# 相对位置 alpha-beta 估计。位置本身直接跟随当前测量，beta 只用于估计
# 相对速度，因此不会在摄像头 EMA 之外再增加一层明显的位置滞后。
POSITION_FILTER_ALPHA = 1.0
POSITION_FILTER_BETA = 0.35

# 沿用 car141929/旧双车方案的无线链路参数；不改变现有动作参数。
WIRELESS_BAUD = 230400
FEEDFORWARD_TIMEOUT_MS = 300

# 摄像头协议已经是物理平面三元组 x_cm,y_cm,theta_deg。以下范围覆盖
# 当前 QVGA 有效视野经 IPM 后的坐标包络，并为标定误差留出余量。
CAMERA_OUTPUTS_PLANE_POSE = True
TARGET_X_MIN_CM = -25.0
TARGET_X_MAX_CM = 25.0
TARGET_Y_MIN_CM = -30.0
TARGET_Y_MAX_CM = 25.0
THETA_MIN_DEG = -90.0
THETA_MAX_DEG = 90.0

# 摄像头端理想队形下“像素 x 较大右灯”的 IPM 物理坐标。车端直接对
# 物理坐标作差，不再运行 IPM；theta 已由摄像头减去物理参考方向，目标为 0°。
REF_TARGET_X_CM = 0.316
REF_TARGET_Y_CM = -6.183
REF_TARGET_THETA_DEG = -0.70

# 摄像头已经输出厘米坐标，下面误差节点单位均为 cm。
FRONT_DEADBAND_PLANE = 0.4
FRONT_RESPONSE_POINTS_PLANE = (
    (0.0, 0.0),
    (0.5, 5.0),
    (2,15),
    (3,25),
    (10.0, 70.0),
)
# 前后误差在收敛到目标的中心区域时，逐步减弱 P，避免超调；D 置零则为纯 P。
FRONT_OPENING_D_GAIN = 0.0
FRONT_CLOSING_D_GAIN = 0.353
FRONT_RATE_DEADBAND_PLANE = 8.0
LATERAL_RATE_DEADBAND_PLANE = 5.0
MAX_FRONT_D_TRIM = 30.0
FRONT_CLOSING_P_SCALE = 0.3
FRONT_CLOSING_P_BAND_PLANE = 5.0

# 横向误差来自摄像头输出的 IPM 平面右灯坐标，不再使用灯距反比例估算。
# 重新标定 IPM 后，纯前后 10cm 平移造成的横向串扰已降至 0.06 cm 左右。
# 几何误差被消除，因此将横向死区缩小至 0.8 cm，提升并排跟车的紧密感。
LATERAL_DEADBAND_PLANE = 0.3
LATERAL_RESPONSE_POINTS_PLANE = (
    (0.0, 0.0),
    (0.5, 5.0),
    (2,15),
    (3,25),
    (10.0, 70.0),
)
LATERAL_D_GAIN = 0.5
MAX_LATERAL_D_TRIM = 15.0
# 航向平行：摄像头 IPM 物理 theta -> w。节点使用去死区后的绝对角度误差（deg）和
# 视觉修正角速度（rad/s）；中心段极软 P=0.08 rad/(s·deg)，大误差保留高响应，
# 在 3 rad/s 限幅。此通道不使用积分，避免转向误差反向后留下积分拖慢跟随。
ANGLE_DEADBAND_DEG = 3.0
ANGLE_RESPONSE_POINTS = (
    (0.0, 0.0),
    (2.0, 0.10),
    (4.0, 0.25),
    (6.0, 0.45),
    (8.0, 0.70),
    (10.0, 1.00),
    (15.0, 1.50),
)
 
# IPM 后物理角相对原像素角发生确定性的符号翻转，因此 W_SIGN 由旧协议的
# -1 改为 +1，保持相同实车姿态下的最终电机转向修正方向不变。
VX_SIGN = 1.0
VY_SIGN = 1.0
W_SIGN = 1.0

# 视觉闭环只负责队形误差修正；中心附近力度由分段节点保证不变。
MAX_VX = 120.0
MAX_VY = 120.0
MAX_W = 3.0

# 有无线前馈时，视觉只修正编队残差，不再承担全部跟随速度。保持纯视觉
# 参数不变，仅对前馈模式的视觉输出使用独立缩放、限幅和低通。
FEEDFORWARD_VISUAL_VX_SCALE = 0.3
FEEDFORWARD_VISUAL_VY_SCALE = 1.0
FEEDFORWARD_VISUAL_W_SCALE = 0.60
FEEDFORWARD_VISUAL_MAX_VX = 80.0
FEEDFORWARD_VISUAL_MAX_VY = 80.0
FEEDFORWARD_VISUAL_MAX_W = 1.5
# 每个新视觉测量吸收目标修正的比例；越小越平缓，但纠偏也越慢。
# 1.0 表示不做逐帧低通；保留无线模式专用比例、限幅和模式切换渐变。
FEEDFORWARD_VISUAL_FILTER_ALPHA = 1.0
# 前馈出现/消失时，两套视觉输出每个主循环的混合比例。
FEEDFORWARD_VISUAL_MODE_BLEND_ALPHA =0.35
# 电机线速度比例从 0.877 提升到 1.512 后，为保持现有视觉跟随 PID、
# 限幅节点的实际平移效果不变，仅视觉 vx/vy 输出乘以旧/新比例。
# 无线前馈本身已是主车真实速度，不使用此比例。
VISUAL_FOLLOW_LINEAR_COMMAND_SCALE = 58.0 / 100.0
# 无线前馈当前帧确认主车静止时，临时不叠加视觉平移修正，让辅助车优先
# 停住；下一帧出现非零前馈即自动恢复视觉修正，不进入锁止状态。
FEEDFORWARD_STATIONARY_VISUAL_XY_SUPPRESS_ENABLED = False
FEEDFORWARD_STATIONARY_LINEAR_THRESHOLD_CM_S = 1.0
FEEDFORWARD_STATIONARY_W_THRESHOLD_RAD_S = 0.01

# 车体系中辅助车相对主车的位置：右为正、前为正。实测并排队形中，
# 辅助车旋转中心在主车旋转中心左侧 20 cm，前后对齐。
# 刚性队形转动补偿：右移速度补偿=-w*前后偏置，前移速度补偿=w*左右偏置。
FORMATION_RIGHT_OFFSET_CM = -19.0
FORMATION_FORWARD_OFFSET_CM = 0.0
# 小角速度下不做 w×编队偏置的平移补偿，避免视觉/无线角速度抖动让
# 前后速度反复跳变；实际角速度命令仍按原始 w 下发。
RIGID_SLOT_W_COMP_DEADBAND_RAD_S = 0.3

# 主车发送的平移速度位于主车车体系。视觉灯条给出主车与辅助车的相对
# 航向后，将完整刚体槽位速度旋转到辅助车车体系再与视觉修正相加。
# 新协议的 theta 已是 IPM 后物理角误差；相较旧原始像素角已完成符号
# 翻转，因此这里由 -1 改为 +1，保持同一实车姿态下的坐标旋转方向不变。
SE2_BODY_FRAME_ROTATION_ENABLED = True
SE2_BODY_FRAME_HEADING_SIGN = 1.0

# 视觉闭环角速度专用的旋转中心偏置，不复用无线前馈的 formation offset。
# 视觉 w 的平移补偿：vx += -w*前向偏置，vy += w*右向偏置。
SE2_VISUAL_W_RIGHT_OFFSET_CM = -18.0
SE2_VISUAL_W_FORWARD_OFFSET_CM = 0.0

# 这两个参数只处理用于速度坐标变换的角度；航向 P 仍使用原始角度。
# 0.25° 忽略量化抖动，单帧最多变 1.0°，抑制异常视觉角度突跳。
SE2_BODY_FRAME_ANGLE_DEADBAND_DEG = 0.25
SE2_BODY_FRAME_MAX_STEP_DEG = 1.0
RELATIVE_HEADING_LIMIT_DEG = 45.0

# 无线前馈不可用时，由连续视觉相对位姿和辅助车自身里程计反推主车速度，
# 再使用同一组 formation offset 解算辅助车刚体槽位速度。
VISUAL_RIGID_ENABLED = False

# 主车前馈与视觉修正相加后的最终命令限幅。不能复用上面的视觉限幅，
# 否则主车 90 cm/s 前移只会给辅助车留下 45 cm/s，横移更只剩 20 cm/s。
# 这里与两车相同的 MotorConfig 底盘能力保持一致。
MAX_COMMAND_VX = 700.0
MAX_COMMAND_VY = 700.0
MAX_COMMAND_W = 3.4

# 板端调试输出。比赛前可关闭以减少串口负担。
DEBUG_OUTPUT = True
# 视觉闭环调试：每个有效视觉测量输出 x、y、θ 误差及对应视觉修正
# vx、vy、w。默认关闭，避免影响控制循环和串口带宽。
FOLLOW_CONTROL_DEBUG_OUTPUT = False

# 脱机运行日志。每 200 ms 在内存中采样一次，每 1 s 批量写入板载文件，
# 减少文件系统写入对 10 ms 控制循环的干扰。达到 1 MiB 后停止记录并保留
# 已有数据，不影响车辆继续运行。
OFFLINE_LOG_ENABLED = False
OFFLINE_LOG_PATH = "assistant_follow_log.txt"
OFFLINE_LOG_PERIOD_MS = 200
OFFLINE_LOG_FLUSH_MS = 1000
OFFLINE_LOG_MAX_BYTES = 1024 * 1024


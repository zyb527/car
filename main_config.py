"""主车集中配置。

动作逻辑以 car141929 为基线。旧底盘线速度约 400 对应当前底盘约 200，
因此从旧方案迁移的 vx/vy 参数统一乘 LINEAR_SPEED_SCALE=0.5；
角速度、角度阈值和PID结构保持旧方案。
"""

import math

CONTROL_PERIOD_MS = 20  # 控制算法循环周期（毫秒），20ms 对应 50Hz 控制频率
LINEAR_SPEED_SCALE = 0.5  # 线速度缩放系数（旧底盘与当前底盘速度的缩放比例）

INITIAL_X_CM = 0.0  # 小车初始 X 坐标（厘米）
INITIAL_Y_CM = 0.0  # 小车初始 Y 坐标（厘米）
INITIAL_HEADING_RAD = math.pi / 2.0  # 小车初始航向角（弧度，朝向正北/正上）
INITIAL_HEADING_DEG = 90.0

# 启动盲区：只有首次同时越过这两个世界坐标后，主流程才接受视觉目标。
# 使用严格大于，避免恰好停在边界时误触发目标锁定。
VISUAL_ENABLE_MIN_X_CM = 50.0
VISUAL_ENABLE_MIN_Y_CM = 50.0

# 坐标找物主流程参数。
INITIAL_WAYPOINT = (100.0, 70.0)
POST_PUSH_WAYPOINT_BY_CLASS = {
    1: (100.0, 120.0),
    2: (100.0, 120.0),
    3: (160.0, 180.0),
    4: (220.0, 120.0),
    5: (220.0, 120.0),
}
# 到达 Push 后返场点的固定朝向：1/2=沙包，3=网球，4/5=小熊。
POST_PUSH_FORWARD_HEADING_DEG_BY_CLASS = {
    1: 0.0,
    2: 0.0,
    3: -90.0,
    4: 180.0,
    5: 180.0,
}
POST_PUSH_POINT_WAIT_S = 1.0
POST_PUSH_FORWARD_SPEED_CM_S = 40.0
POST_PUSH_FORWARD_MAX_DISTANCE_CM = 100.0

# Approach 丢失目标并完成一整圈搜索后使用的循环搜物路径。
# 每个三元组为 (世界 X cm, 世界 Y cm, 平移期间保持的世界航向 deg)。
APPROACH_LOSS_SEARCH_WAYPOINTS = (
    (100.0, 160.0, 0.0),
    (100.0, 80.0, 0.0),
    (120.0, 60.0, 90.0),
    (200.0, 60.0, 90.0),
    (220.0, 80.0, 180.0),
    (220.0, 160.0, 180.0),
)
# Push 后转完 180°、返回下一个场地点时的视觉保护门。三元组为
# (世界坐标轴, 需要增加/减少的符号, 最小坐标变化量 cm)。
# 1/2=沙包，3=网球，4/5=小熊。
POST_PUSH_VISUAL_GATE_BY_CLASS = {
    1: ("x", 1.0, 50.0),
    2: ("x", 1.0, 50.0),
    3: ("y", -1.0, 25.0),
    4: ("x", -1.0, 50.0),
    5: ("x", -1.0, 50.0),
}
CLASS_HEADING_DEG = {1: 180.0, 2: 180.0, 3: 90.0, 4: 0.0, 5: 0.0}
TOF_VALID_MIN_MM = 20.0
TOF_VALID_MAX_MM = 1500.0
TOF_ORBIT_ENTRY_MM = 170.0
TOF_EMERGENCY_MM = 120.0
TOF_EMERGENCY_RELEASE_MM = 160.0
TOF_CENTER_OFFSET_MM = 20.0
MISSION_MAX_XY_SPEED_CM_S = 100.0


class NavigationConfig:
    """坐标直线巡航与原地转向相关参数配置。"""

    POSITION_TOLERANCE_CM = 5.0  # 到达目标点的位置距离容差（厘米）
    CROSS_TRACK_TOLERANCE_CM = 5.0  # 沿直线路径巡航时的横向轨迹偏离容差（厘米）

    PATH_FAST_DISTANCE_CM = 55.0  # 快速巡航加速/减速切换距离（厘米）
    PATH_SLOW_DISTANCE_CM = 20.0  # 接近目标点时的低速减速距离阈值（厘米）
    PATH_FAST_SPEED_CM_S = 110.0  # 路径巡航高速阶段线速度（厘米/秒）
    PATH_MID_SPEED_CM_S = 70.0  # 路径巡航中速阶段线速度（厘米/秒）
    PATH_SLOW_SPEED_CM_S = 70.0  # 路径巡航低速接近阶段线速度（厘米/秒）
    PATH_CROSS_KP = 1.2  # 路径横向偏航纠偏比例 P 增益
    PATH_CROSS_MAX_SPEED_CM_S = 60.0  # 路径横向偏航纠偏的最大修正速度（厘米/秒）
    # 只提高坐标巡航的速度上限；不影响 Approach、Orbit 与 Push 的 100 cm/s 上限。
    PATH_MAX_SPEED_CM_S = 150.0

    TRANSLATE_HEADING_KP = 1.8  # 平移巡航时保持航向角不偏转的 P 增益
    TRANSLATE_HEADING_KD = 0.10  # 平移巡航时保持航向角的微分 D 增益
    TRANSLATE_HEADING_DEADBAND_RAD = math.radians(2.0)
    TRANSLATE_MAX_W_RAD_S = 4.0

    TURN_FAST_ERROR_RAD = math.radians(25.0)  # 原地转向：快速旋转阶段角度误差阈值（弧度，25度）
    TURN_MID_ERROR_RAD = math.radians(5.0)
    TURN_FAST_W_RAD_S = 3.14 # 原地转向：高速旋转阶段角速度（弧度/秒）
    TURN_MID_KP = 5.0  # 中速段比例增益（大于慢速段）
    TURN_MID_W_RAD_S = 1.50  # 中速段 P 输出上限（弧度/秒）
    TURN_SLOW_KP = 4.0  # 慢速段比例增益；5 度误差时输出约 0.35 rad/s
    TURN_DAMPING_KD = 0.10
    TURN_TOLERANCE_RAD = math.radians(4.0)
    TURN_YAW_RATE_TOLERANCE_RAD_S = 0.12  # 原地转向完成允许的最大残留角速度上限（弧度/秒）

class ApproachConfig:
    """car141929 逼近（Approach）阶段参数及当前底盘比例换算。"""

    LINEAR_SPEED_SCALE = LINEAR_SPEED_SCALE  # 继承全局线速度比例系数
    TARGET_CENTER_X_PX = 160.0  # 摄像头视野中心 X 轴图像坐标（像素）
    STOP_Y_THRESHOLD_PX = 110.0  # 逼近阶段停止时目标物在图像中的目标 Y 轴坐标（像素）
    VISUAL_STOP_Y_THRESHOLD_BY_CLASS = {3: 120.0}  # 网球视觉完成靠近的 Y 像素阈值
    SLOW_FORWARD_X_ERROR_PX = 80.0  # 当 X 轴偏差大于该值时触发减速前行（像素）
    APPROACH_Y_SLOW_START_PX = 50.0  # 图像 Y 轴方向接近目标时开始前向减速的距离（像素）

    APPROACH_SPEED_CM_S = 100.0
    MIN_APPROACH_SPEED_CM_S = 30.0
    TOF_SLOW_START_MM = 500.0  # ToF 激光传感器开始触发减速前行进给的距离门限（毫米）

    STOP_DISTANCE_MM = TOF_ORBIT_ENTRY_MM
    TENNIS_STOP_DISTANCE_MM = 200.0
    TOF_VALID_MIN_MM = 20.0  # ToF 传感器有效读数最小下门限（毫米）
    TOF_VALID_MAX_MM = TOF_VALID_MAX_MM

    TARGET_ALIGN_ERROR_PX = 10.0
    ALIGN_TIMEOUT_S = 1.0  # 逼近阶段视觉对准等待的最大超时时间（秒）
    TARGET_LOSS_DECAY_S = 0.4  # 丢失视觉目标后速度线性衰减清零的缓冲时间（秒）
    ORBIT_MIN_RADIUS_MM = 0.0
    TOF_FALLBACK_SPEED_CM_S = 30.0
    TOF_FALLBACK_STOP_Y_PX = 120.0
    MAX_XY_SPEED_CM_S = MISSION_MAX_XY_SPEED_CM_S
    VISUAL_STOP_ENABLED = True

    # 逼近阶段 PID 参数
    PID_APPROACH_W_KP = 0.012  # 逼近阶段角速度 PID P 增益
    PID_APPROACH_W_KI = 0.0  # 逼近阶段角速度 PID I 增益
    PID_APPROACH_W_KD = 0.0  # 逼近阶段角速度 PID D 增益
    PID_APPROACH_W_OUTPUT_LIMIT = 3.0  # 逼近阶段角速度 PID 输出上限（弧度/秒）
    PID_APPROACH_W_I_LIMIT = 100.0  # 逼近阶段角速度 PID 积分限幅

class OrbitConfig:
    """car141929 绕行（Orbit）阶段参数及当前底盘比例换算。"""

    LINEAR_SPEED_SCALE = LINEAR_SPEED_SCALE  # 继承全局线速度比例系数
    TARGET_CENTER_X_PX = 160.0  # 摄像头画面物理中心 X 坐标（像素）

    # 绕行/对位完成后，目标需要落在此像素 X 坐标才能处于斜推杆正前方
    ORBIT_ROD_TARGET_X_PX = 65.0
    ORBIT_ROD_TARGET_Y_PX = 140.0
    CAMERA_TURN_DEAD_BAND_X_PX = 15.0
    ORBIT_Y_DEAD_BAND_PX = 15.0

    ORBIT_DIRECTION = "left"
    ORBIT_MIN_RADIUS_MM = 150.0
    TOF_CENTER_OFFSET_MM = TOF_CENTER_OFFSET_MM

    ORBIT_MAX_VX_CM_S = MISSION_MAX_XY_SPEED_CM_S
    ORBIT_MAX_VY_CM_S = MISSION_MAX_XY_SPEED_CM_S
    ORBIT_MAX_W_RAD_S = 4.0  # 绕行阶段最大允许旋转角速度 limit（弧度/秒）
    ORBIT_ROTATION_SPEED_RAD_S = 2.0  # 绕行阶段基础旋转角速度基准值（弧度/秒）
    ORBIT_CAMERA_W_WEIGHT = 0.65  # 视觉辅助转向在合成角速度中所占权重比例 (0.0~1.0)
    ORBIT_W_SCALE = 1.0  # 绕行角速度整体最终输出缩放比

    ORBIT_TOF_WEIGHT = 1.0
    ORBIT_BAND_VY_ENABLED = True  # 是否开启绕行半径安全带 (Radius Band) 径向平移限速功能

    ORBIT_STOP_ERROR_RAD = math.radians(2.0)  # 绕行/对位终止判定合格的角度误差门限（弧度，约 2.0 度）
    ORBIT_ENTER_ALIGN_ERROR_RAD = math.radians(5.0)  # 自动切入推杆对位 (PHASE_ALIGN) 阶段的角度误差门限（弧度，约 5.0 度）
    ORBIT_STOP_X_ERROR_PX = 15.0
    ORBIT_FINAL_ALIGN_X_ERROR_PX = 15.0
    ORBIT_FINAL_ALIGN_Y_ERROR_PX = 15.0
    ORBIT_ALIGN_TIMEOUT_S = 1.0  # 推杆横向对位阶段超时限定时间（秒）
    ORBIT_CLOSE_IN_TIMEOUT_S = 1.0  # 推杆逼近贴靠阶段超时限定时间（秒）
    ORBIT_SLOW_DOWN_START_RAD = math.radians(30.0)  # 接近绕行终点开始降角速度的角度偏差门限（弧度，30度）
    ORBIT_SLOW_DOWN_MIN_SCALE = 0.32  # 接近绕行终点时角速度减速的最小比例下限

    ORBIT_ALIGN_KP = 0.55  # 对位与贴靠阶段纯靠 IMU 维持目标姿态角的 P 增益
    ORBIT_ALIGN_KD = 0.032  # 对位与贴靠阶段纯靠 IMU 维持目标姿态角的 D 增益
    ORBIT_ALIGN_MAX_W_RAD_S = 3.0  # 对位与贴靠阶段允许的最大修正角速度（弧度/秒）

    ORBIT_CLOSE_IN_TENNIS_STOP_MM = TOF_EMERGENCY_MM
    ORBIT_CLOSE_IN_STOP_MM = TOF_EMERGENCY_MM

    TOF_VALID_MIN_MM = 20.0  # ToF 传感器有效读数最小下门限（毫米）
    TOF_VALID_MAX_MM = TOF_VALID_MAX_MM
    TOF_EMERGENCY_MM = TOF_EMERGENCY_MM
    TOF_EMERGENCY_RELEASE_MM = TOF_EMERGENCY_RELEASE_MM
    TOF_EMERGENCY_RETREAT_SPEED_CM_S = 20.0
    MAX_XY_SPEED_CM_S = MISSION_MAX_XY_SPEED_CM_S
    PUSH_READY_STABLE_S = 0.20

    # 绕行与对位 PID 参数
    PID_CAMERA_TURN_KP = 0.0145  # 视觉转角辅助 PID P 增益
    PID_CAMERA_TURN_KI = 0.0  # 视觉转角辅助 PID I 增益
    PID_CAMERA_TURN_KD = 0.0004  # 视觉转角辅助 PID D 增益
    PID_CAMERA_TURN_I_LIMIT = 150.0  # 视觉转角辅助 PID 积分限幅
    PID_CAMERA_TURN_OUTPUT_LIMIT = 1.0  # 视觉转角辅助 PID 输出上限

    PID_ORBIT_TOF_KP = 0.5 * LINEAR_SPEED_SCALE  # ToF 测距半径修正 PID P 增益
    PID_ORBIT_TOF_KI = 0.0  # ToF 测距半径修正 PID I 增益
    PID_ORBIT_TOF_KD = 0.10 * LINEAR_SPEED_SCALE  # ToF 测距半径修正 PID D 增益
    PID_ORBIT_TOF_I_LIMIT = 300.0  # ToF 测距半径修正 PID 积分限幅

    PID_ORBIT_Y_KP = 1.2 * LINEAR_SPEED_SCALE  # 图像 Y 轴像素偏置修正 PID P 增益
    PID_ORBIT_Y_KI = 0.0  # 图像 Y 轴像素偏置修正 PID I 增益
    PID_ORBIT_Y_KD = 0.10 * LINEAR_SPEED_SCALE  # 图像 Y 轴像素偏置修正 PID D 增益
    PID_ORBIT_Y_I_LIMIT = 200.0  # 图像 Y 轴像素偏置修正 PID 积分限幅

    PID_X_KP = 0.67 * LINEAR_SPEED_SCALE  # 图像 X 轴横向平移修正 PID P 增益
    PID_X_KI = 0.03 * LINEAR_SPEED_SCALE  # 图像 X 轴横向平移修正 PID I 增益
    PID_X_KD = 0.2 * LINEAR_SPEED_SCALE  # 图像 X 轴横向平移修正 PID D 增益
    PID_X_I_LIMIT = 100.0  # 图像 X 轴横向平移修正 PID 积分限幅

    ORBIT_ALIGN_MIN_W_RAD_S = 0.45
    ORBIT_ALIGN_MIN_W_ERROR_RAD = math.radians(2.0)
    CONTINUOUS_HOLD = True
    TARGET_LOSS_DECAY_S = 0.4


class PushConfig:
    """推行（Push）阶段参数配置（速度已按当前底盘比例缩放）。"""

    LINEAR_SPEED_SCALE = LINEAR_SPEED_SCALE  # 继承全局线速度比例系数
    # PUSH 顶层状态（普通推行和避障）发送给辅助车的三轴无线速度统一缩放。
    WIRELESS_FEEDFORWARD_SCALE = 0.9

    # 主车接收的是主摄放大后的 320x240 坐标；推行时让物体保持在
    # 主车坐标 (30, 75)，该参考点与双车系统的避障中心相互独立。
    TARGET_CENTER_X_PX = 30.0
    TARGET_Y_PX = 75.0

    PUSH_SPEED_CM_S = 100.0  # 推行阶段斜坡加速后的目标前进速度（厘米/秒）
    PUSH_START_SPEED_CM_S = 70.0 * LINEAR_SPEED_SCALE  # 推行起步时的初始缓启速度（厘米/秒）
    PUSH_RAMP_S = 0.7  # 推行速度平滑斜坡上升斜率加速时间（秒）
    PUSH_DURATION_S = 6.0

    MAX_LATERAL_SPEED_CM_S = 200.0 * LINEAR_SPEED_SCALE  # 推行过程允许的最大横向平移修正速度（厘米/秒）
    MAX_FORWARD_ADJUST_CM_S = 180.0 * LINEAR_SPEED_SCALE  # 推行过程允许的最大前向速度调整量（厘米/秒）
    MAX_W_RAD_S = 1.2  # 推行过程允许的最大修正旋转角速度（弧度/秒）
    HEADING_DEADBAND_RAD = math.radians(1.0)  # 航向控制死区角度（弧度，1度）
    HEADING_KP = 2.0  # 保持推行直线的航向 P 增益
    HEADING_KD = 0.2  # 保持推行直线的航向 D 增益

    PID_X_KP = 0.67 * LINEAR_SPEED_SCALE  # 横向 X 轴像素纠偏 PID P 增益
    PID_X_KI = 0.03 * LINEAR_SPEED_SCALE  # 横向 X 轴像素纠偏 PID I 增益
    PID_X_KD = 0.2 * LINEAR_SPEED_SCALE  # 横向 X 轴像素纠偏 PID D 增益
    PID_X_I_LIMIT = 100.0  # 横向 X 轴像素纠偏 PID 积分限幅
    PID_Y_KP = 0.6 * LINEAR_SPEED_SCALE  # 纵向 Y 轴像素纠偏 PID P 增益
    PID_Y_KI = 0.0  # 纵向 Y 轴像素纠偏 PID I 增益
    PID_Y_KD = 0.0  # 纵向 Y 轴像素纠偏 PID D 增益
    PID_Y_I_LIMIT = 200.0  # 纵向 Y 轴像素纠偏 PID 积分限幅

    TOF_VALID_MIN_MM = 20.0  # ToF 传感器有效读数最小下门限（毫米）
    TOF_VALID_MAX_MM = TOF_VALID_MAX_MM
    CONTACT_DISTANCE_MM = 30.0  # 推杆贴靠目标的触发接触距离（毫米）
    CONTACT_KP = 1.5 * LINEAR_SPEED_SCALE  # 推杆接触控制 PID P 增益
    CONTACT_KI = 0.05 * LINEAR_SPEED_SCALE  # 推杆接触控制 PID I 增益
    CONTACT_KD = 0.2 * LINEAR_SPEED_SCALE  # 推杆接触控制 PID D 增益
    CONTACT_I_LIMIT = 100.0  # 推杆接触控制 PID 积分限幅
    CONTACT_MAX_ADJUST_CM_S = 80.0 * LINEAR_SPEED_SCALE  # 接触调整最大修正速度上限（厘米/秒）

    YELLOW_STOP_DELAY_S = 0.3  # 黄线命中后的继续推行及后续硬停保持时间（秒）
    HAZARD_OBSTACLE = 7  # 危险障碍物分类标识符
    HAZARD_YELLOW = 6  # 黄色避障带分类标识符

    # 双车同侧推行编队避障几何参数
    FORMATION_BASELINE_CM = 20.0  # 双车旋转中心之间的基线物理距离（厘米）
    OBJECT_FORWARD_OFFSET_CM = 10.0  # 物体相对于旋转中心的前向距离偏移（厘米）
    AVOID_CENTER_X_PX = 65.0  # 双车推行系统的避障视觉中心 X 轴像素坐标
    AVOID_CENTER_DEADBAND_PX = 10.0  # 避障视觉中心死区像素
    PREFERRED_AVOID_DIRECTION = "left"  # 偏好避障转向方向 ("left" 左避)
    AVOID_Y_NEAR_PX = 30.0  # 较近预警像素距离
    AVOID_Y_DANGER_PX = 60.0  # 危险临界像素距离
    AVOID_ANGLE_FAR_RAD = math.radians(20.0)  # 远距离避障偏转角（弧度，20度）
    AVOID_ANGLE_NEAR_RAD = math.radians(45.0)  # 中距离避障偏转角（弧度，45度）
    AVOID_ANGLE_DANGER_RAD = math.radians(60.0)  # 危险距离紧急偏转角（弧度，60度）
    AVOID_SPEED_SCALE_FAR = 0.75  # 远距离避障推行减速因子
    AVOID_SPEED_SCALE_NEAR = 0.50  # 中距离避障推行减速因子
    AVOID_SPEED_SCALE_DANGER = 0.30  # 危险距离避障推行减速因子
    AVOID_CLEAR_HOLD_S = 0.5  # 避障警报清除后保持避障姿态的缓冲维持时间（秒）
    AVOID_TARGET_ANGLE_SLEW_RAD_S = math.radians(90.0)  # 避障转向角速率斜率限制（弧度/秒）
    AVOID_RETURN_TOLERANCE_RAD = math.radians(2.0)  # 避障完成后返回原本航向的完成容差（弧度，2度）
    AVOID_RETURN_STABLE_S = 0.10  # 避障归位稳定判定所需持续时间（秒）
    AVOID_MAX_W_RAD_S = MAX_W_RAD_S  # 避障允许的最大转向角速度（弧度/秒）
    AVOID_MAX_W_ACCEL_RAD_S2 = 6.0  # 避障允许的最大角加速度（弧度/秒²）
    AVOID_PIXEL_HYSTERESIS_NEAR = 3.0  # 像素迟滞区间（较近）
    AVOID_PIXEL_HYSTERESIS_DANGER = 5.0  # 像素迟滞区间（危险）
    AVOID_GEAR_DOWN_STABLE_FRAMES = 5

    VEHICLE_MAX_XY_SPEED_CM_S = MISSION_MAX_XY_SPEED_CM_S
    VEHICLE_MAX_W_RAD_S = 3.4  # 底盘最大角速度保护限幅（弧度/秒）

    # 单车推行使用 35→100 cm/s 的线性斜坡；避障状态会在此基础上叠加
    # 目标 Y/ToF 接触闭环，再按危险档位减速并进行刚体解算。
    PUSH_AVOIDANCE_ENABLED = True
    PUSH_SINGLE_VEHICLE_MODE = False
    PUSH_Y_TOF_GOVERNOR_ENABLED = False
    PUSH_FIXED_FORWARD_SPEED_CM_S = None
    TARGET_LOSS_CONTINUE_ENABLED = True
    TARGET_LOSS_FORWARD_SPEED_CM_S = 100.0


class GarageConfig:
    """基于全局坐标与黄线回库的新版参数。"""
    HEADING_KP = 2.0
    HEADING_KD = 0.08
    HEADING_MAX_W = 2.0
    HEADING_TOLERANCE_RAD = math.radians(5.0)
    HEADING_RATE_TOLERANCE_RAD_S = 0.20

    # 运动线速度
    FORWARD_SPEED_CM_S = 100.0
    LATERAL_SPEED_CM_S = 100.0

    # 回库逻辑参数
    X_ADJUST_DISTANCE_CM = 50.0
    YELLOW_Y_THRESHOLD_PX = 60.0
    LATERAL_MAX_Y_CM = -50.0
    LATERAL_TIMEOUT_S = 5.0
    STOP_WAIT_S = 0.5


class MissionConfig:
    """顶层状态机、通信周期和安全超时。"""

    CONTROL_PERIOD_MS = CONTROL_PERIOD_MS
    FEEDFORWARD_ENABLED = True
    FEEDFORWARD_TX_PERIOD_MS = 10
    # 无线前馈 = 实测速度 * 此权重 + S 曲线目标指令 * (1 - 此权重)。
    # vx/vy 使用编码器里程计，w 使用 IMU；调试范围必须为 0.0～1.0。
    FEEDFORWARD_MEASURED_WEIGHT = 0
    START_DELAY_MS = 2000
    INITIAL_HEADING_RESET_TIMEOUT_MS = 100

    MAIN_CAMERA_UART_ID = 7
    MAIN_CAMERA_BAUD = 115200
    MAIN_CAMERA_TIMEOUT_MS = 500
    TOF_TIMEOUT_MS = 300

    SEARCH_W_RAD_S = 1.5
    SEARCH_LOCK_TIMEOUT_S = 8.0
    APPROACH_LOSS_SEARCH_TURN_RAD = 2.0 * math.pi
    APPROACH_LOSS_SEARCH_WAYPOINTS = APPROACH_LOSS_SEARCH_WAYPOINTS
    TARGET_CLASS_IDS = (1, 2, 3, 4, 5)
    TOTAL_OBJECTS_TO_PUSH = 3
    INITIAL_WAYPOINT = INITIAL_WAYPOINT
    VISUAL_ENABLE_MIN_X_CM = VISUAL_ENABLE_MIN_X_CM
    VISUAL_ENABLE_MIN_Y_CM = VISUAL_ENABLE_MIN_Y_CM
    POST_PUSH_WAYPOINT_BY_CLASS = POST_PUSH_WAYPOINT_BY_CLASS
    POST_PUSH_FORWARD_HEADING_DEG_BY_CLASS = (
        POST_PUSH_FORWARD_HEADING_DEG_BY_CLASS
    )
    POST_PUSH_POINT_WAIT_S = POST_PUSH_POINT_WAIT_S
    POST_PUSH_FORWARD_SPEED_CM_S = POST_PUSH_FORWARD_SPEED_CM_S
    POST_PUSH_FORWARD_MAX_DISTANCE_CM = POST_PUSH_FORWARD_MAX_DISTANCE_CM
    POST_PUSH_VISUAL_GATE_BY_CLASS = POST_PUSH_VISUAL_GATE_BY_CLASS
    CLASS_HEADING_DEG = CLASS_HEADING_DEG

    TOF_VALID_MIN_MM = TOF_VALID_MIN_MM
    TOF_VALID_MAX_MM = TOF_VALID_MAX_MM


PATROL_WAYPOINTS = ()  # 巡航路径点列表（默认为空元组）

"""跟随摄像头现场配置。"""

# 辅助摄像头发送串口；已按当前实车接线确认。
UART_ID = 12
BAUD = 230400

# 图像与安装方向（QVGA: 320 x 240）
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
# 当前 OpenART 固件的 QVGA 灰度模式最高可靠设定为 60 FPS；不能直接填 120。
# 若以后为 120 FPS 改用更低分辨率，必须重新标定所有像素参考值和 IPM。
SENSOR_FRAME_RATE = 60
H_MIRROR = False
V_FLIP = False
EXPOSURE_US = 1500
SENSOR_SKIP_MS = 1000

# 灰度亮斑检测
IR_THRESHOLD = (80, 255)
ROI = (0, 0, IMAGE_WIDTH, IMAGE_HEIGHT)
BLOB_PIXELS_MIN = 2
BLOB_AREA_MIN = 2
BLOB_ASPECT_MIN = 0.35
BLOB_ASPECT_MAX = 2.80

# 理想并排队形参考值：实车平行静止采样得到。
REF_CX = 141.0
REF_CY = 67.5
REF_LINE_ANGLE_DEG = 2.92
REF_DISTANCE_PX = 98.1

# 图像到水平灯板平面的逆单应矩阵。输出第一维向车体右侧为正、第二维
# 向车体前方为正，单位 cm。逆透视和物理角度统一在摄像头端完成。
IPM_IMAGE_TO_PLANE = (
    (0.00326432496185, -0.16495807789, 11.9655135432),
    (-0.125367937314, -0.0217332301034, 18.0826401545),
    (-0.0000564214666812, 0.00295916960889, 1.0),
)
IPM_DENOMINATOR_EPSILON = 0.25

# 由当前理想队形的双灯像素均值经上述 IPM 得到。位置跟随永久使用像素
# x 较大的右灯；物理灯线角减去参考角后作为 theta 发送，因此平行时为 0°。
# 更换安装位置或重新标定 IPM 后，必须同步重新采集这些参考值。
REF_TARGET_X_CM = 0.8682933380
REF_TARGET_Y_CM = -6.0654768945
REF_PLANE_ANGLE_DEG = 87.6204502806

# 候选对硬约束
PAIR_DISTANCE_MIN_PX = 20.0
PAIR_DISTANCE_MAX_PX = 150.0
PAIR_SIZE_RATIO_MIN = 0.35
PAIR_ANGLE_TOLERANCE_DEG = 45.0

# 单灯退化仅在已经锁定过双灯后启用。根据最大灯距 150 px 和 QVGA
# 宽度 320 px：x<=150 表示左灯已从左侧出界、剩余点为右灯；x>=170
# 表示右灯已从右侧出界、剩余点为左灯。中间区使用历史端点连续性。
SINGLE_RIGHT_MAX_X = 150.0
SINGLE_LEFT_MIN_X = 170.0
SINGLE_HISTORY_MAX_JUMP_PX = 45.0

# 相邻有效帧最大允许跳变
MAX_MIDPOINT_JUMP_PX = 45.0
MAX_DISTANCE_JUMP_PX = 30.0
MAX_ANGLE_JUMP_DEG = 20.0

# 配对评分权重与归一化尺度
SCORE_REF_MID_WEIGHT = 1.0
SCORE_REF_DISTANCE_WEIGHT = 1.5
SCORE_REF_ANGLE_WEIGHT = 1.0
SCORE_SIZE_BALANCE_WEIGHT = 0.5
SCORE_HISTORY_MID_WEIGHT = 2.0
SCORE_HISTORY_DISTANCE_WEIGHT = 2.0
SCORE_HISTORY_ANGLE_WEIGHT = 2.0
SCORE_MID_SCALE_PX = 80.0
SCORE_DISTANCE_SCALE_PX = 60.0
SCORE_ANGLE_SCALE_DEG = 45.0

# 确认、滤波与发送
ACQUIRE_CONFIRM_FRAMES = 2
LOST_CONFIRM_FRAMES = 3
EMA_ALPHA = 0.85
TX_MIN_INTERVAL_MS = 15
LOST_REPORT_INTERVAL_MS = 200

# 标定时可开启画面和 USB/REPL 日志；跟随运行时关闭日志。
DEBUG_DRAW = False
DEBUG_PRINT = False
DEBUG_PRINT_INTERVAL_MS = 200


TRACKER_CONFIG = {
    "blob_pixels_min": BLOB_PIXELS_MIN,
    "blob_area_min": BLOB_AREA_MIN,
    "blob_aspect_min": BLOB_ASPECT_MIN,
    "blob_aspect_max": BLOB_ASPECT_MAX,
    "ref_cx": REF_CX,
    "ref_cy": REF_CY,
    "ref_line_angle_deg": REF_LINE_ANGLE_DEG,
    "ref_distance_px": REF_DISTANCE_PX,
    "ipm_image_to_plane": IPM_IMAGE_TO_PLANE,
    "ipm_denominator_epsilon": IPM_DENOMINATOR_EPSILON,
    "ref_plane_angle_deg": REF_PLANE_ANGLE_DEG,
    "pair_distance_min_px": PAIR_DISTANCE_MIN_PX,
    "pair_distance_max_px": PAIR_DISTANCE_MAX_PX,
    "pair_size_ratio_min": PAIR_SIZE_RATIO_MIN,
    "pair_angle_tolerance_deg": PAIR_ANGLE_TOLERANCE_DEG,
    "single_right_max_x": SINGLE_RIGHT_MAX_X,
    "single_left_min_x": SINGLE_LEFT_MIN_X,
    "single_history_max_jump_px": SINGLE_HISTORY_MAX_JUMP_PX,
    "max_midpoint_jump_px": MAX_MIDPOINT_JUMP_PX,
    "max_distance_jump_px": MAX_DISTANCE_JUMP_PX,
    "max_angle_jump_deg": MAX_ANGLE_JUMP_DEG,
    "score_ref_mid_weight": SCORE_REF_MID_WEIGHT,
    "score_ref_distance_weight": SCORE_REF_DISTANCE_WEIGHT,
    "score_ref_angle_weight": SCORE_REF_ANGLE_WEIGHT,
    "score_size_balance_weight": SCORE_SIZE_BALANCE_WEIGHT,
    "score_history_mid_weight": SCORE_HISTORY_MID_WEIGHT,
    "score_history_distance_weight": SCORE_HISTORY_DISTANCE_WEIGHT,
    "score_history_angle_weight": SCORE_HISTORY_ANGLE_WEIGHT,
    "score_mid_scale_px": SCORE_MID_SCALE_PX,
    "score_distance_scale_px": SCORE_DISTANCE_SCALE_PX,
    "score_angle_scale_deg": SCORE_ANGLE_SCALE_DEG,
    "acquire_confirm_frames": ACQUIRE_CONFIRM_FRAMES,
    "lost_confirm_frames": LOST_CONFIRM_FRAMES,
    "ema_alpha": EMA_ALPHA,
}

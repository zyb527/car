"""跟随摄像头 UART 接收与纯图像闭环控制。

本模块不导入 machine、seekfree 或 smartcar，可直接在桌面 Python 测试。
"""

import math


EVENT_MEASUREMENT = "measurement"
EVENT_LOST = "lost"
EVENT_INVALID = "invalid"
EVENT_TIMEOUT = "timeout"


def clamp(value, lower, upper):
    if value < lower:
        return lower
    if value > upper:
        return upper
    return value


def _finite(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return False
    return value == value and -1.0e30 < value < 1.0e30


def _ticks_diff(new_value, old_value):
    try:
        import time

        if hasattr(time, "ticks_diff"):
            return time.ticks_diff(new_value, old_value)
    except ImportError:
        pass
    return new_value - old_value


def normalize_line_error_deg(angle):
    """按无方向直线的 180° 周期归一化到 (-90, 90]。"""
    angle = float(angle)
    while angle > 90.0:
        angle -= 180.0
    while angle <= -90.0:
        angle += 180.0
    if abs(angle) < 1.0e-12:
        return 0.0
    return angle


def continuous_deadband(error, deadband):
    """移除中心死区，同时保持阈值两侧输出连续。"""
    error = float(error)
    deadband = max(0.0, float(deadband))
    if error > deadband:
        return error - deadband
    if error < -deadband:
        return error + deadband
    return 0.0


def slot_compensation_w(w, deadband):
    """只为 w×编队偏置项去除小角速度，不改变角速度命令本身。"""
    w = float(w)
    return 0.0 if abs(w) < max(0.0, float(deadband)) else w


def _validated_homography(matrix):
    """校验并复制 3x3 图像到灯板平面的逆单应矩阵。"""
    try:
        if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
            raise ValueError
        result = tuple(
            tuple(float(value) for value in row)
            for row in matrix
        )
    except (TypeError, ValueError):
        raise ValueError("IPM_IMAGE_TO_PLANE must be a finite 3x3 matrix")
    if not all(_finite(value) for row in result for value in row):
        raise ValueError("IPM_IMAGE_TO_PLANE must be a finite 3x3 matrix")
    return result


def project_image_point_to_plane(
    image_x,
    image_y,
    image_to_plane,
    denominator_epsilon=1.0e-6,
):
    """用逆单应矩阵将一个图像点映射到水平灯板平面。"""
    image_x = float(image_x)
    image_y = float(image_y)
    matrix = _validated_homography(image_to_plane)
    if not _finite(image_x) or not _finite(image_y):
        raise ValueError("image point must be finite")
    return _project_validated_image_point_to_plane(
        image_x,
        image_y,
        matrix,
        denominator_epsilon,
    )


def _project_validated_image_point_to_plane(
    image_x,
    image_y,
    matrix,
    denominator_epsilon,
):
    """Fast projection using a matrix already validated at startup."""

    denominator = (
        matrix[2][0] * image_x
        + matrix[2][1] * image_y
        + matrix[2][2]
    )
    epsilon = max(0.0, float(denominator_epsilon))
    if not _finite(denominator) or abs(denominator) <= epsilon:
        raise ValueError("IPM projection denominator is too small")

    plane_right = (
        matrix[0][0] * image_x
        + matrix[0][1] * image_y
        + matrix[0][2]
    ) / denominator
    plane_forward = (
        matrix[1][0] * image_x
        + matrix[1][1] * image_y
        + matrix[1][2]
    ) / denominator
    if not _finite(plane_right) or not _finite(plane_forward):
        raise ValueError("IPM projection produced a non-finite point")
    return plane_right, plane_forward


def measurement_to_plane_pose(
    measurement,
    image_to_plane,
    denominator_epsilon=1.0e-6,
    project_endpoints=True,
    image_to_plane_validated=False,
):
    """计算矫正平面中的中点；端点投影仅供诊断使用。"""
    mid_x = float(measurement["mid_x"])
    mid_y = float(measurement["mid_y"])
    distance_px = float(measurement["distance_px"])
    angle_deg = float(measurement["line_angle_deg"])
    if not all(_finite(value) for value in (
        mid_x,
        mid_y,
        distance_px,
        angle_deg,
    )):
        raise ValueError("camera measurement must be finite")
    if distance_px <= 0.0:
        raise ValueError("distance_px must be positive")

    matrix = (
        image_to_plane
        if image_to_plane_validated
        else _validated_homography(image_to_plane)
    )
    center = _project_validated_image_point_to_plane(
        mid_x,
        mid_y,
        matrix,
        denominator_epsilon,
    )

    # 运行控制只依赖中点的 IPM 坐标；航向使用原始角度加位置补偿。
    # 鱼眼原始坐标上，灯条某一端可能越过单应矩阵的有效区域，即使中点
    # 仍在可靠区域。此时不能让纯诊断计算中断整个跟随循环。
    if not project_endpoints:
        return {
            "right": center[0],
            "forward": center[1],
            "line_angle_deg": angle_deg,
            "light_separation": None,
            "first": None,
            "second": None,
        }

    angle_rad = math.radians(angle_deg)
    half_dx = 0.5 * distance_px * math.cos(angle_rad)
    half_dy = 0.5 * distance_px * math.sin(angle_rad)
    first = _project_validated_image_point_to_plane(
        mid_x - half_dx,
        mid_y - half_dy,
        matrix,
        denominator_epsilon,
    )
    second = _project_validated_image_point_to_plane(
        mid_x + half_dx,
        mid_y + half_dy,
        matrix,
        denominator_epsilon,
    )

    delta_right = second[0] - first[0]
    delta_forward = second[1] - first[1]
    separation = math.sqrt(
        delta_right * delta_right
        + delta_forward * delta_forward
    )
    if not _finite(separation) or separation <= 0.0:
        raise ValueError("IPM produced coincident light points")

    return {
        # IPM 由实测灯板中点标定，因此中心直接映射；端点映射只用于
        # 矫正后角度、灯距和诊断，不能再反过来平均为中心。
        "right": center[0],
        "forward": center[1],
        "line_angle_deg": normalize_line_error_deg(
            math.degrees(math.atan2(delta_forward, delta_right))
        ),
        "light_separation": separation,
        "first": first,
        "second": second,
    }


def prepare_parallel_angle_calibration(calibration_samples):
    """预计算无向角二倍角的正余弦，避免控制循环重复三角运算。"""
    if not calibration_samples:
        raise ValueError("PARALLEL_ANGLE_CALIBRATION must not be empty")

    prepared = []
    for sample in calibration_samples:
        if len(sample) != 3:
            raise ValueError(
                "parallel angle samples must be (mid_x, mid_y, angle_deg)"
            )
        sample_x = float(sample[0])
        sample_y = float(sample[1])
        sample_angle = float(sample[2])
        if not all(_finite(value) for value in (
            sample_x,
            sample_y,
            sample_angle,
        )):
            raise ValueError("parallel angle samples must be finite")
        doubled = math.radians(2.0 * sample_angle)
        prepared.append(
            (
                sample_x,
                sample_y,
                sample_angle,
                math.sin(doubled),
                math.cos(doubled),
            )
        )
    return tuple(prepared)


def interpolate_parallel_image_angle_deg(
    mid_x,
    mid_y,
    calibration_samples,
):
    """按图像位置插值物理平行灯板应呈现的原始鱼眼角度。"""
    mid_x = float(mid_x)
    mid_y = float(mid_y)
    if not calibration_samples:
        raise ValueError("PARALLEL_ANGLE_CALIBRATION must not be empty")

    weighted_sin = 0.0
    weighted_cos = 0.0
    total_weight = 0.0
    for sample in calibration_samples:
        if len(sample) == 3:
            # 保持公共函数兼容；FollowController 运行时传入的是预计算的
            # 五字段样本，不会进入这个较慢的兼容分支。
            sample_x = float(sample[0])
            sample_y = float(sample[1])
            sample_angle = float(sample[2])
            if not all(_finite(value) for value in (
                sample_x,
                sample_y,
                sample_angle,
            )):
                raise ValueError("parallel angle samples must be finite")
            doubled = math.radians(2.0 * sample_angle)
            sample_sin = math.sin(doubled)
            sample_cos = math.cos(doubled)
        elif len(sample) == 5:
            sample_x = sample[0]
            sample_y = sample[1]
            sample_angle = sample[2]
            sample_sin = sample[3]
            sample_cos = sample[4]
        else:
            raise ValueError(
                "parallel angle samples must have 3 or 5 fields"
            )

        dx = mid_x - sample_x
        dy = mid_y - sample_y
        distance_squared = dx * dx + dy * dy
        if distance_squared <= 1.0e-12:
            return normalize_line_error_deg(sample_angle)

        # 反距离平方加权；无向直线用二倍角平均，正确处理 ±90° 换边。
        weight = 1.0 / distance_squared
        weighted_sin += weight * sample_sin
        weighted_cos += weight * sample_cos
        total_weight += weight

    if total_weight <= 0.0:
        raise ValueError("invalid parallel angle calibration weights")
    return normalize_line_error_deg(
        0.5 * math.degrees(
            math.atan2(weighted_sin, weighted_cos)
        )
    )


def evaluate_parallel_angle_model(
    mid_x,
    mid_y,
    origin,
    inverse_scale,
    coefficients,
):
    """Evaluate the low-cost quadratic model for the parallel image angle."""
    if (
        len(origin) != 2
        or len(inverse_scale) != 2
        or len(coefficients) != 6
    ):
        raise ValueError("invalid parallel angle model dimensions")
    values = tuple(origin) + tuple(inverse_scale) + tuple(coefficients)
    if not all(_finite(float(value)) for value in values):
        raise ValueError("parallel angle model values must be finite")

    prepared = (
        tuple(float(value) for value in origin),
        tuple(float(value) for value in inverse_scale),
        tuple(float(value) for value in coefficients),
    )
    return _evaluate_prepared_parallel_angle_model(mid_x, mid_y, prepared)


def _evaluate_prepared_parallel_angle_model(mid_x, mid_y, prepared):
    """Fast path for a model validated and converted at controller startup."""
    origin, inverse_scale, coefficients = prepared
    x = (float(mid_x) - origin[0]) * inverse_scale[0]
    y = (float(mid_y) - origin[1]) * inverse_scale[1]
    c0, c1, c2, c3, c4, c5 = coefficients
    return normalize_line_error_deg(
        c0
        + c1 * x
        + c2 * y
        + c3 * x * x
        + c4 * x * y
        + c5 * y * y
    )


def piecewise_linear_response(error, points, output_limit):
    """按绝对误差节点线性插值，并恢复误差符号。"""
    magnitude = abs(float(error))
    output_limit = max(0.0, float(output_limit))
    if not points:
        raise ValueError("response points must not be empty")

    previous_error = float(points[0][0])
    previous_output = float(points[0][1])
    if previous_error != 0.0 or previous_output != 0.0:
        raise ValueError("response points must start at (0, 0)")

    output = previous_output
    for point in points[1:]:
        current_error = float(point[0])
        current_output = float(point[1])
        if current_error <= previous_error:
            raise ValueError("response error points must increase")
        if current_output < previous_output:
            raise ValueError("response outputs must not decrease")
        if magnitude <= current_error:
            ratio = (magnitude - previous_error) / (
                current_error - previous_error
            )
            output = previous_output + ratio * (
                current_output - previous_output
            )
            break
        previous_error = current_error
        previous_output = current_output
        output = current_output

    output = clamp(output, 0.0, output_limit)
    return -output if float(error) < 0.0 else output


def _invalid(reason, line=None):
    event = {"type": EVENT_INVALID, "reason": reason}
    if line is not None:
        event["line"] = line
    return event


def parse_camera_line(line, config):
    """解析摄像头输出的物理平面三元组。"""
    try:
        if isinstance(line, bytes):
            text = line.decode("ascii")
        else:
            text = str(line)
    except Exception:
        return _invalid("decode_error")

    text = text.strip()
    if text == "0":
        return {"type": EVENT_LOST}

    fields = text.split(",")
    if len(fields) != 3:
        return _invalid("field_count", text)
    try:
        target_x, target_y, theta_deg = (
            float(field.strip()) for field in fields
        )
    except (TypeError, ValueError):
        return _invalid("number_format", text)
    if not all(_finite(value) for value in (
        target_x,
        target_y,
        theta_deg,
    )):
        return _invalid("non_finite", text)
    if not (
        float(config.TARGET_X_MIN_CM)
        <= target_x
        <= float(config.TARGET_X_MAX_CM)
    ):
        return _invalid("target_x_range", text)
    if not (
        float(config.TARGET_Y_MIN_CM)
        <= target_y
        <= float(config.TARGET_Y_MAX_CM)
    ):
        return _invalid("target_y_range", text)
    if not (
        float(config.THETA_MIN_DEG)
        <= theta_deg
        <= float(config.THETA_MAX_DEG)
    ):
        return _invalid("theta_range", text)
    return {
        "type": EVENT_MEASUREMENT,
        "measurement": {
            "found": True,
            "target_x_cm": target_x,
            "target_y_cm": target_y,
            "theta_deg": theta_deg,
            "coordinate_space": "plane",
        },
    }


class FollowSensor:
    """非阻塞 UART 字节流解析器和摄像头新鲜度监视器。"""

    def __init__(self, uart, config):
        self.uart = uart
        self.config = config
        self.buffer = b""
        self.latest_measurement = None
        self.last_valid_ms = None
        self._timeout_reported = False

    def _read_uart(self):
        try:
            available = int(self.uart.any())
            if available <= 0:
                return None, None
            maximum = max(
                1,
                int(
                    getattr(
                        self.config,
                        "UART_READ_MAX_BYTES",
                        available,
                    )
                ),
            )
            # 只取调用 any() 时已经位于硬件缓存中的字节。给 read() 明确
            # 长度，避免摄像头连续发帧时等待下一批字符或默认超时。
            data = self.uart.read(min(available, maximum))
        except Exception:
            return None, _invalid("uart_read_error")

        if data is None:
            return None, None
        if isinstance(data, str):
            try:
                data = data.encode("ascii")
            except Exception:
                return None, _invalid("uart_decode_error")
        try:
            return bytes(data), None
        except Exception:
            return None, _invalid("uart_data_type")

    def poll(self, now_ms):
        events = []
        data, read_error = self._read_uart()
        if read_error is not None:
            # 单次 UART 坏帧只记录诊断，继续保留上一有效测量；真正失效
            # 统一由 CAMERA_TIMEOUT_MS 判定。
            events.append(read_error)
        elif data:
            self.buffer += data
            if len(self.buffer) > int(self.config.UART_BUFFER_MAX_BYTES):
                self.buffer = b""
                events.append(_invalid("buffer_overflow"))
            else:
                parts = self.buffer.split(b"\n")
                self.buffer = parts[-1]
                complete_lines = parts[:-1]
                if complete_lines:
                    # 控制只需要最新测量。旧测量全部丢弃，避免主循环落后
                    # 时逐帧做 ASCII/float 解析；但旧帧中的明确 LOST 仍保留，
                    # 防止同批后续测量掩盖安全事件。
                    stale_lost = False
                    for stale_line in complete_lines[:-1]:
                        if stale_line.rstrip(b"\r") == b"0":
                            stale_lost = True
                            break
                    if stale_lost:
                        events.append(
                            {
                                "type": EVENT_LOST,
                                "timestamp_ms": now_ms,
                            }
                        )
                        self.latest_measurement = None
                        self._timeout_reported = True

                    line = complete_lines[-1].rstrip(b"\r")
                    event = parse_camera_line(line, self.config)
                    event["timestamp_ms"] = now_ms
                    events.append(event)

                    if event["type"] == EVENT_MEASUREMENT:
                        self.latest_measurement = event["measurement"]
                        self.last_valid_ms = now_ms
                        self._timeout_reported = False
                    elif event["type"] == EVENT_LOST:
                        self.latest_measurement = None
                        self._timeout_reported = True
                    # EVENT_INVALID 只丢弃当前坏帧，不清除上一有效测量。

        if (
            self.last_valid_ms is not None
            and not self._timeout_reported
            and _ticks_diff(now_ms, self.last_valid_ms)
            > int(self.config.CAMERA_TIMEOUT_MS)
        ):
            self.latest_measurement = None
            self._timeout_reported = True
            events.append(
                {
                    "type": EVENT_TIMEOUT,
                    "timestamp_ms": now_ms,
                    "age_ms": _ticks_diff(now_ms, self.last_valid_ms),
                }
            )

        return events


class FollowController:
    """逆透视平面中的相对位姿控制器。

    位置通道使用分段 P 加可选的 alpha-beta 速度估计/D 修正；航向通道
    继续使用分段 P。relative_heading_rad 供运行时把主车车体系前馈旋转
    到辅助车车体系，不经过角度死区。
    """

    def __init__(self, config):
        self.config = config
        self.camera_outputs_plane_pose = bool(
            getattr(config, "CAMERA_OUTPUTS_PLANE_POSE", False)
        )
        self.image_to_plane = None
        self.parallel_angle_model = None
        self.parallel_angle_calibration = None
        self.ipm_denominator_epsilon = 0.0
        if self.camera_outputs_plane_pose:
            self.reference_plane_pose = {
                "right": float(config.REF_TARGET_X_CM),
                "forward": float(config.REF_TARGET_Y_CM),
                "line_angle_deg": float(config.REF_TARGET_THETA_DEG),
                "light_separation": None,
                "first": None,
                "second": None,
            }
            if not all(_finite(value) for value in (
                self.reference_plane_pose["right"],
                self.reference_plane_pose["forward"],
                self.reference_plane_pose["line_angle_deg"],
            )):
                raise ValueError("camera plane reference must be finite")
        else:
            # 兼容旧摄像头固件和桌面测试；生产配置不进入此分支。
            if not bool(config.IPM_CALIBRATED):
                raise ValueError(
                    "IPM is not calibrated; set IPM_CALIBRATED=True "
                    "after filling IPM_IMAGE_TO_PLANE"
                )
            self.image_to_plane = _validated_homography(
                config.IPM_IMAGE_TO_PLANE
            )
            model_coefficients = getattr(
                config,
                "PARALLEL_ANGLE_MODEL_COEFFICIENTS",
                None,
            )
            if model_coefficients is not None:
                self.parallel_angle_model = (
                    tuple(
                        float(value)
                        for value in config.PARALLEL_ANGLE_MODEL_ORIGIN
                    ),
                    tuple(
                        float(value)
                        for value in config.PARALLEL_ANGLE_MODEL_INV_SCALE
                    ),
                    tuple(float(value) for value in model_coefficients),
                )
                # Validate once at startup, not inside every control update.
                evaluate_parallel_angle_model(
                    config.REF_CX,
                    config.REF_CY,
                    *self.parallel_angle_model
                )
            else:
                self.parallel_angle_calibration = (
                    prepare_parallel_angle_calibration(
                        config.PARALLEL_ANGLE_CALIBRATION
                    )
                )
            self.ipm_denominator_epsilon = float(
                config.IPM_DENOMINATOR_EPSILON
            )
            reference_measurement = {
                "mid_x": config.REF_CX,
                "mid_y": config.REF_CY,
                "distance_px": config.REF_DISTANCE_PX,
                "line_angle_deg": config.REF_LINE_ANGLE_DEG,
            }
            self.reference_plane_pose = measurement_to_plane_pose(
                reference_measurement,
                self.image_to_plane,
                self.ipm_denominator_epsilon,
                image_to_plane_validated=True,
            )
        self.last_measurement_ms = None
        self.last_errors = {
            "front": 0.0,
            "lateral": 0.0,
            "angle": 0.0,
        }
        self.last_error_rates = {
            "front": 0.0,
            "lateral": 0.0,
        }
        self.position_estimate = {
            "front": None,
            "lateral": None,
        }
        self.position_rate_estimate = {
            "front": 0.0,
            "lateral": 0.0,
        }
        self.follower_twist = None
        self.previous_follower_twist = None
        self.visual_rigid_command = None
        self.visual_rigid_pose_estimate = None
        self.visual_main_twist_estimate = None
        self.visual_rigid_last_predict_ms = None
        self.visual_rigid_last_measurement_ms = None
        self.relative_heading_rad = 0.0
        self.raw_relative_heading_rad = 0.0
        self.last_plane_pose = None
        self.last_command = (0.0, 0.0, 0.0)

    def reset(self):
        self.last_measurement_ms = None
        self.last_errors["front"] = 0.0
        self.last_errors["lateral"] = 0.0
        self.last_errors["angle"] = 0.0
        self.last_error_rates["front"] = 0.0
        self.last_error_rates["lateral"] = 0.0
        self.position_estimate["front"] = None
        self.position_estimate["lateral"] = None
        self.position_rate_estimate["front"] = 0.0
        self.position_rate_estimate["lateral"] = 0.0
        self.follower_twist = None
        self.previous_follower_twist = None
        self.visual_rigid_command = None
        self.visual_rigid_pose_estimate = None
        self.visual_main_twist_estimate = None
        self.visual_rigid_last_predict_ms = None
        self.visual_rigid_last_measurement_ms = None
        self.relative_heading_rad = 0.0
        self.raw_relative_heading_rad = 0.0
        self.last_plane_pose = None
        self.last_command = (0.0, 0.0, 0.0)

    def set_follower_twist(self, vx, vy, w):
        """保存辅助车里程计实测车体速度，供纯视觉刚体解算使用。"""
        values = (float(vx), float(vy), float(w))
        if not all(_finite(value) for value in values):
            self.follower_twist = None
            return
        self.follower_twist = values

    def _update_visual_rigid_command(
        self,
        lateral_position,
        front_position,
        relative_heading_rad,
        now_ms,
    ):
        """用视觉残差校正主车速度状态，不对相邻视觉帧直接求速度。"""
        if not bool(
            getattr(self.config, "VISUAL_RIGID_ENABLED", True)
        ) or self.follower_twist is None:
            self.visual_rigid_command = None
            self.visual_rigid_pose_estimate = None
            self.visual_main_twist_estimate = None
            self.visual_rigid_last_predict_ms = None
            self.visual_rigid_last_measurement_ms = None
            self.previous_follower_twist = self.follower_twist
            return

        self.predict_visual_rigid(now_ms)
        measurement = (
            float(lateral_position),
            float(front_position),
            float(relative_heading_rad),
        )
        if self.visual_rigid_pose_estimate is None:
            self.visual_rigid_pose_estimate = measurement
            self.visual_main_twist_estimate = (0.0, 0.0, 0.0)
            self.visual_rigid_last_predict_ms = now_ms
            self.visual_rigid_last_measurement_ms = now_ms
            self.previous_follower_twist = self.follower_twist
            self.visual_rigid_command = None
            return

        previous_ms = self.visual_rigid_last_measurement_ms
        dt = _ticks_diff(now_ms, previous_ms) / 1000.0
        dt = clamp(
            dt,
            float(getattr(self.config, "PI_DT_MIN_S", 0.005)),
            float(getattr(self.config, "PI_DT_MAX_S", 0.100)),
        )
        alpha = clamp(
            float(getattr(self.config, "POSITION_FILTER_ALPHA", 1.0)),
            0.0,
            1.0,
        )
        beta = clamp(
            float(getattr(self.config, "POSITION_FILTER_BETA", 0.0)),
            0.0,
            1.0,
        )
        predicted = self.visual_rigid_pose_estimate
        residual_right = measurement[0] - predicted[0]
        residual_forward = measurement[1] - predicted[1]
        residual_heading = math.radians(
            normalize_line_error_deg(
                math.degrees(measurement[2] - predicted[2])
            )
        )
        self.visual_rigid_pose_estimate = (
            predicted[0] + alpha * residual_right,
            predicted[1] + alpha * residual_forward,
            math.radians(
                normalize_line_error_deg(
                    math.degrees(predicted[2] + alpha * residual_heading)
                )
            ),
        )
        main_vx, main_vy, main_w = self.visual_main_twist_estimate
        max_vx = float(
            getattr(self.config, "MAX_COMMAND_VX", self.config.MAX_VX)
        )
        max_vy = float(
            getattr(self.config, "MAX_COMMAND_VY", self.config.MAX_VY)
        )
        max_w = float(
            getattr(self.config, "MAX_COMMAND_W", self.config.MAX_W)
        )
        self.visual_main_twist_estimate = (
            clamp(main_vx + beta * residual_right / dt, -max_vx, max_vx),
            clamp(main_vy + beta * residual_forward / dt, -max_vy, max_vy),
            clamp(main_w + beta * residual_heading / dt, -max_w, max_w),
        )
        self.visual_rigid_last_measurement_ms = now_ms
        self.visual_rigid_last_predict_ms = now_ms
        self._refresh_visual_rigid_command()

    def predict_visual_rigid(self, now_ms):
        """按辅助车实测速度每个主循环推进纯视觉刚体状态。"""
        if not bool(
            getattr(self.config, "VISUAL_RIGID_ENABLED", True)
        ):
            self.visual_rigid_command = None
            return
        if (
            self.follower_twist is None
            or self.visual_rigid_pose_estimate is None
            or self.visual_main_twist_estimate is None
        ):
            return
        previous_ms = self.visual_rigid_last_predict_ms
        if previous_ms is None:
            self.visual_rigid_last_predict_ms = now_ms
            return
        elapsed_s = _ticks_diff(now_ms, previous_ms) / 1000.0
        if elapsed_s <= 0.0:
            return
        dt = clamp(
            elapsed_s,
            float(getattr(self.config, "PI_DT_MIN_S", 0.005)),
            float(getattr(self.config, "PI_DT_MAX_S", 0.100)),
        )
        previous_twist = (
            self.previous_follower_twist
            if self.previous_follower_twist is not None
            else self.follower_twist
        )
        follower_vx = 0.5 * (
            previous_twist[0] + self.follower_twist[0]
        )
        follower_vy = 0.5 * (
            previous_twist[1] + self.follower_twist[1]
        )
        follower_w = 0.5 * (
            previous_twist[2] + self.follower_twist[2]
        )
        right, forward, heading = self.visual_rigid_pose_estimate
        main_vx, main_vy, main_w = self.visual_main_twist_estimate
        # right/forward 保存的是相对理想编队位置的误差，而旋转坐标系中的
        # -w×p 项必须使用主车相对辅助车的完整位置 p。辅助车相对主车的
        # 配置偏移取反后，才是主车相对辅助车的参考位置。
        reference_right = -float(
            getattr(self.config, "FORMATION_RIGHT_OFFSET_CM", 0.0)
        )
        reference_forward = -float(
            getattr(self.config, "FORMATION_FORWARD_OFFSET_CM", 0.0)
        )
        absolute_right = reference_right + right
        absolute_forward = reference_forward + forward
        self.visual_rigid_pose_estimate = (
            right
            + dt
            * (
                main_vx
                - follower_vx
                + follower_w * absolute_forward
            ),
            forward
            + dt
            * (
                main_vy
                - follower_vy
                - follower_w * absolute_right
            ),
            math.radians(
                normalize_line_error_deg(
                    math.degrees(heading + dt * (main_w - follower_w))
                )
            ),
        )
        self.previous_follower_twist = self.follower_twist
        self.visual_rigid_last_predict_ms = now_ms
        self._refresh_visual_rigid_command()

    def _refresh_visual_rigid_command(self):
        if self.visual_main_twist_estimate is None:
            self.visual_rigid_command = None
            return
        main_vx, main_vy, main_w = self.visual_main_twist_estimate
        reference_right = -float(
            getattr(self.config, "FORMATION_RIGHT_OFFSET_CM", 0.0)
        )
        reference_forward = -float(
            getattr(self.config, "FORMATION_FORWARD_OFFSET_CM", 0.0)
        )
        compensation_w = slot_compensation_w(
            main_w,
            getattr(self.config, "RIGID_SLOT_W_COMP_DEADBAND_RAD_S", 0.0),
        )
        slot_vx = main_vx + compensation_w * reference_forward
        slot_vy = main_vy - compensation_w * reference_right
        max_vx = float(
            getattr(self.config, "MAX_COMMAND_VX", self.config.MAX_VX)
        )
        max_vy = float(
            getattr(self.config, "MAX_COMMAND_VY", self.config.MAX_VY)
        )
        max_w = float(
            getattr(self.config, "MAX_COMMAND_W", self.config.MAX_W)
        )
        self.visual_rigid_command = (
            clamp(slot_vx, -max_vx, max_vx),
            clamp(slot_vy, -max_vy, max_vy),
            clamp(main_w, -max_w, max_w),
        )

    def get_visual_rigid_command(self):
        return self.visual_rigid_command

    def _update_position_estimate(
        self,
        lateral_measurement,
        front_measurement,
        now_ms,
    ):
        previous_ms = self.last_measurement_ms
        measurements = {
            "lateral": float(lateral_measurement),
            "front": float(front_measurement),
        }
        if (
            previous_ms is None
            or self.position_estimate["lateral"] is None
            or self.position_estimate["front"] is None
        ):
            for name in ("lateral", "front"):
                self.position_estimate[name] = measurements[name]
                self.position_rate_estimate[name] = 0.0
            return

        dt = _ticks_diff(now_ms, previous_ms) / 1000.0
        dt = clamp(
            dt,
            float(getattr(self.config, "PI_DT_MIN_S", 0.005)),
            float(getattr(self.config, "PI_DT_MAX_S", 0.100)),
        )
        alpha = clamp(
            float(getattr(self.config, "POSITION_FILTER_ALPHA", 1.0)),
            0.0,
            1.0,
        )
        beta = clamp(
            float(getattr(self.config, "POSITION_FILTER_BETA", 0.0)),
            0.0,
            1.0,
        )
        for name in ("lateral", "front"):
            predicted = (
                self.position_estimate[name]
                + self.position_rate_estimate[name] * dt
            )
            residual = measurements[name] - predicted
            self.position_estimate[name] = predicted + alpha * residual
            self.position_rate_estimate[name] += beta * residual / dt

    def update(self, measurement, now_ms):
        if self.camera_outputs_plane_pose:
            try:
                target_right = float(measurement["target_x_cm"])
                target_forward = float(measurement["target_y_cm"])
                measured_theta = float(measurement["theta_deg"])
            except (KeyError, TypeError, ValueError):
                raise ValueError("camera plane measurement is incomplete")
            if not all(_finite(value) for value in (
                target_right,
                target_forward,
                measured_theta,
            )):
                raise ValueError("camera plane measurement must be finite")
            plane_pose = {
                "right": target_right,
                "forward": target_forward,
                "line_angle_deg": measured_theta,
                "light_separation": None,
                "first": None,
                "second": None,
            }
            raw_angle_error = normalize_line_error_deg(
                measured_theta
                - self.reference_plane_pose["line_angle_deg"]
            )
        else:
            plane_pose = measurement_to_plane_pose(
                measurement,
                self.image_to_plane,
                self.ipm_denominator_epsilon,
                project_endpoints=False,
                image_to_plane_validated=True,
            )
            if self.parallel_angle_model is not None:
                parallel_angle = _evaluate_prepared_parallel_angle_model(
                    measurement["mid_x"],
                    measurement["mid_y"],
                    self.parallel_angle_model,
                )
            else:
                parallel_angle = interpolate_parallel_image_angle_deg(
                    measurement["mid_x"],
                    measurement["mid_y"],
                    self.parallel_angle_calibration,
                )
            raw_angle_error = normalize_line_error_deg(
                float(measurement["line_angle_deg"]) - parallel_angle
            )
        lateral_measurement = (
            plane_pose["right"]
            - self.reference_plane_pose["right"]
        )
        front_measurement = (
            plane_pose["forward"]
            - self.reference_plane_pose["forward"]
        )
        self._update_position_estimate(
            lateral_measurement,
            front_measurement,
            now_ms,
        )
        lateral_error = continuous_deadband(
            self.position_estimate["lateral"],
            self.config.LATERAL_DEADBAND_PLANE,
        )
        front_error = continuous_deadband(
            self.position_estimate["front"],
            self.config.FRONT_DEADBAND_PLANE,
        )
        angle_error = continuous_deadband(
            raw_angle_error,
            self.config.ANGLE_DEADBAND_DEG,
        )
        self.raw_relative_heading_rad = math.radians(raw_angle_error)
        heading_sign = -1.0 if float(self.config.W_SIGN) < 0.0 else 1.0
        self.relative_heading_rad = math.radians(
            heading_sign * raw_angle_error
        )
        self._update_visual_rigid_command(
            self.position_estimate["lateral"],
            self.position_estimate["front"],
            self.relative_heading_rad,
            now_ms,
        )
        self.last_measurement_ms = now_ms
        lateral_rate = (
            continuous_deadband(
                self.position_rate_estimate["lateral"],
                float(getattr(self.config, "LATERAL_RATE_DEADBAND_PLANE", 0.0))
            )
            if lateral_error != 0.0
            else 0.0
        )
        front_rate = (
            continuous_deadband(
                self.position_rate_estimate["front"],
                float(getattr(self.config, "FRONT_RATE_DEADBAND_PLANE", 0.0))
            )
            if front_error != 0.0
            else 0.0
        )
        self.last_errors = {
            "front": front_error,
            "lateral": lateral_error,
            "angle": angle_error,
        }
        self.last_error_rates = {
            "front": front_rate,
            "lateral": lateral_rate,
        }
        self.last_plane_pose = plane_pose

        front_output = piecewise_linear_response(
            front_error,
            self.config.FRONT_RESPONSE_POINTS_PLANE,
            self.config.MAX_VY,
        )
        lateral_output = piecewise_linear_response(
            lateral_error,
            self.config.LATERAL_RESPONSE_POINTS_PLANE,
            self.config.MAX_VX,
        )
        angle_output = piecewise_linear_response(
            angle_error,
            self.config.ANGLE_RESPONSE_POINTS,
            self.config.MAX_W,
        )
        front_closing = (
            front_error != 0.0
            and front_rate != 0.0
            and front_error * front_rate < 0.0
        )
        if front_closing:
            minimum_p_scale = clamp(
                float(
                    getattr(
                        self.config,
                        "FRONT_CLOSING_P_SCALE",
                        1.0,
                    )
                ),
                0.0,
                1.0,
            )
            closing_band = max(
                0.0,
                float(
                    getattr(
                        self.config,
                        "FRONT_CLOSING_P_BAND_PLANE",
                        0.0,
                    )
                ),
            )
            if closing_band > 0.0:
                near_ratio = clamp(
                    1.0 - abs(front_error) / closing_band,
                    0.0,
                    1.0,
                )
                p_scale = 1.0 - (
                    1.0 - minimum_p_scale
                ) * near_ratio
            else:
                p_scale = minimum_p_scale
            front_output *= p_scale
            front_d_gain = float(
                getattr(self.config, "FRONT_CLOSING_D_GAIN", 0.0)
            )
        else:
            front_d_gain = float(
                getattr(self.config, "FRONT_OPENING_D_GAIN", 0.0)
            )
        front_output += clamp(
            front_d_gain * front_rate,
            -float(getattr(self.config, "MAX_FRONT_D_TRIM", 0.0)),
            float(getattr(self.config, "MAX_FRONT_D_TRIM", 0.0)),
        )
        lateral_output += clamp(
            float(getattr(self.config, "LATERAL_D_GAIN", 0.0))
            * lateral_rate,
            -float(getattr(self.config, "MAX_LATERAL_D_TRIM", 0.0)),
            float(getattr(self.config, "MAX_LATERAL_D_TRIM", 0.0)),
        )

        command = (
            clamp(
                float(self.config.VX_SIGN) * lateral_output,
                -float(self.config.MAX_VX),
                float(self.config.MAX_VX),
            ),
            clamp(
                float(self.config.VY_SIGN) * front_output,
                -float(self.config.MAX_VY),
                float(self.config.MAX_VY),
            ),
            clamp(
                float(self.config.W_SIGN) * angle_output,
                -float(self.config.MAX_W),
                float(self.config.MAX_W),
            ),
        )
        if not all(_finite(value) for value in command):
            self.reset()
            raise ValueError("non-finite follow command")
        self.last_command = command
        return command

    def get_state(self):
        return {
            "last_measurement_ms": self.last_measurement_ms,
            "errors": dict(self.last_errors),
            "error_rates": dict(self.last_error_rates),
            "relative_heading_rad": self.relative_heading_rad,
            "raw_relative_heading_rad": self.raw_relative_heading_rad,
            "visual_rigid_command": self.visual_rigid_command,
            "visual_main_twist_estimate": self.visual_main_twist_estimate,
            "plane_pose": (
                None
                if self.last_plane_pose is None
                else dict(self.last_plane_pose)
            ),
            "reference_plane_pose": dict(self.reference_plane_pose),
            "command": self.last_command,
        }

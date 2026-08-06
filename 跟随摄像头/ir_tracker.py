"""双红外点识别与跟踪的纯算法模块。

本文件不导入 OpenART 硬件模块，因此可以在桌面 Python 上直接测试。
"""

import math


def normalize_line_angle_deg(angle):
    """把无方向直线角归一化到 (-90, 90] 度。"""
    angle = float(angle)
    while angle > 90.0:
        angle -= 180.0
    while angle <= -90.0:
        angle += 180.0
    if abs(angle) < 1e-9:
        return 0.0
    return angle


def _is_finite_number(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return False
    return value == value and -1e308 < value < 1e308


def _validated_homography(matrix):
    """校验并复制摄像头端使用的 3x3 图像到物理平面矩阵。"""
    try:
        if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
            raise ValueError
        result = tuple(
            tuple(float(value) for value in row)
            for row in matrix
        )
    except (TypeError, ValueError):
        raise ValueError("ipm_image_to_plane must be a finite 3x3 matrix")
    if not all(_is_finite_number(value) for row in result for value in row):
        raise ValueError("ipm_image_to_plane must be a finite 3x3 matrix")
    return result


def project_image_point_to_plane(
    image_x,
    image_y,
    image_to_plane,
    denominator_epsilon=1.0e-6,
    matrix_validated=False,
):
    """把一个像素点投影为车体系物理平面坐标，返回 (right, forward)。"""
    image_x = float(image_x)
    image_y = float(image_y)
    if not _is_finite_number(image_x) or not _is_finite_number(image_y):
        raise ValueError("image point must be finite")
    matrix = (
        image_to_plane
        if matrix_validated
        else _validated_homography(image_to_plane)
    )
    denominator = (
        matrix[2][0] * image_x
        + matrix[2][1] * image_y
        + matrix[2][2]
    )
    epsilon = max(0.0, float(denominator_epsilon))
    if (
        not _is_finite_number(denominator)
        or abs(denominator) <= epsilon
    ):
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
    if (
        not _is_finite_number(plane_right)
        or not _is_finite_number(plane_forward)
    ):
        raise ValueError("IPM projection produced a non-finite point")
    return plane_right, plane_forward


def plane_line_angle_deg(first_plane, second_plane):
    """由两个物理平面点计算 180° 周期的灯线方向角。"""
    delta_right = float(second_plane[0]) - float(first_plane[0])
    delta_forward = float(second_plane[1]) - float(first_plane[1])
    if abs(delta_right) < 1e-12 and abs(delta_forward) < 1e-12:
        raise ValueError("IPM produced coincident light points")
    return normalize_line_angle_deg(
        math.degrees(math.atan2(delta_forward, delta_right))
    )


def is_valid_candidate(candidate, config):
    """检查单个亮斑的数值、面积和宽高比。"""
    required = ("cx", "cy", "pixels", "w", "h")
    for key in required:
        if key not in candidate or not _is_finite_number(candidate[key]):
            return False

    pixels = float(candidate["pixels"])
    width = float(candidate["w"])
    height = float(candidate["h"])
    if pixels < config["blob_pixels_min"]:
        return False
    if width <= 0.0 or height <= 0.0:
        return False
    if width * height < config["blob_area_min"]:
        return False

    aspect = width / height
    return config["blob_aspect_min"] <= aspect <= config["blob_aspect_max"]


def _measure_pair(first, second):
    points = sorted(
        (first, second),
        key=lambda candidate: (float(candidate["cx"]), float(candidate["cy"])),
    )
    first, second = points
    x1 = float(first["cx"])
    y1 = float(first["cy"])
    x2 = float(second["cx"])
    y2 = float(second["cy"])
    dx = x2 - x1
    dy = y2 - y1
    distance = math.sqrt(dx * dx + dy * dy)
    if not _is_finite_number(distance) or distance <= 0.0:
        return None

    return {
        "found": True,
        "x1": int(round(x1)),
        "y1": int(round(y1)),
        "x2": int(round(x2)),
        "y2": int(round(y2)),
        "mid_x": (x1 + x2) * 0.5,
        "mid_y": (y1 + y2) * 0.5,
        "line_angle_deg": normalize_line_angle_deg(
            math.degrees(math.atan2(dy, dx))
        ),
        "distance_px": distance,
        "quality": min(
            999,
            int(float(first["pixels"]) + float(second["pixels"])),
        ),
        "_size_ratio": min(float(first["pixels"]), float(second["pixels"]))
        / max(float(first["pixels"]), float(second["pixels"])),
    }


def _scaled_error(error, scale):
    scale = max(abs(float(scale)), 1e-6)
    return abs(float(error)) / scale


def _pair_passes_history_limits(measurement, previous, config):
    if previous is None:
        return True

    dx = measurement["mid_x"] - previous["mid_x"]
    dy = measurement["mid_y"] - previous["mid_y"]
    midpoint_jump = math.sqrt(dx * dx + dy * dy)
    distance_jump = abs(measurement["distance_px"] - previous["distance_px"])
    angle_jump = abs(
        normalize_line_angle_deg(
            measurement["line_angle_deg"] - previous["line_angle_deg"]
        )
    )
    return (
        midpoint_jump <= config["max_midpoint_jump_px"]
        and distance_jump <= config["max_distance_jump_px"]
        and angle_jump <= config["max_angle_jump_deg"]
    )


def _pair_score(measurement, config, previous=None):
    ref_dx = measurement["mid_x"] - config["ref_cx"]
    ref_dy = measurement["mid_y"] - config["ref_cy"]
    ref_mid_error = math.sqrt(ref_dx * ref_dx + ref_dy * ref_dy)

    score = config["score_ref_mid_weight"] * _scaled_error(
        ref_mid_error,
        config["score_mid_scale_px"],
    )
    score += config["score_ref_distance_weight"] * _scaled_error(
        measurement["distance_px"] - config["ref_distance_px"],
        config["score_distance_scale_px"],
    )
    score += config["score_ref_angle_weight"] * _scaled_error(
        normalize_line_angle_deg(
            measurement["line_angle_deg"] - config["ref_line_angle_deg"]
        ),
        config["score_angle_scale_deg"],
    )
    score += config["score_size_balance_weight"] * (
        1.0 - measurement["_size_ratio"]
    )

    if previous is not None:
        history_dx = measurement["mid_x"] - previous["mid_x"]
        history_dy = measurement["mid_y"] - previous["mid_y"]
        history_mid_error = math.sqrt(
            history_dx * history_dx + history_dy * history_dy
        )
        history_angle_error = normalize_line_angle_deg(
            measurement["line_angle_deg"] - previous["line_angle_deg"]
        )
        score += config["score_history_mid_weight"] * _scaled_error(
            history_mid_error,
            config["score_mid_scale_px"],
        )
        score += config["score_history_distance_weight"] * _scaled_error(
            measurement["distance_px"] - previous["distance_px"],
            config["score_distance_scale_px"],
        )
        score += config["score_history_angle_weight"] * _scaled_error(
            history_angle_error,
            config["score_angle_scale_deg"],
        )

    return score


def select_best_pair(candidates, config, previous=None):
    """从全部候选亮斑中选出最符合参考几何和历史连续性的一对。"""
    valid = [
        candidate
        for candidate in (candidates or [])
        if is_valid_candidate(candidate, config)
    ]
    best = None
    best_score = None

    for first_index in range(len(valid)):
        for second_index in range(first_index + 1, len(valid)):
            measurement = _measure_pair(
                valid[first_index],
                valid[second_index],
            )
            if measurement is None:
                continue
            if not (
                config["pair_distance_min_px"]
                <= measurement["distance_px"]
                <= config["pair_distance_max_px"]
            ):
                continue
            if measurement["_size_ratio"] < config["pair_size_ratio_min"]:
                continue
            reference_angle_error = abs(
                normalize_line_angle_deg(
                    measurement["line_angle_deg"]
                    - config["ref_line_angle_deg"]
                )
            )
            if reference_angle_error > config["pair_angle_tolerance_deg"]:
                continue
            if not _pair_passes_history_limits(measurement, previous, config):
                continue
            score = _pair_score(measurement, config, previous)
            if best_score is None or score < best_score:
                best = measurement
                best_score = score

    if best is not None:
        del best["_size_ratio"]
    return best


def ema(current, sample, alpha):
    """普通指数滑动平均。"""
    alpha = max(0.0, min(1.0, float(alpha)))
    return float(current) + alpha * (float(sample) - float(current))


def ema_line_angle_deg(current, sample, alpha):
    """按 180°周期平滑无方向直线角。"""
    alpha = max(0.0, min(1.0, float(alpha)))
    delta = normalize_line_angle_deg(float(sample) - float(current))
    return normalize_line_angle_deg(float(current) + alpha * delta)


def _filtered_measurement(current, sample, alpha):
    result = dict(sample)
    for key in ("x1", "y1", "x2", "y2"):
        result[key] = ema(current[key], sample[key], alpha)
    result["mid_x"] = ema(current["mid_x"], sample["mid_x"], alpha)
    result["mid_y"] = ema(current["mid_y"], sample["mid_y"], alpha)
    result["distance_px"] = ema(
        current["distance_px"],
        sample["distance_px"],
        alpha,
    )
    result["line_angle_deg"] = ema_line_angle_deg(
        current["line_angle_deg"],
        sample["line_angle_deg"],
        alpha,
    )
    return result


class IRTracker:
    """维护配对历史、获取确认状态和测量滤波。"""

    def __init__(self, config):
        self.config = config
        self.image_to_plane = _validated_homography(
            config["ipm_image_to_plane"]
        )
        self.ipm_denominator_epsilon = float(
            config.get("ipm_denominator_epsilon", 1.0e-6)
        )
        self.reference_plane_angle_deg = float(
            config["ref_plane_angle_deg"]
        )
        self.previous = None
        self.filtered = None
        self.confirm_count = 0
        self.lost_count = 0
        self.confirmed = False
        self.last_output = None

    def _project(self, image_x, image_y):
        return project_image_point_to_plane(
            image_x,
            image_y,
            self.image_to_plane,
            self.ipm_denominator_epsilon,
            matrix_validated=True,
        )

    def _rectified_pair_result(self, measurement):
        first_plane = self._project(
            measurement["x1"], measurement["y1"]
        )
        second_plane = self._project(
            measurement["x2"], measurement["y2"]
        )
        physical_angle = plane_line_angle_deg(first_plane, second_plane)
        result = dict(measurement)
        result["found"] = True
        result["mode"] = "pair"
        # 位置跟随永久使用像素 x 较大的第二个灯点。
        result["target_x_cm"] = second_plane[0]
        result["target_y_cm"] = second_plane[1]
        # 对车端发送相对理想平行方向的真实物理角误差，因此 0° 即无需
        # 视觉转向修正。
        result["theta_deg"] = normalize_line_angle_deg(
            physical_angle - self.reference_plane_angle_deg
        )
        result["plane_line_angle_deg"] = physical_angle
        return result

    def _single_identity(self, candidates):
        valid = [
            candidate
            for candidate in (candidates or [])
            if is_valid_candidate(candidate, self.config)
        ]
        if len(valid) != 1 or self.previous is None:
            return None, None

        candidate = valid[0]
        image_x = float(candidate["cx"])
        if image_x <= float(self.config["single_right_max_x"]):
            return "right", candidate
        if image_x >= float(self.config["single_left_min_x"]):
            return "left", candidate

        # 150~170 px 的中间区仅凭视野边界无法判定，使用上一帧两端点
        # 连续性；超过单帧跳变门限则保持不确定。
        image_y = float(candidate["cy"])
        right_dx = image_x - float(self.previous["x2"])
        right_dy = image_y - float(self.previous["y2"])
        left_dx = image_x - float(self.previous["x1"])
        left_dy = image_y - float(self.previous["y1"])
        right_distance = math.sqrt(
            right_dx * right_dx + right_dy * right_dy
        )
        left_distance = math.sqrt(left_dx * left_dx + left_dy * left_dy)
        maximum = float(self.config["single_history_max_jump_px"])
        if right_distance <= maximum and right_distance < left_distance:
            return "right", candidate
        if left_distance <= maximum and left_distance < right_distance:
            return "left", candidate
        return None, candidate

    @staticmethod
    def _shift_pair_to_right_candidate(measurement, image_x, image_y):
        """用单点位移平移历史灯对，保证恢复双点时历史门限连续。"""
        result = dict(measurement)
        delta_x = float(image_x) - float(measurement["x2"])
        delta_y = float(image_y) - float(measurement["y2"])
        for key in ("x1", "x2", "mid_x"):
            result[key] = float(measurement[key]) + delta_x
        for key in ("y1", "y2", "mid_y"):
            result[key] = float(measurement[key]) + delta_y
        return result

    def _right_single_result(self, candidate):
        image_x = float(candidate["cx"])
        image_y = float(candidate["cy"])
        self.previous = self._shift_pair_to_right_candidate(
            self.previous, image_x, image_y
        )
        self.filtered = self._shift_pair_to_right_candidate(
            self.filtered,
            ema(self.filtered["x2"], image_x, self.config["ema_alpha"]),
            ema(self.filtered["y2"], image_y, self.config["ema_alpha"]),
        )
        target_plane = self._project(
            self.filtered["x2"], self.filtered["y2"]
        )
        result = dict(self.filtered)
        result["found"] = True
        result["mode"] = "single_right"
        result["target_x_cm"] = target_plane[0]
        result["target_y_cm"] = target_plane[1]
        result["theta_deg"] = 0.0
        result["plane_line_angle_deg"] = None
        result["quality"] = min(999, int(float(candidate["pixels"])))
        return result

    def _reset_lost(self):
        self.previous = None
        self.filtered = None
        self.confirm_count = 0
        self.lost_count = 0
        self.confirmed = False
        self.last_output = None

    def update(self, candidates):
        measurement = select_best_pair(
            candidates,
            self.config,
            previous=self.previous,
        )
        if measurement is None:
            identity, candidate = self._single_identity(candidates)
            if self.confirmed and self.filtered is not None:
                if identity == "right":
                    self.lost_count = 0
                    result = self._right_single_result(candidate)
                    self.last_output = dict(result)
                    return result
                if identity == "left":
                    # 已明确只剩左灯，说明位置基准右灯丢失，立即停止使用
                    # 视觉位置，不把左灯冒充右灯。
                    self._reset_lost()
                    return {"found": False}
            self.lost_count += 1
            required = max(1, int(self.config.get("lost_confirm_frames", 1)))
            if (
                self.confirmed
                and self.last_output is not None
                and self.lost_count < required
            ):
                # 短暂漏检时保持最后一帧确认结果，避免主控因单帧噪声急停。
                return dict(self.last_output)
            self._reset_lost()
            return {"found": False}

        if self.filtered is None:
            next_filtered = dict(measurement)
        else:
            next_filtered = _filtered_measurement(
                self.filtered,
                measurement,
                self.config["ema_alpha"],
            )

        next_confirm_count = self.confirm_count + 1
        required = max(1, int(self.config["acquire_confirm_frames"]))
        # 先完成 IPM 与角度计算，再提交跟踪状态。若投影异常，外层把该帧
        # 当作漏检时不会被这里提前清零 lost_count，能够按配置进入 LOST。
        result = None
        if next_confirm_count >= required:
            result = self._rectified_pair_result(next_filtered)

        self.lost_count = 0
        self.previous = dict(measurement)
        self.filtered = next_filtered
        self.confirm_count = next_confirm_count
        if result is None:
            return {"found": False}

        self.confirmed = True
        self.last_output = dict(result)
        return result


def format_measurement_line(measurement):
    """发送物理平面目标点 (x_cm,y_cm,theta_deg)，丢失发送 0。"""
    if not measurement or not measurement.get("found", False):
        return "0\n"

    return "%.3f,%.3f,%.2f\n" % (
        float(measurement["target_x_cm"]),
        float(measurement["target_y_cm"]),
        float(measurement["theta_deg"]),
    )


class ReportScheduler:
    """决定 found/lost 帧何时发送，不负责 UART。"""

    def __init__(self, found_interval_ms=30, lost_interval_ms=200):
        self.found_interval_ms = max(0, int(found_interval_ms))
        self.lost_interval_ms = max(0, int(lost_interval_ms))
        self.last_state = None
        self.last_found_ms = None
        self.last_lost_ms = None

    @staticmethod
    def _elapsed(now_ms, last_ms, ticks_diff):
        if last_ms is None:
            return None
        if ticks_diff is not None:
            return ticks_diff(now_ms, last_ms)
        return now_ms - last_ms

    def should_send(self, found, now_ms, ticks_diff=None):
        found = bool(found)
        state_changed = self.last_state is None or found != self.last_state
        self.last_state = found

        if found:
            elapsed = self._elapsed(now_ms, self.last_found_ms, ticks_diff)
            due = elapsed is None or elapsed >= self.found_interval_ms
            if state_changed or due:
                self.last_found_ms = now_ms
                return True
            return False

        elapsed = self._elapsed(now_ms, self.last_lost_ms, ticks_diff)
        due = elapsed is None or elapsed >= self.lost_interval_ms
        if state_changed or due:
            self.last_lost_ms = now_ms
            return True
        return False

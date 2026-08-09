"""IMU 姿态估计与轮速/IMU 坐标积分。

本模块设计为挂接到 motor.MotorSystem：

    odometry = OdometrySystem()
    motor = MotorSystem(odometry=odometry)
    motor.start()

车体坐标采用 +x 向右、+y 向前、+z 向上。世界航向角为从世界 +X 到车辆
前进方向的逆时针角度。距离单位为厘米，角度单位为弧度。
"""

import math
import time

from motor import ChassisKinematics, clamp


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _finite(value):
    return value == value and -1.0e30 < value < 1.0e30


def _sleep_ms(milliseconds):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(milliseconds)
    else:
        time.sleep(milliseconds / 1000.0)


class OdometryConfig:
    def __init__(self):
        # 旧工程 IMU660RX 在 +/-8 g、+/-2000 dps 量程下使用的数值。
        self.acc_lsb_per_g = 4096.0
        self.gyro_lsb_per_dps = 16.384

        # 当前安装方向下，原始 IMU 坐标轴 -> 车体坐标轴：
        # 车体（右、前、上）= 原始（x、z、-y）。
        self.axis_indices = (0, 2, 1)
        self.axis_signs = (1.0, 1.0, -1.0)

        # 各原始陀螺轴在重映射前分别缩放。保留旧偏航轴修正仅作为待复测的
        # 初始值。
        self.gyro_scale_raw = (1.0, 1.160388477, 1.0)

        self.calibration_wait_ms = 1000
        self.calibration_samples = 500
        self.calibration_period_ms = 2
        self.calibration_attempts = 3
        self.calibration_retry_wait_ms = 250
        self.calibration_accel_min_g = 0.75
        self.calibration_accel_max_g = 1.25
        self.calibration_gyro_std_max_raw = 12.0

        self.mahony_kp = 0.8
        self.mahony_ki = 0.0
        self.accel_full_confidence_error = 0.05
        self.accel_zero_confidence_error = 0.20

        self.static_gyro_threshold_dps = 0.8
        self.static_hold_s = 0.20
        self.bias_adapt_time_constant_s = 1.0
        self.yaw_rate_lpf_time_constant_s = 0.03

        self.initial_heading_rad = 0.0
        # 右移实车测试：里程计显示 143.48 cm，实际横向位移约 180 cm。
        self.lateral_distance_scale = 0.6275
        # 前向 100 cm 里程计积分测试：旧比例下里程计到 100 cm 时，
        # 地面实测约为 54 cm；故 1.263 * 54 / 100 = 0.682。
        self.forward_distance_scale = 0.6138


class MahonyAHRS:
    """六轴 Mahony 滤波器；加速度计校正横滚/俯仰，不校正偏航。"""

    def __init__(self, kp=0.8, ki=0.0):
        self.kp = float(kp)
        self.ki = float(ki)
        self.qw = 1.0
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.ix = 0.0
        self.iy = 0.0
        self.iz = 0.0

    def quaternion(self):
        return self.qw, self.qx, self.qy, self.qz

    def _normalize_quaternion(self):
        norm_squared = (
            self.qw * self.qw
            + self.qx * self.qx
            + self.qy * self.qy
            + self.qz * self.qz
        )
        if not _finite(norm_squared) or norm_squared < 1.0e-12:
            return False
        inverse_norm = 1.0 / math.sqrt(norm_squared)
        self.qw *= inverse_norm
        self.qx *= inverse_norm
        self.qy *= inverse_norm
        self.qz *= inverse_norm
        return True

    def initialize_from_accel(self, ax, ay, az):
        norm_squared = ax * ax + ay * ay + az * az
        if not _finite(norm_squared) or norm_squared < 1.0e-12:
            return False
        inverse_norm = 1.0 / math.sqrt(norm_squared)
        ax *= inverse_norm
        ay *= inverse_norm
        az *= inverse_norm

        if az < -0.999999:
            self.qw = 0.0
            self.qx = 1.0
            self.qy = 0.0
            self.qz = 0.0
        else:
            self.qw = 1.0 + az
            self.qx = ay
            self.qy = -ax
            self.qz = 0.0
            if not self._normalize_quaternion():
                return False
        self.ix = 0.0
        self.iy = 0.0
        self.iz = 0.0
        return True

    def update(self, gx, gy, gz, ax, ay, az, dt, accel_weight=1.0):
        if not _finite(dt) or dt <= 0.0 or dt > 0.1:
            return False

        accel_norm_squared = ax * ax + ay * ay + az * az
        if (
            accel_weight > 0.0
            and _finite(accel_norm_squared)
            and accel_norm_squared > 1.0e-12
        ):
            inverse_accel = 1.0 / math.sqrt(accel_norm_squared)
            ax *= inverse_accel
            ay *= inverse_accel
            az *= inverse_accel

            qw = self.qw
            qx = self.qx
            qy = self.qy
            qz = self.qz
            estimated_up_x = 2.0 * (qx * qz - qw * qy)
            estimated_up_y = 2.0 * (qy * qz + qw * qx)
            estimated_up_z = 1.0 - 2.0 * (qx * qx + qy * qy)

            error_x = ay * estimated_up_z - az * estimated_up_y
            error_y = az * estimated_up_x - ax * estimated_up_z
            error_z = ax * estimated_up_y - ay * estimated_up_x
            weight = clamp(accel_weight, 0.0, 1.0)

            if self.ki > 0.0:
                self.ix += self.ki * weight * error_x * dt
                self.iy += self.ki * weight * error_y * dt
                self.iz += self.ki * weight * error_z * dt
            else:
                self.ix = 0.0
                self.iy = 0.0
                self.iz = 0.0

            gx += self.kp * weight * error_x + self.ix
            gy += self.kp * weight * error_y + self.iy
            gz += self.kp * weight * error_z + self.iz

        half_dt = 0.5 * dt
        qw = self.qw
        qx = self.qx
        qy = self.qy
        qz = self.qz
        self.qw = qw + (-qx * gx - qy * gy - qz * gz) * half_dt
        self.qx = qx + (qw * gx + qy * gz - qz * gy) * half_dt
        self.qy = qy + (qw * gy - qx * gz + qz * gx) * half_dt
        self.qz = qz + (qw * gz + qx * gy - qy * gx) * half_dt
        return self._normalize_quaternion()

    def attitude(self):
        qw, qx, qy, qz = self.quaternion()
        forward_x = 2.0 * (qx * qy - qw * qz)
        forward_y = 1.0 - 2.0 * (qx * qx + qz * qz)
        forward_z = 2.0 * (qy * qz + qw * qx)
        right_z = 2.0 * (qx * qz - qw * qy)
        up_z = 1.0 - 2.0 * (qx * qx + qy * qy)

        roll = math.atan2(-right_z, up_z)
        pitch = math.asin(clamp(forward_z, -1.0, 1.0))
        heading = math.atan2(forward_y, forward_x)
        return roll, pitch, heading


class OdometrySystem:
    """管理 IMU 校准/姿态，并积分编码器位移。"""

    def __init__(self, config=None):
        self.config = config or OdometryConfig()
        self.ahrs = MahonyAHRS(self.config.mahony_kp, self.config.mahony_ki)
        self.imu = None
        self.calibrated = False
        self.calibration_error = None
        self.gyro_bias_raw = [0.0, 0.0, 0.0]
        self.gravity_reference_raw = self.config.acc_lsb_per_g
        self.static_time_s = 0.0

        self.x_cm = 0.0
        self.y_cm = 0.0
        self.heading_rad = self.config.initial_heading_rad
        self.heading_unwrapped_rad = self.config.initial_heading_rad
        self.roll_rad = 0.0
        self.pitch_rad = 0.0
        self.yaw_rate_rad_s = 0.0
        self.body_vx_cm_s = 0.0
        self.body_vy_cm_s = 0.0
        self.wheel_w_rad_s = 0.0
        self.heading_offset_rad = 0.0
        # 主线程不能直接与 IMU ticker 并发重置航向；请求会由 ticker
        # 在自身的单一执行上下文中完成。
        self._pending_heading_reset_rad = None
        self._heading_reset_request_id = 0
        self._heading_reset_completed_id = 0
        self.last_error = None

    def initialize_hardware(self):
        """创建并校准 IMU；返回值供 ticker.capture_list() 使用。"""
        if self.imu is None:
            from seekfree import IMU660RX
            self.imu = IMU660RX()
        if not self.calibrate():
            raise RuntimeError(
                "IMU calibration failed: "
                + (self.calibration_error or "keep the car still and restart")
            )
        return self.imu

    def calibrate(self):
        if self.imu is None:
            raise RuntimeError("IMU hardware has not been initialized")

        self.calibration_error = None

        _sleep_ms(self.config.calibration_wait_ms)
        count = int(self.config.calibration_samples)
        if count <= 0:
            self.calibration_error = "calibration_samples must be positive"
            return False
        attempts = max(1, int(self.config.calibration_attempts))
        inverse_count = 1.0 / float(count)

        for attempt_index in range(attempts):
            if attempt_index > 0:
                _sleep_ms(self.config.calibration_retry_wait_ms)

            accel_sum = [0.0, 0.0, 0.0]
            gyro_sum = [0.0, 0.0, 0.0]
            gyro_square_sum = [0.0, 0.0, 0.0]
            accel_norm_sum = 0.0
            for _ in range(count):
                sample = self.imu.read()
                for axis in range(3):
                    accel_sum[axis] += sample[axis]
                    gyro_sum[axis] += sample[axis + 3]
                    gyro_square_sum[axis] += sample[axis + 3] * sample[axis + 3]
                accel_norm_sum += math.sqrt(
                    sample[0] * sample[0]
                    + sample[1] * sample[1]
                    + sample[2] * sample[2]
                )
                _sleep_ms(self.config.calibration_period_ms)

            mean_accel = [
                accel_sum[0] * inverse_count,
                accel_sum[1] * inverse_count,
                accel_sum[2] * inverse_count,
            ]
            gyro_bias_raw = [
                gyro_sum[0] * inverse_count,
                gyro_sum[1] * inverse_count,
                gyro_sum[2] * inverse_count,
            ]
            gravity_reference_raw = accel_norm_sum * inverse_count

            largest_gyro_std = 0.0
            for axis in range(3):
                variance = (
                    gyro_square_sum[axis] * inverse_count
                    - gyro_bias_raw[axis] * gyro_bias_raw[axis]
                )
                largest_gyro_std = max(
                    largest_gyro_std, math.sqrt(max(variance, 0.0))
                )

            gravity_g = gravity_reference_raw / self.config.acc_lsb_per_g
            if (
                gravity_g < self.config.calibration_accel_min_g
                or gravity_g > self.config.calibration_accel_max_g
                or largest_gyro_std > self.config.calibration_gyro_std_max_raw
            ):
                self.calibration_error = (
                    "keep the car still; attempt={}/{}, gravity_g={:.3f}, "
                    "gyro_std_raw={:.2f}"
                ).format(
                    attempt_index + 1,
                    attempts,
                    gravity_g,
                    largest_gyro_std,
                )
                continue

            body_accel = self._map_raw_vector(mean_accel)
            if not self.ahrs.initialize_from_accel(
                body_accel[0], body_accel[1], body_accel[2]
            ):
                self.calibration_error = (
                    "invalid accelerometer direction; attempt={}/{}"
                ).format(attempt_index + 1, attempts)
                continue

            self.gyro_bias_raw = gyro_bias_raw
            self.gravity_reference_raw = gravity_reference_raw
            self.calibrated = True
            self.calibration_error = None
            self.static_time_s = 0.0
            _, _, raw_heading = self.ahrs.attitude()
            self.heading_offset_rad = normalize_angle(
                self.heading_rad - raw_heading
            )
            self._publish_attitude()
            return True

        self.calibrated = False
        return False

    def update_imu_from_buffer(self, dt, stationary=False):
        if self.imu is None:
            return False
        return self.update_imu(self.imu.get(), dt, stationary)

    def update_imu(self, sample, dt, stationary=False):
        """根据一组原始六轴 IMU 数据更新姿态。"""
        if len(sample) < 6:
            self.last_error = "IMU sample must contain at least six values"
            return False
        dt = clamp(float(dt), 0.0005, 0.05)

        raw_accel = (sample[0], sample[1], sample[2])
        raw_gyro = (
            (sample[3] - self.gyro_bias_raw[0])
            * self.config.gyro_scale_raw[0],
            (sample[4] - self.gyro_bias_raw[1])
            * self.config.gyro_scale_raw[1],
            (sample[5] - self.gyro_bias_raw[2])
            * self.config.gyro_scale_raw[2],
        )
        body_accel = self._map_raw_vector(raw_accel)
        body_gyro_raw = self._map_raw_vector(raw_gyro)
        radians_per_raw = math.pi / (
            self.config.gyro_lsb_per_dps * 180.0
        )
        gx = body_gyro_raw[0] * radians_per_raw
        gy = body_gyro_raw[1] * radians_per_raw
        gz = body_gyro_raw[2] * radians_per_raw

        accel_confidence = self._accel_confidence(
            body_accel[0], body_accel[1], body_accel[2]
        )
        if not self.ahrs.update(
            gx,
            gy,
            gz,
            body_accel[0],
            body_accel[1],
            body_accel[2],
            dt,
            accel_confidence,
        ):
            self.last_error = "Mahony update rejected the sample"
            return False

        self._publish_attitude()
        if self.config.yaw_rate_lpf_time_constant_s <= 0.0:
            alpha = 0.0
        else:
            alpha = math.exp(
                -dt / self.config.yaw_rate_lpf_time_constant_s
            )
        self.yaw_rate_rad_s = (
            alpha * self.yaw_rate_rad_s + (1.0 - alpha) * gz
        )
        self._adapt_bias(sample, dt, stationary, accel_confidence)
        self._apply_pending_heading_reset()
        return True

    def update_wheels(
        self,
        wheel_1,
        wheel_2,
        wheel_3,
        dt,
        robot_radius_cm=9.1,
        rotation_gain=1.0,
    ):
        """将一组滤波后的轮速采样积分到世界坐标。"""
        dt = clamp(float(dt), 0.001, 0.05)
        vx, vy, wheel_w = ChassisKinematics.wheels_to_body(
            float(wheel_1),
            float(wheel_2),
            float(wheel_3),
            robot_radius_cm,
            rotation_gain,
        )
        vx *= self.config.lateral_distance_scale
        vy *= self.config.forward_distance_scale
        self.body_vx_cm_s = vx
        self.body_vy_cm_s = vy
        self.wheel_w_rad_s = wheel_w

        theta = self.heading_rad
        world_vx = vy * math.cos(theta) + vx * math.sin(theta)
        world_vy = vy * math.sin(theta) - vx * math.cos(theta)
        self.x_cm += world_vx * dt
        self.y_cm += world_vy * dt
        return self.x_cm, self.y_cm, self.heading_rad

    def set_pose(self, x_cm=0.0, y_cm=0.0, heading_rad=0.0):
        self.x_cm = float(x_cm)
        self.y_cm = float(y_cm)
        self.reset_heading(heading_rad)

    def reset_position(self, x_cm=0.0, y_cm=0.0):
        self.x_cm = float(x_cm)
        self.y_cm = float(y_cm)

    def reset_heading(self, heading_rad=0.0):
        target_heading = normalize_angle(float(heading_rad))
        _, _, raw_heading = self.ahrs.attitude()
        new_offset = normalize_angle(target_heading - raw_heading)
        
        # 将写入顺序调整为先更新 offset，再更新实际航向
        self.heading_offset_rad = new_offset
        self.heading_rad = target_heading
        self.heading_unwrapped_rad = target_heading
        self.yaw_rate_rad_s = 0.0

    def request_heading_reset(self, heading_rad=0.0):
        """Request a heading reset to be applied safely by the IMU ticker."""
        self._heading_reset_request_id += 1
        self._pending_heading_reset_rad = normalize_angle(float(heading_rad))
        return self._heading_reset_request_id

    def heading_reset_completed(self, request_id):
        return self._heading_reset_completed_id >= int(request_id)

    def get_pose(self):
        return self.x_cm, self.y_cm, self.heading_rad

    def get_attitude(self):
        return self.roll_rad, self.pitch_rad, self.heading_rad

    def get_state(self):
        return {
            "calibrated": self.calibrated,
            "x_cm": self.x_cm,
            "y_cm": self.y_cm,
            "heading_rad": self.heading_rad,
            "heading_unwrapped_rad": self.heading_unwrapped_rad,
            "roll_rad": self.roll_rad,
            "pitch_rad": self.pitch_rad,
            "yaw_rate_rad_s": self.yaw_rate_rad_s,
            "body_vx_cm_s": self.body_vx_cm_s,
            "body_vy_cm_s": self.body_vy_cm_s,
            "wheel_w_rad_s": self.wheel_w_rad_s,
            "gyro_bias_raw": tuple(self.gyro_bias_raw),
            "gravity_reference_raw": self.gravity_reference_raw,
            "error": self.last_error,
        }

    def _map_raw_vector(self, raw_vector):
        indices = self.config.axis_indices
        signs = self.config.axis_signs
        return (
            raw_vector[indices[0]] * signs[0],
            raw_vector[indices[1]] * signs[1],
            raw_vector[indices[2]] * signs[2],
        )

    def _accel_confidence(self, ax, ay, az):
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        reference = self.gravity_reference_raw
        if not _finite(norm) or norm < 1.0e-12 or reference < 1.0e-12:
            return 0.0
        error = abs(norm / reference - 1.0)
        full = self.config.accel_full_confidence_error
        zero = self.config.accel_zero_confidence_error
        if error <= full:
            return 1.0
        if error >= zero or zero <= full:
            return 0.0
        return (zero - error) / (zero - full)

    def _publish_attitude(self):
        roll, pitch, raw_heading = self.ahrs.attitude()
        self.roll_rad = roll
        self.pitch_rad = pitch
        next_heading = normalize_angle(
            raw_heading + self.heading_offset_rad
        )
        self.heading_unwrapped_rad += normalize_angle(
            next_heading - self.heading_rad
        )
        self.heading_rad = next_heading

    def _apply_pending_heading_reset(self):
        """Apply one requested reset within the IMU ticker execution context."""
        target_heading = self._pending_heading_reset_rad
        if target_heading is None:
            return False
        _, _, raw_heading = self.ahrs.attitude()
        self.heading_offset_rad = normalize_angle(
            target_heading - raw_heading
        )
        self.heading_rad = target_heading
        self.heading_unwrapped_rad = target_heading
        self.yaw_rate_rad_s = 0.0
        self._pending_heading_reset_rad = None
        self._heading_reset_completed_id = self._heading_reset_request_id
        return True

    def _adapt_bias(self, sample, dt, stationary, accel_confidence):
        gyro_norm_dps_squared = 0.0
        for axis in range(3):
            corrected = (
                sample[axis + 3] - self.gyro_bias_raw[axis]
            ) * self.config.gyro_scale_raw[axis]
            gyro_dps = corrected / self.config.gyro_lsb_per_dps
            gyro_norm_dps_squared += gyro_dps * gyro_dps
        gyro_norm_dps = math.sqrt(gyro_norm_dps_squared)

        if (
            stationary
            and gyro_norm_dps < self.config.static_gyro_threshold_dps
            and accel_confidence >= 0.95
        ):
            self.static_time_s += dt
        else:
            self.static_time_s = 0.0

        if self.static_time_s < self.config.static_hold_s:
            return
        time_constant = self.config.bias_adapt_time_constant_s
        alpha = 1.0 if time_constant <= 0.0 else 1.0 - math.exp(-dt / time_constant)
        for axis in range(3):
            self.gyro_bias_raw[axis] += (
                sample[axis + 3] - self.gyro_bias_raw[axis]
            ) * alpha

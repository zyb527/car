"""RT1021 主板三轮底盘电机控制。

坐标约定
---------------------
vx：车体向右速度，cm/s
vy：车体向前速度，cm/s
w：逆时针角速度，rad/s

本模块可在电脑上直接导入。只有调用 MotorSystem.start() 才会创建 RT1021
硬件，因此无需开发板也能测试运动学、S 曲线和 PI 控制器。
"""

import math
import time


SQRT3 = 1.7320508075688772
SQRT3_OVER_2 = 0.8660254037844386

def clamp(value, lower, upper):
    if value < lower:
        return lower
    if value > upper:
        return upper
    return value


def _vector_limit(x, y, maximum):
    if maximum <= 0.0:
        return 0.0, 0.0
    magnitude = math.sqrt(x * x + y * y)
    if magnitude > maximum and magnitude > 1.0e-12:
        scale = maximum / magnitude
        return x * scale, y * scale
    return x, y


def _normalize_angle(angle):
    """Normalize a radian angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.monotonic() * 1000.0)


def _ticks_us():
    if hasattr(time, "ticks_us"):
        return time.ticks_us()
    return int(time.monotonic() * 1000000.0)


def _ticks_diff(new_value, old_value):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(new_value, old_value)
    return new_value - old_value


def map_minimum_wheel_speed(
    wheel_speeds,
    deadband_cm_s=2.0,
    minimum_active_cm_s=4.0,
):
    """按峰值统一映射三轮目标，消除低速轮子不动的死区。"""
    deadband_cm_s = float(deadband_cm_s)
    minimum_active_cm_s = float(minimum_active_cm_s)
    if deadband_cm_s < 0.0:
        raise ValueError("deadband_cm_s must be non-negative")
    if minimum_active_cm_s < deadband_cm_s:
        raise ValueError(
            "minimum_active_cm_s must not be smaller than deadband_cm_s"
        )
    wheels = tuple(float(wheel_speeds[index]) for index in range(3))
    peak = max(abs(wheels[0]), abs(wheels[1]), abs(wheels[2]))
    if peak < deadband_cm_s or peak <= 1.0e-12:
        return (0.0, 0.0, 0.0), 0.0
    if peak < minimum_active_cm_s:
        scale = minimum_active_cm_s / peak
        return tuple(value * scale for value in wheels), scale
    return wheels, 1.0


class WheelConfig:
    """单个轮子的硬件与控制器参数。"""

    def __init__(
        self,
        motor_index,
        encoder_a,
        encoder_b,
        motor_invert=False,
        encoder_invert=False,
        pulses_per_meter=6000.0,
        kp=3.1,
        ki=50.0,
        feedforward=5.6,
        ka=0.0,
        stiction_duty=340.0,
        running_offset_duty=0.0,
        stiction_full_speed=5.0,
        acceleration_lpf_time_constant_s=0.030,
        max_target_acceleration_cm_s2=400.0,
    ):
        self.motor_index = motor_index
        self.encoder_a = encoder_a
        self.encoder_b = encoder_b
        self.motor_invert = motor_invert
        self.encoder_invert = encoder_invert
        self.pulses_per_meter = pulses_per_meter
        self.kp = kp
        self.ki = ki
        self.feedforward = feedforward
        # 加速度前馈，单位为 PWM 占空比/(cm/s²)。必须在实车上从 0 开始调。
        self.ka = ka
        self.stiction_duty = stiction_duty
        self.running_offset_duty = running_offset_duty
        self.stiction_full_speed = stiction_full_speed
        # 只滤波目标加速度，不对噪声较大的编码器实测速度求导。
        self.acceleration_lpf_time_constant_s = acceleration_lpf_time_constant_s
        self.max_target_acceleration_cm_s2 = max_target_acceleration_cm_s2


class MotorConfig:
    """底盘全局默认参数。

    初始值保留旧工程的接线和保守增益。
    docs/calibration.md 中标注的参数必须在实车上测量。
    """

    def __init__(self):

        self.wheels = (
            WheelConfig(
                "PWM_D4_DIR_D5",
                "D13",
                "D14",
                pulses_per_meter=21100.0,
                kp=3,
                ki=15.2,
                stiction_duty=800.0,
                running_offset_duty=440.0,
                feedforward=16.87,
                ka=0.72,
            ),
            WheelConfig(
                "PWM_C28_DIR_C29",
                "D15",
                "D16",
                pulses_per_meter=20950.0,
                kp=2.7,
                ki=15,
                stiction_duty=1050.0,
                running_offset_duty=400.0,
                feedforward=17.58,
                ka=0.63,
            ),
            WheelConfig(
                "PWM_C30_DIR_C31",
                "C2",
                "C3",
                pulses_per_meter=21000.0,
                kp=3.5,
                ki=18,
                stiction_duty=1050.0,
                running_offset_duty=400.0,
                feedforward=19,
                ka=0.84,
            ),
        )
        self.pwm_frequency_hz = 13000
        self.max_duty = 9000.0

        self.robot_radius_cm = 9.1
        # 1.0 是物理模型值；仅在完成旋转测试后才可增大。
        # 录像绝对角速度标定：2.0 rad/s 命令在旧系数 1.03 下，
        # 15.707963 s 仅转过 1410°（目标 1800°）。按 1800/1410
        # 修正车体角速度到轮速的映射，使公开的 w 命令更接近真实 rad/s。
        self.rotation_gain = 1.315
        self.max_wheel_speed_cm_s = 700.0

        # 兼容旧调用方的公共比例；新的控制路径按横向/前向分别取值。
        self.body_command_speed_scale = 50.0 / 57.0
        # Public move()/command() vx, vy use real body cm/s.  定速两秒实测
        # 前进 58 cm，故 0.877 * 100 / 58 = 1.512；当前统一用于两个线轴。
        # 角速度不缩放。
        self.body_command_lateral_speed_scale = (
            self.body_command_speed_scale * 100.0 / 58.0
        )
        self.body_command_forward_speed_scale = (
            self.body_command_speed_scale * 100.0 / 58.0
        )

        self.base_period_ms = 2
        self.control_period_ms = 10
        self.watchdog_timeout_ms = 150
        self.speed_lpf_time_constant_s = 0.025
        self.zero_target_cm_s = 0.05
        self.stopped_speed_cm_s = 1.0
        # 低速三轮统一映射：峰值 < 2 cm/s 视为停止；2--4 cm/s
        # 等比抬升至峰值 4 cm/s，以保持全向运动方向不变。
        self.wheel_speed_deadband_cm_s = 2.0
        self.minimum_active_wheel_speed_cm_s = 4.0

        self.max_xy_speed_cm_s = 700.0
        self.max_w_rad_s = 3.4
        self.xy_accel_up_cm_s2 = 1200.0
        # 黄线回库等正常零速度指令使用 S 曲线减速；提高此值可缩短滑行距离。
        self.xy_accel_down_cm_s2 = 600.0
        self.xy_jerk_cm_s3 = 15000.0
        self.w_accel_up_rad_s2 = 22.0
        self.w_accel_down_rad_s2 = 18.0
        self.w_jerk_rad_s3 = 360.0

        self.heading_hold_enabled = True
        self.heading_hold_kp = 2.5
        self.heading_hold_kd = 0.12
        self.heading_hold_max_w_rad_s = 0.5
        self.heading_hold_w_deadband_rad_s = 0.01
        self.heading_hold_min_xy_speed_cm_s = 0.5


class ChassisKinematics:
    """已验证的 car141929 三轮车体到电机输出映射。"""

    @staticmethod
    def body_to_wheels(vx, vy, w, robot_radius_cm=9.1, rotation_gain=1.0):
        rotation_speed = w * robot_radius_cm * rotation_gain
        wheel_1 = -0.5 * vx - SQRT3_OVER_2 * vy + rotation_speed
        wheel_2 = -0.5 * vx + SQRT3_OVER_2 * vy + rotation_speed
        wheel_3 = vx + rotation_speed
        return wheel_1, wheel_2, wheel_3

    @staticmethod
    def wheels_to_body(wheel_1, wheel_2, wheel_3, robot_radius_cm=9.1, rotation_gain=1.0):
        rotation_speed = (wheel_1 + wheel_2 + wheel_3) / 3.0
        vx = wheel_3 - rotation_speed
        vy = (wheel_2 - wheel_1) / SQRT3
        denominator = robot_radius_cm * rotation_gain
        w = rotation_speed / denominator if abs(denominator) > 1.0e-12 else 0.0
        return vx, vy, w

    @staticmethod
    def limit_wheels(wheel_speeds, maximum):
        largest = max(abs(wheel_speeds[0]), abs(wheel_speeds[1]), abs(wheel_speeds[2]))
        if maximum <= 0.0:
            return (0.0, 0.0, 0.0), 0.0
        if largest <= maximum * (1.0 + 1.0e-12) or largest <= 1.0e-12:
            return wheel_speeds, 1.0
        scale = maximum / largest
        return (
            wheel_speeds[0] * scale,
            wheel_speeds[1] * scale,
            wheel_speeds[2] * scale,
        ), scale


class SCurveLimiter:
    """在线限加加速度的车体指令生成器。"""

    def __init__(self, config=None):
        config = config or MotorConfig()
        self.max_xy_speed = config.max_xy_speed_cm_s
        self.max_w = config.max_w_rad_s
        self.xy_accel_up = config.xy_accel_up_cm_s2
        self.xy_accel_down = config.xy_accel_down_cm_s2
        self.xy_jerk = config.xy_jerk_cm_s3
        self.w_accel_up = config.w_accel_up_rad_s2
        self.w_accel_down = config.w_accel_down_rad_s2
        self.w_jerk = config.w_jerk_rad_s3
        self.reset()

    def reset(self, vx=0.0, vy=0.0, w=0.0):
        self.vx = float(vx)
        self.vy = float(vy)
        self.w = float(w)
        self.ax = 0.0
        self.ay = 0.0
        self.aw = 0.0

    def step(self, target_vx, target_vy, target_w, dt):
        # 防止回调延迟造成一次异常大的状态跳变。
        dt = clamp(float(dt), 0.001, 0.05)
        target_vx, target_vy = _vector_limit(
            float(target_vx), float(target_vy), self.max_xy_speed
        )
        target_w = clamp(float(target_w), -self.max_w, self.max_w)

        self._step_xy(target_vx, target_vy, dt)
        self.w, self.aw = self._step_scalar(
            self.w,
            self.aw,
            target_w,
            self.w_accel_up,
            self.w_accel_down,
            self.w_jerk,
            dt,
        )
        return self.vx, self.vy, self.w

    def scale_state(self, scale):
        """发生轮空间饱和时，同步限幅器状态。"""
        scale = clamp(scale, 0.0, 1.0)
        self.vx *= scale
        self.vy *= scale
        self.w *= scale
        self.ax *= scale
        self.ay *= scale
        self.aw *= scale

    def is_settled(self, velocity_tolerance=0.1, accel_tolerance=1.0):
        return (
            abs(self.vx) <= velocity_tolerance
            and abs(self.vy) <= velocity_tolerance
            and abs(self.w) <= velocity_tolerance
            and abs(self.ax) <= accel_tolerance
            and abs(self.ay) <= accel_tolerance
            and abs(self.aw) <= accel_tolerance
        )

    def _step_xy(self, target_vx, target_vy, dt):
        error_x = target_vx - self.vx
        error_y = target_vy - self.vy
        error_magnitude = math.sqrt(error_x * error_x + error_y * error_y)
        current_speed = math.sqrt(self.vx * self.vx + self.vy * self.vy)
        target_speed = math.sqrt(target_vx * target_vx + target_vy * target_vy)
        braking = target_speed < current_speed
        if current_speed > 1.0e-9 and error_x * self.vx + error_y * self.vy < 0.0:
            braking = True
        accel_limit = self.xy_accel_down if braking else self.xy_accel_up

        accel_magnitude = math.sqrt(self.ax * self.ax + self.ay * self.ay)
        # 加速度回落至零时速度仍会继续变化。
        # 需提前开始卸载，避免旧实现到达目标速度时瞬间将加速度清零。
        stopping_delta_v = (
            accel_magnitude * accel_magnitude / (2.0 * self.xy_jerk)
            if self.xy_jerk > 1.0e-12
            else 0.0
        )
        accel_points_to_target = self.ax * error_x + self.ay * error_y > 0.0
        if error_magnitude <= 1.0e-6:
            desired_ax = 0.0
            desired_ay = 0.0
        elif accel_points_to_target and error_magnitude <= stopping_delta_v:
            desired_ax = 0.0
            desired_ay = 0.0
        else:
            desired_ax = error_x * accel_limit / error_magnitude
            desired_ay = error_y * accel_limit / error_magnitude

        delta_ax = desired_ax - self.ax
        delta_ay = desired_ay - self.ay
        delta_ax, delta_ay = _vector_limit(delta_ax, delta_ay, self.xy_jerk * dt)
        self.ax += delta_ax
        self.ay += delta_ay
        self.ax, self.ay = _vector_limit(self.ax, self.ay, accel_limit)

        self.vx += self.ax * dt
        self.vy += self.ay * dt

        new_error_x = target_vx - self.vx
        new_error_y = target_vy - self.vy
        new_error_magnitude = math.sqrt(
            new_error_x * new_error_x + new_error_y * new_error_y
        )
        # S 曲线的离散积分可能在一个控制周期内跨过目标。若允许它跨过，
        # 下一周期会产生反向加速度，进而让停车时的速度指令在零点两侧振荡。
        # 限速器的职责是生成不越过目标的轨迹，因此在跨越时直接收敛到目标。
        crossed_target = (
            error_magnitude > 1.0e-6
            and error_x * new_error_x + error_y * new_error_y <= 0.0
        )
        velocity_snap_tolerance = max(1.0e-4, self.xy_jerk * dt * dt)
        if crossed_target or new_error_magnitude <= velocity_snap_tolerance:
            self.vx = target_vx
            self.vy = target_vy
            self.ax = 0.0
            self.ay = 0.0
        self.vx, self.vy = _vector_limit(self.vx, self.vy, self.max_xy_speed)

    @staticmethod
    def _step_scalar(current, accel, target, accel_up, accel_down, jerk, dt):
        error = target - current
        braking = abs(target) < abs(current)
        if abs(current) > 1.0e-9 and error * current < 0.0:
            braking = True
        accel_limit = accel_down if braking else accel_up
        stopping_delta = accel * accel / (2.0 * jerk) if jerk > 1.0e-12 else 0.0
        if abs(error) <= 1.0e-7:
            desired_accel = 0.0
        elif accel * error > 0.0 and abs(error) <= stopping_delta:
            desired_accel = 0.0
        else:
            desired_accel = accel_limit if error > 0.0 else -accel_limit

        max_accel_change = jerk * dt
        accel += clamp(desired_accel - accel, -max_accel_change, max_accel_change)
        accel = clamp(accel, -accel_limit, accel_limit)

        current += accel * dt
        crossed_target = error * (target - current) <= 0.0 and abs(error) > 1.0e-7
        velocity_snap_tolerance = max(1.0e-5, jerk * dt * dt)
        if crossed_target or abs(target - current) <= velocity_snap_tolerance:
            current = target
            accel = 0.0
        return current, accel


class WheelPIController:
    """考虑时间间隔的 PI 加前馈轮速控制器。"""

    def __init__(self, wheel_config, max_duty):
        self.config = wheel_config
        self.max_duty = float(max_duty)
        self.integral = 0.0
        self.stiction_output = 0.0
        self.breakaway_active = False
        self.last_output = 0.0
        self.last_target = 0.0
        self.target_acceleration = 0.0
        self.acceleration_feedforward = 0.0

    def reset(self):
        self.integral = 0.0
        self.stiction_output = 0.0
        self.breakaway_active = False
        self.last_output = 0.0
        self.last_target = 0.0
        self.target_acceleration = 0.0
        self.acceleration_feedforward = 0.0

    def update(self, target, measured, dt, zero_target=0.05, stopped_speed=1.0):
        target = float(target)
        measured = float(measured)
        dt = clamp(float(dt), 0.001, 0.05)

        if abs(target) <= zero_target and abs(measured) <= stopped_speed:
            self.reset()
            return 0

        # 使用目标轮速的变化率估算目标加速度。阶跃指令会先限幅，再经过一阶
        # 低通滤波，避免 10 ms 内的目标突变产生过大的 PWM 冲击。
        raw_target_acceleration = (target - self.last_target) / dt
        acceleration_limit = max(
            0.0,
            float(self.config.max_target_acceleration_cm_s2),
        )
        if acceleration_limit > 0.0:
            raw_target_acceleration = clamp(
                raw_target_acceleration,
                -acceleration_limit,
                acceleration_limit,
            )
        else:
            raw_target_acceleration = 0.0

        acceleration_tau = max(
            0.0,
            float(self.config.acceleration_lpf_time_constant_s),
        )
        if acceleration_tau > 0.0:
            acceleration_alpha = dt / (acceleration_tau + dt)
        else:
            acceleration_alpha = 1.0
        self.target_acceleration += acceleration_alpha * (
            raw_target_acceleration - self.target_acceleration
        )
        self.last_target = target
        self.acceleration_feedforward = (
            self.config.ka * self.target_acceleration
        )

        error = target - measured
        candidate_integral = clamp(
            self.integral + self.config.ki * error * dt,
            -self.max_duty,
            self.max_duty,
        )

        feedforward = self.config.feedforward * target
        desired_stiction = 0.0
        if abs(target) > zero_target and self.config.stiction_duty > 0.0:
            # 仅在轮速低于目标时需要起步占空比。若只根据目标速度施加，在
            # 正弦减速阶段会有害：轮子可能已经远快于目标，旧代码却仍继续推它。
            #
            # `directed_measured` 仅在轮子沿目标方向运动时为正。仍沿旧方向
            # 运动的轮子会得到完整起步辅助；已经超速的轮子则不再获得补偿，
            # 使 PI/前馈能够直接制动。
            target_magnitude = abs(target)
            direction = 1.0 if target > 0.0 else -1.0
            directed_measured = direction * measured
            ramp_speed = max(self.config.stiction_full_speed, zero_target)
            ramp = min(target_magnitude / ramp_speed, 1.0)
            exit_speed = min(
                ramp_speed,
                max(stopped_speed * 2.0, target_magnitude),
            )

            if abs(measured) <= stopped_speed:
                self.breakaway_active = True
            elif directed_measured >= exit_speed:
                self.breakaway_active = False

            compensation_duty = self.config.running_offset_duty
            if self.breakaway_active:
                compensation_duty = self.config.stiction_duty
            desired_stiction = direction * compensation_duty * ramp

        # 编码器的 10 ms 轮速采样在相邻周期可能相差数 cm/s。若直接使用
        # desired_stiction，补偿会在追上目标附近反复大幅开关并造成车体震动。
        # 静止起步时仍立即给满补偿；车辆已运动时，用约 60 ms 平滑撤出或
        # 调整补偿，既保留起步能力，也避免高频抖振。
        if abs(measured) <= stopped_speed:
            self.stiction_output = desired_stiction
        else:
            stiction_alpha = dt / (0.060 + dt)
            self.stiction_output += stiction_alpha * (
                desired_stiction - self.stiction_output
            )
        stiction = self.stiction_output

        provisional = (
            self.config.kp * error
            + candidate_integral
            + feedforward
            + self.acceleration_feedforward
            + stiction
        )
        saturated = clamp(provisional, -self.max_duty, self.max_duty)

        # 仅当积分会将已饱和的输出继续推向饱和时拒绝积分；反向误差仍可
        # 释放积分项。
        pushes_farther = (
            provisional > self.max_duty and error > 0.0
        ) or (
            provisional < -self.max_duty and error < 0.0
        )
        if not pushes_farther:
            self.integral = candidate_integral
        else:
            provisional = (
                self.config.kp * error
                + self.integral
                + feedforward
                + self.acceleration_feedforward
                + stiction
            )
            saturated = clamp(provisional, -self.max_duty, self.max_duty)

        self.last_output = saturated
        return int(round(saturated))


class MotorSystem:
    """管理电机/编码器硬件并运行完整底盘速度环。"""

    def __init__(self, config=None, odometry=None, ticker_id=1):
        self.config = config or MotorConfig()
        self.odometry = odometry
        self.ticker_id = ticker_id

        self.controllers = [
            WheelPIController(wheel, self.config.max_duty)
            for wheel in self.config.wheels
        ]
        self.limiter = SCurveLimiter(self.config)

        self._motors = None
        self._encoders = None
        self._ticker = None
        self._running = False
        self._motion_active = False
        self._soft_stopping = False
        self._use_s_curve = True
        self._open_loop_calibration = False
        self._open_loop_duty = (0, 0, 0)
        self._target_body = (0.0, 0.0, 0.0)
        self._limited_body = (0.0, 0.0, 0.0)
        self._target_wheels = (0.0, 0.0, 0.0)
        self._heading_hold_target_rad = None
        self._heading_hold_active = False
        self._heading_hold_error_rad = 0.0
        self._heading_hold_w_rad_s = 0.0
        self._wheel_speeds = [0.0, 0.0, 0.0]
        self._encoder_last_counts = [0, 0, 0]
        self._encoder_total_counts = [0, 0, 0]
        self._last_duty = (0, 0, 0)
        self._last_command_ms = _ticks_ms()
        self._last_tick_us = None
        self._control_elapsed_s = 0.0
        self._base_tick_count = 0
        self.last_error = None

    def start(self):
        """创建硬件、校准可选 IMU，然后启动定时器。"""
        if self._running:
            return

        from seekfree import MOTOR_CONTROLLER
        from smartcar import encoder, ticker

        control_div = max(
            1,
            int(round(
                float(self.config.control_period_ms)
                / float(self.config.base_period_ms)
            )),
        )

        motors = []
        encoders = []
        for wheel in self.config.wheels:
            motor_index = getattr(MOTOR_CONTROLLER, wheel.motor_index)
            motor = MOTOR_CONTROLLER(
                motor_index,
                self.config.pwm_frequency_hz,
                duty=0,
                invert=wheel.motor_invert,
            )
            motor.duty(0)
            motors.append(motor)
            encoders.append(
                encoder(
                    wheel.encoder_a,
                    wheel.encoder_b,
                    capture_div=control_div,
                )
            )

        self._motors = motors
        self._encoders = encoders

        capture_devices = list(encoders)
        if self.odometry is not None:
            imu_device = self.odometry.initialize_hardware()
            capture_devices.append(imu_device)

        self._ticker = ticker(self.ticker_id)
        self._ticker.capture_list(*capture_devices)
        self._ticker.callback(self._pit_handler)
        self._last_tick_us = _ticks_us()
        self._running = True
        self._ticker.start(self.config.base_period_ms)

    def move(self, vx, vy, w):
        """设置车体目标速度；需要持续运动时应重复调用。"""
        self._open_loop_calibration = False
        corrected_w = self._heading_hold_w(vx, vy, w)
        vx, vy = self._scale_linear_command(vx, vy)
        target = self._make_feasible_body_target(vx, vy, corrected_w)
        if not self._use_s_curve:
            # 从实际发送给轮速控制器的速度继续，而非从过期的限幅器状态继续。
            self.limiter.reset(
                self._limited_body[0],
                self._limited_body[1],
                self._limited_body[2],
            )
        self._use_s_curve = True
        self._activate_target(target)

    def command(self, vx, vy, w):
        """设置立即生效的车体目标速度，仅绕过 S 曲线。

        轮速限幅、PI 控制和指令看门狗仍然有效。正常行驶请优先使用 move()；
        本接口仅保留给明确需要立即改变速度的控制模式。
        """
        self._open_loop_calibration = False
        corrected_w = self._heading_hold_w(vx, vy, w)
        vx, vy = self._scale_linear_command(vx, vy)
        self._use_s_curve = False
        self._activate_target(
            self._make_feasible_body_target(vx, vy, corrected_w)
        )

    def apply_motion_step(self, result):
        """执行上层 MotionStep，并统一处理硬停和即时交接命令。"""
        if result.failed or result.debug.get("hard_stop", False):
            self.hard_stop()
        elif result.debug.get("immediate_command", False):
            self.command(*result.command)
        else:
            self.move(*result.command)
        return result.command

    def _scale_linear_command(self, vx, vy):
        """Convert public physical vx/vy requests to wheel-loop targets."""
        legacy_scale = float(self.config.body_command_speed_scale)
        lateral_scale = float(
            getattr(
                self.config,
                "body_command_lateral_speed_scale",
                legacy_scale,
            )
        )
        forward_scale = float(
            getattr(
                self.config,
                "body_command_forward_speed_scale",
                legacy_scale,
            )
        )
        if lateral_scale < 0.0 or forward_scale < 0.0:
            raise ValueError("body command speed scales must be non-negative")
        return float(vx) * lateral_scale, float(vy) * forward_scale

    def calibration_duty(self, duty_1, duty_2, duty_3):
        """仅用于特性测试的直接 PWM 输出。

        此接口绕过 S 曲线和轮速 PI，但仍保留输出限幅和普通指令看门狗。调用方
        必须以快于 watchdog_timeout_ms 的频率刷新。严禁用于比赛运动代码。
        """
        self._open_loop_duty = (
            int(clamp(duty_1, -self.config.max_duty, self.config.max_duty)),
            int(clamp(duty_2, -self.config.max_duty, self.config.max_duty)),
            int(clamp(duty_3, -self.config.max_duty, self.config.max_duty)),
        )
        self._target_body = (0.0, 0.0, 0.0)
        self._target_wheels = (0.0, 0.0, 0.0)
        self._reset_heading_hold()
        self._last_command_ms = _ticks_ms()
        self._soft_stopping = False
        self._open_loop_calibration = True
        self._motion_active = True

    def _make_feasible_body_target(self, vx, vy, w):
        vx, vy = _vector_limit(
            float(vx), float(vy), self.config.max_xy_speed_cm_s
        )
        w = clamp(float(w), -self.config.max_w_rad_s, self.config.max_w_rad_s)

        # 指令进入限幅器前，先将其缩放为可实现的车体指令。
        requested_wheels = ChassisKinematics.body_to_wheels(
            vx,
            vy,
            w,
            self.config.robot_radius_cm,
            self.config.rotation_gain,
        )
        _, scale = ChassisKinematics.limit_wheels(
            requested_wheels, self.config.max_wheel_speed_cm_s
        )
        return vx * scale, vy * scale, w * scale

    def _activate_target(self, target):
        self._target_body = target
        self._last_command_ms = _ticks_ms()
        self._soft_stopping = False
        self._motion_active = True

    def _reset_heading_hold(self):
        self._heading_hold_target_rad = None
        self._heading_hold_active = False
        self._heading_hold_error_rad = 0.0
        self._heading_hold_w_rad_s = 0.0

    def _heading_hold_w(self, vx, vy, requested_w):
        """Return the IMU-corrected w for a no-rotation translation command."""
        requested_w = float(requested_w)
        xy_speed = math.sqrt(float(vx) * float(vx) + float(vy) * float(vy))
        if (
            self.odometry is None
            or not self.config.heading_hold_enabled
            or abs(requested_w) > self.config.heading_hold_w_deadband_rad_s
        ):
            self._reset_heading_hold()
            return requested_w

        if xy_speed < self.config.heading_hold_min_xy_speed_cm_s:
            self._heading_hold_active = False
            self._heading_hold_error_rad = 0.0
            self._heading_hold_w_rad_s = 0.0
            return requested_w

        state = self.odometry.get_state()
        if not state.get("calibrated", False):
            self._reset_heading_hold()
            return requested_w

        heading = state.get("heading_rad")
        if heading is None:
            self._reset_heading_hold()
            return requested_w

        if self._heading_hold_target_rad is None:
            self._heading_hold_target_rad = heading

        error = _normalize_angle(heading - self._heading_hold_target_rad)
        correction = -(
            self.config.heading_hold_kp * error
            + self.config.heading_hold_kd * state["yaw_rate_rad_s"]
        )
        correction = clamp(
            correction,
            -self.config.heading_hold_max_w_rad_s,
            self.config.heading_hold_max_w_rad_s,
        )
        self._heading_hold_active = True
        self._heading_hold_error_rad = error
        self._heading_hold_w_rad_s = correction
        return correction

    def soft_stop(self):
        """限加加速度停车；不会被指令看门狗中断。"""
        if self._open_loop_calibration:
            # 原始 PWM 模式下没有可靠的车体目标速度。
            self.hard_stop()
            return
        if not self._use_s_curve:
            self.limiter.reset(
                self._limited_body[0],
                self._limited_body[1],
                self._limited_body[2],
            )
        self._use_s_curve = True
        self._target_body = (0.0, 0.0, 0.0)
        self._soft_stopping = True
        self._motion_active = True

    def hard_stop(self):
        """立即清零 PWM 及所有动态控制器状态。"""
        self._target_body = (0.0, 0.0, 0.0)
        self._limited_body = (0.0, 0.0, 0.0)
        self._target_wheels = (0.0, 0.0, 0.0)
        self._motion_active = False
        self._soft_stopping = False
        self._use_s_curve = True
        self._open_loop_calibration = False
        self._open_loop_duty = (0, 0, 0)
        self._reset_heading_hold()
        self.limiter.reset()
        for controller in self.controllers:
            controller.reset()
        self._set_duty(0, 0, 0)

    def stop(self):
        """停止定时器，并使三个 PWM 输出均为零。"""
        self.hard_stop()
        if self._ticker is not None:
            self._ticker.stop()
        self._running = False

    def get_wheel_speeds(self):
        return tuple(self._wheel_speeds)

    def get_encoder_counts(self):
        """返回（最新 10 ms 采样，累计采样）。"""
        return tuple(self._encoder_last_counts), tuple(self._encoder_total_counts)

    def reset_encoder_totals(self):
        self._encoder_total_counts = [0, 0, 0]

    def get_limited_command(self):
        """返回轮速控制内部使用的、经 S 曲线处理的车体指令。"""
        return self._limited_body

    def get_limited_physical_command(self):
        """返回适合上层/无线转发的真实单位车体速度。"""
        vx, vy, w = self._limited_body
        legacy_scale = float(self.config.body_command_speed_scale)
        lateral_scale = float(
            getattr(
                self.config,
                "body_command_lateral_speed_scale",
                legacy_scale,
            )
        )
        forward_scale = float(
            getattr(
                self.config,
                "body_command_forward_speed_scale",
                legacy_scale,
            )
        )
        if lateral_scale <= 0.0 or forward_scale <= 0.0:
            raise ValueError("body command speed scales must be positive")
        return vx / lateral_scale, vy / forward_scale, w

    def get_state(self):
        return {
            "running": self._running,
            "motion_active": self._motion_active,
            "soft_stopping": self._soft_stopping,
            "s_curve_enabled": self._use_s_curve,
            "open_loop_calibration": self._open_loop_calibration,
            "target_body": self._target_body,
            "limited_body": self._limited_body,
            "target_wheels": self._target_wheels,
            "heading_hold_active": self._heading_hold_active,
            "heading_hold_target_rad": self._heading_hold_target_rad,
            "heading_hold_error_rad": self._heading_hold_error_rad,
            "heading_hold_w_rad_s": self._heading_hold_w_rad_s,
            "target_wheel_accelerations": tuple(
                controller.target_acceleration
                for controller in self.controllers
            ),
            "acceleration_feedforward": tuple(
                controller.acceleration_feedforward
                for controller in self.controllers
            ),
            "wheel_speeds": tuple(self._wheel_speeds),
            "encoder_counts": tuple(self._encoder_last_counts),
            "encoder_totals": tuple(self._encoder_total_counts),
            "duty": self._last_duty,
            "error": self.last_error,
        }

    def _pit_handler(self, ticker_object):
        try:
            now_us = _ticks_us()
            elapsed_us = _ticks_diff(now_us, self._last_tick_us)
            self._last_tick_us = now_us
            nominal_us = self.config.base_period_ms * 1000
            if elapsed_us < 100 or elapsed_us > 100000:
                elapsed_us = nominal_us
            base_dt = elapsed_us / 1000000.0

            if self.odometry is not None:
                stationary = (
                    not self._motion_active
                    and max(
                        abs(self._wheel_speeds[0]),
                        abs(self._wheel_speeds[1]),
                        abs(self._wheel_speeds[2]),
                    ) < self.config.stopped_speed_cm_s
                )
                self.odometry.update_imu_from_buffer(base_dt, stationary)

            self._control_elapsed_s += base_dt
            self._base_tick_count += 1
            control_div = max(
                1,
                int(round(
                    float(self.config.control_period_ms)
                    / float(self.config.base_period_ms)
                )),
            )
            if self._base_tick_count < control_div:
                return

            control_dt = self._control_elapsed_s
            self._base_tick_count = 0
            self._control_elapsed_s = 0.0
            self._update_measured_speeds(control_dt)

            if self.odometry is not None:
                self.odometry.update_wheels(
                    self._wheel_speeds[0],
                    self._wheel_speeds[1],
                    self._wheel_speeds[2],
                    control_dt,
                    self.config.robot_radius_cm,
                    self.config.rotation_gain,
                )

            if (
                self._motion_active
                and not self._soft_stopping
                and _ticks_diff(_ticks_ms(), self._last_command_ms)
                    >= self.config.watchdog_timeout_ms
            ):
                self.hard_stop()
                return

            if not self._motion_active:
                return
            if self._open_loop_calibration:
                self._set_duty(
                    self._open_loop_duty[0],
                    self._open_loop_duty[1],
                    self._open_loop_duty[2],
                )
                return
            self._control_step(control_dt, self._wheel_speeds)
        except Exception as error:
            self.last_error = repr(error)
            self.hard_stop()

    def _update_measured_speeds(self, dt):
        dt = clamp(float(dt), 0.001, 0.05)
        if self.config.speed_lpf_time_constant_s <= 0.0:
            alpha = 0.0
        else:
            alpha = math.exp(-dt / self.config.speed_lpf_time_constant_s)

        for index in range(3):
            count = self._encoders[index].get()
            # 部分 RT1021 智能车固件的 encoder() 接受 ``invert`` 参数，但
            # 不应依赖它。极性修正保留在 Python 中，以保证各块板上的控制器
            # 反馈行为一致。
            if self.config.wheels[index].encoder_invert:
                count = -count
            self._encoder_last_counts[index] = count
            self._encoder_total_counts[index] += count
            pulses_per_meter = self.config.wheels[index].pulses_per_meter
            raw_speed = 0.0
            if pulses_per_meter > 0.0:
                raw_speed = count * 100.0 / (pulses_per_meter * dt)
            self._wheel_speeds[index] = (
                alpha * self._wheel_speeds[index]
                + (1.0 - alpha) * raw_speed
            )

    def _control_step(self, dt, measured_speeds):
        if self._use_s_curve:
            vx, vy, w = self.limiter.step(
                self._target_body[0],
                self._target_body[1],
                self._target_body[2],
                dt,
            )
        else:
            vx, vy, w = self._target_body
            self.limiter.reset(vx, vy, w)
        wheels = ChassisKinematics.body_to_wheels(
            vx,
            vy,
            w,
            self.config.robot_radius_cm,
            self.config.rotation_gain,
        )
        wheels, maximum_scale = ChassisKinematics.limit_wheels(
            wheels, self.config.max_wheel_speed_cm_s
        )
        if maximum_scale < 1.0:
            self.limiter.scale_state(maximum_scale)
            vx *= maximum_scale
            vy *= maximum_scale
            w *= maximum_scale

        wheels, minimum_scale = map_minimum_wheel_speed(
            wheels,
            self.config.wheel_speed_deadband_cm_s,
            self.config.minimum_active_wheel_speed_cm_s,
        )
        if minimum_scale == 0.0:
            vx, vy, w = 0.0, 0.0, 0.0
        else:
            vx *= minimum_scale
            vy *= minimum_scale
            w *= minimum_scale

        self._limited_body = (vx, vy, w)
        self._target_wheels = wheels
        duties = [0, 0, 0]
        for index in range(3):
            duties[index] = self.controllers[index].update(
                wheels[index],
                measured_speeds[index],
                dt,
                self.config.zero_target_cm_s,
                self.config.stopped_speed_cm_s,
            )
        self._set_duty(duties[0], duties[1], duties[2])

        if self._soft_stopping and self.limiter.is_settled():
            self.hard_stop()

    def _set_duty(self, duty_1, duty_2, duty_3):
        duty_1 = int(clamp(duty_1, -self.config.max_duty, self.config.max_duty))
        duty_2 = int(clamp(duty_2, -self.config.max_duty, self.config.max_duty))
        duty_3 = int(clamp(duty_3, -self.config.max_duty, self.config.max_duty))
        self._last_duty = (duty_1, duty_2, duty_3)
        if self._motors is not None:
            self._motors[0].duty(duty_1)
            self._motors[1].duty(duty_2)
            self._motors[2].duty(duty_3)

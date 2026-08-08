"""主车主程序世界坐标落点标定。

在 TARGET_NAME 中选择一个主程序实际使用的坐标点。车辆按 main_config
定义的初始位姿摆放；倒计时后自动转向并驶向目标。停车后测量车体参考点
相对场地原点的实际 X/Y，并与串口输出一起记录。

本脚本直接复用当前主程序的里程计、电机、转向和坐标导航代码，所以坐标
定义、比例、速度限制及到点容差都与当前主程序一致。

坐标约定：+X 向右，+Y 向前/场地上方，单位 cm；航向 0 度朝 +X，
逆时针为正。运动过程中如需中止，请直接关闭电源。
"""

import math
import time

import main_config as cfg
from main_config import MissionConfig, NavigationConfig
from motor import MotorSystem
from navigation import CoordinatePatrolController, HeadingTurnController
from odometry import OdometrySystem


# 每次上电只标定一个点。启动时会打印全部可用名称。
TARGET_NAME = "initial_waypoint"

# TARGET_NAME = "custom" 时使用此坐标。第三项可省略；存在时表示平移期间
# 保持的世界航向角（deg）。
CUSTOM_TARGET = (100.0, 70.0)

AUTO_START_DELAY_MS = 5000
MAX_TRAVEL_TIME_MS = 20000
REPORT_PERIOD_MS = 250
STOP_SETTLE_TIMEOUT_MS = 3000
FINAL_REPORT_COUNT = 20
FINAL_REPORT_PERIOD_MS = 500


def _ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.monotonic() * 1000.0)


def _ticks_diff(new_value, old_value):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(new_value, old_value)
    return new_value - old_value


def _sleep_ms(milliseconds):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(milliseconds)
    else:
        time.sleep(milliseconds / 1000.0)


class StatusLED:
    """尽力驱动 C4；电脑测试环境会自动忽略。"""

    def __init__(self, pin_name="C4"):
        self.pin = None
        try:
            from machine import Pin

            self.pin = Pin(pin_name, Pin.OUT, value=True)
        except Exception:
            self.pin = None

    def toggle(self):
        if self.pin is not None:
            self.pin.toggle()

    def set_idle(self):
        if self.pin is not None:
            self.pin.value(True)

    def set_active(self):
        if self.pin is not None:
            self.pin.value(False)


def main_coordinate_points():
    """返回当前 main_config 中所有用于导航的命名坐标点。"""
    points = [("initial_waypoint", MissionConfig.INITIAL_WAYPOINT)]

    for class_id in sorted(MissionConfig.POST_PUSH_WAYPOINT_BY_CLASS):
        points.append(
            (
                "post_push_class_{}".format(class_id),
                MissionConfig.POST_PUSH_WAYPOINT_BY_CLASS[class_id],
            )
        )

    for index, waypoint in enumerate(
        MissionConfig.APPROACH_LOSS_SEARCH_WAYPOINTS
    ):
        points.append(("approach_search_{}".format(index + 1), waypoint))

    points.append(("custom", CUSTOM_TARGET))
    return tuple(points)


def selected_target(name=TARGET_NAME):
    for point_name, waypoint in main_coordinate_points():
        if point_name == name:
            if len(waypoint) not in (2, 3):
                raise ValueError("target waypoint must contain x, y[, heading_deg]")
            return tuple(float(value) for value in waypoint)
    raise ValueError("unknown TARGET_NAME: {}".format(name))


def _point_text(waypoint):
    if len(waypoint) >= 3:
        return "({:.1f}, {:.1f}, heading={:.1f} deg)".format(
            waypoint[0], waypoint[1], waypoint[2]
        )
    return "({:.1f}, {:.1f})".format(waypoint[0], waypoint[1])


def print_point_catalog():
    print("=== main-coordinate calibration points ===")
    for name, waypoint in main_coordinate_points():
        print("{}: {}".format(name, _point_text(waypoint)))


def automatic_countdown(led, delay_ms):
    print(
        "Keep the car still at ({:.1f}, {:.1f}), heading {:.1f} deg.".format(
            cfg.INITIAL_X_CM,
            cfg.INITIAL_Y_CM,
            cfg.INITIAL_HEADING_DEG,
        )
    )
    print("Automatic movement starts in {} ms; power off to abort.".format(delay_ms))
    led.set_idle()
    start_ms = _ticks_ms()
    last_toggle_ms = start_ms
    while _ticks_diff(_ticks_ms(), start_ms) < delay_ms:
        now_ms = _ticks_ms()
        if _ticks_diff(now_ms, last_toggle_ms) >= 500:
            led.toggle()
            last_toggle_ms = now_ms
        _sleep_ms(20)
    led.set_active()


class MainCoordinateCalibrationController:
    """执行与主程序初始坐标导航相同的“先转向、再平移”。"""

    TURNING = "turning"
    TRAVELLING = "travelling"
    COMPLETE = "complete"
    FAILED = "failed"

    def __init__(self, waypoint, config=NavigationConfig):
        self.waypoint = tuple(waypoint)
        self.patrol = CoordinatePatrolController((self.waypoint,), config)
        self.turn = HeadingTurnController(config)
        self.phase = None

    def start(self, pose):
        self.patrol.reset(pose[0], pose[1], 0)
        target_heading_rad = self.patrol.target_heading_rad(pose)
        if target_heading_rad is None:
            raise ValueError("target heading cannot be calculated")
        self.turn.start(target_heading_rad)
        self.phase = self.TURNING

    def step(self, pose, yaw_rate_rad_s, dt):
        if self.phase == self.TURNING:
            result = self.turn.step(pose[2], yaw_rate_rad_s, dt)
            if result.failed:
                self.phase = self.FAILED
            elif result.done:
                self.phase = self.TRAVELLING
            return result

        if self.phase == self.TRAVELLING:
            result = self.patrol.step(pose, yaw_rate_rad_s)
            if result.failed:
                self.phase = self.FAILED
            elif result.done:
                self.phase = self.COMPLETE
            return result

        raise RuntimeError("calibration controller is not active")


def _reset_to_main_initial_pose(odometry):
    odometry.reset_position(cfg.INITIAL_X_CM, cfg.INITIAL_Y_CM)
    request_id = odometry.request_heading_reset(
        math.radians(cfg.INITIAL_HEADING_DEG)
    )
    start_ms = _ticks_ms()
    while not odometry.heading_reset_completed(request_id):
        if (
            _ticks_diff(_ticks_ms(), start_ms)
            > MissionConfig.INITIAL_HEADING_RESET_TIMEOUT_MS
        ):
            raise RuntimeError("initial heading reset timed out")
        _sleep_ms(1)


def _report_pose(label, target, pose, phase):
    dx = float(pose[0]) - float(target[0])
    dy = float(pose[1]) - float(target[1])
    distance_error = math.sqrt(dx * dx + dy * dy)
    print(
        "{} phase={} target=({:.2f},{:.2f}) "
        "odometry=({:.2f},{:.2f},{:.2f}deg) error=({:.2f},{:.2f}) "
        "distance_error={:.2f}cm".format(
            label,
            phase,
            target[0],
            target[1],
            pose[0],
            pose[1],
            math.degrees(pose[2]),
            dx,
            dy,
            distance_error,
        )
    )


def _settle_and_stop(motor, odometry, target, phase):
    """重复发送零指令完成主程序同款 S 曲线减速，再硬停。"""
    start_ms = _ticks_ms()
    while _ticks_diff(_ticks_ms(), start_ms) < STOP_SETTLE_TIMEOUT_MS:
        motor.move(0.0, 0.0, 0.0)
        state = motor.get_state()
        limited = state["limited_body"]
        wheels = state["wheel_speeds"]
        if (
            max(abs(value) for value in limited) < 0.5
            and max(abs(value) for value in wheels)
            < motor.config.stopped_speed_cm_s
        ):
            break
        _sleep_ms(10)
    motor.hard_stop()
    _sleep_ms(200)
    _report_pose("FINAL", target, odometry.get_pose(), phase)


def run_calibration(motor, odometry, target):
    controller = MainCoordinateCalibrationController(target)
    controller.start(odometry.get_pose())

    start_ms = _ticks_ms()
    last_control_ms = start_ms - MissionConfig.CONTROL_PERIOD_MS
    last_report_ms = start_ms - REPORT_PERIOD_MS
    last_phase = None

    while controller.phase not in (
        controller.COMPLETE,
        controller.FAILED,
    ):
        now_ms = _ticks_ms()
        if _ticks_diff(now_ms, start_ms) > MAX_TRAVEL_TIME_MS:
            raise RuntimeError("coordinate calibration travel timed out")

        if (
            _ticks_diff(now_ms, last_control_ms)
            >= MissionConfig.CONTROL_PERIOD_MS
        ):
            dt = max(
                0.001,
                min(_ticks_diff(now_ms, last_control_ms) / 1000.0, 0.1),
            )
            last_control_ms = now_ms
            pose = odometry.get_pose()
            state = odometry.get_state()
            result = controller.step(
                pose,
                state["yaw_rate_rad_s"],
                dt,
            )
            motor.apply_motion_step(result)

            if controller.phase != last_phase:
                print("phase={} reason={}".format(controller.phase, result.reason))
                last_phase = controller.phase

        if _ticks_diff(now_ms, last_report_ms) >= REPORT_PERIOD_MS:
            _report_pose("RUN", target, odometry.get_pose(), controller.phase)
            last_report_ms = now_ms
        _sleep_ms(1)

    if controller.phase == controller.FAILED:
        raise RuntimeError("coordinate controller failed")

    _settle_and_stop(motor, odometry, target, controller.phase)
    return odometry.get_pose()


def main():
    motor = None
    led = StatusLED()
    target = selected_target()
    print_point_catalog()
    print("Selected TARGET_NAME={}: {}".format(TARGET_NAME, _point_text(target)))

    try:
        odometry = OdometrySystem()
        motor = MotorSystem(odometry=odometry)
        motor.start()
        motor.hard_stop()
        automatic_countdown(led, AUTO_START_DELAY_MS)
        _reset_to_main_initial_pose(odometry)
        final_pose = run_calibration(motor, odometry, target)
        led.set_idle()
        print("Movement complete. Measure the real field X/Y of the car reference point.")
        for _ in range(FINAL_REPORT_COUNT):
            _report_pose("MEASURE", target, final_pose, "stopped")
            _sleep_ms(FINAL_REPORT_PERIOD_MS)
    except KeyboardInterrupt:
        print("calibration stopped")
    except Exception as error:
        print("calibration error: {}".format(repr(error)))
        try:
            import sys

            sys.print_exception(error)
        except Exception:
            pass
    finally:
        led.set_idle()
        if motor is not None:
            motor.hard_stop()
            motor.stop()


if __name__ == "__main__":
    main()

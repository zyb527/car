"""Production main flow: coordinate navigation, vision approach/orbit,\npush/yellow-line handling, repeated search, and garage return.\n\nAll tunable values are in main_config.py.\n"""

import math
import time

from control import MotionStep, clamp
from approach import ApproachController
import main_config as cfg
from main_config import (
    ApproachConfig,
    GarageConfig,
    MissionConfig,
    NavigationConfig,
    OrbitConfig,
    PushConfig,
)
from garage import GarageController
from motor import MotorSystem
from navigation import (
    ApproachLossSearchController,
    CounterclockwiseTurnController,
    CoordinatePatrolController,
    HeadingTurnController,
    PostPushPointSearchController,
)
from odometry import OdometrySystem
from orbit import OrbitController
from push import PushController
from tof import ToFSensor
from vision import VisionReceiver
from wireless_feedforward import FeedforwardSender


class MainTaskState:
    # 先原地朝向 (100, 70) 计算出的方向，稳定后再开始坐标平移。
    NAV_PRETURN = "NAV_PRETURN"
    # 坐标导航到 (100, 70)。
    NAVIGATE = "NAVIGATE"
    # 已到坐标点，停车等待首个有效视觉目标。
    WAIT_TARGET = "WAIT_TARGET"
    # 锁定目标后，前移并横移纠偏，直到 y 超过 ORBIT_START_Y_PX。
    APPROACH = "APPROACH"
    # Approach 丢失后的完整转圈与六点巡航动作由 navigation 控制器负责。
    APPROACH_SEARCH = "APPROACH_SEARCH"
    # 通过切向平移、视觉 X 转向与 ToF 半径闭环绕物，直到车头到达类别目标航向。
    ORBIT = "ORBIT"
    # 最终对准稳定后，冻结航向并单车推行至黄线。
    PUSH = "PUSH"
    # 黄线首帧命中后持续硬停，再开始右转，避免 S 曲线滑过停车线。
    POST_PUSH_YELLOW_HOLD = "POST_PUSH_YELLOW_HOLD"
    # 黄线停车后，不读取物体识别结果，强制向车体左侧旋转 180 度。
    POST_PUSH_TURN = "POST_PUSH_TURN"
    # 旋转完成后，先对准按类别确定的下一搜物坐标，再开始平移。
    POST_PUSH_NAV_PRETURN = "POST_PUSH_NAV_PRETURN"
    # 前往下一搜物坐标；此阶段恢复物体识别，发现物体立即进入靠近流程。
    POST_PUSH_NAVIGATE = "POST_PUSH_NAVIGATE"
    # 返场点到达后由 navigation 控制器负责对准、等待和限距前移搜索。
    POST_PUSH_POINT_SEARCH = "POST_PUSH_POINT_SEARCH"
    # 已按固定数量完成推行，交给新版 GarageController 执行回库。
    GARAGE = "GARAGE"
    COMPLETE = "COMPLETE"


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
        time.sleep_ms(int(milliseconds))
    else:
        time.sleep(milliseconds / 1000.0)


class MainTaskController:
    """Top-level task state coordinator; concrete controllers live in modules."""

    def __init__(self, vision_receiver=None):
        self.vision_receiver = vision_receiver
        self.patrol = CoordinatePatrolController(
            (MissionConfig.INITIAL_WAYPOINT,),
            NavigationConfig,
        )
        self.nav_turn = HeadingTurnController(NavigationConfig)
        self.post_push_turn = CounterclockwiseTurnController(NavigationConfig)
        self.approach_search = ApproachLossSearchController(
            MissionConfig.APPROACH_LOSS_SEARCH_WAYPOINTS,
            NavigationConfig,
            MissionConfig.APPROACH_LOSS_SEARCH_TURN_RAD,
            MissionConfig.SEARCH_W_RAD_S,
        )
        self.post_push_search = PostPushPointSearchController(
            NavigationConfig,
            MissionConfig.POST_PUSH_POINT_WAIT_S,
            MissionConfig.POST_PUSH_FORWARD_SPEED_CM_S,
            MissionConfig.POST_PUSH_FORWARD_MAX_DISTANCE_CM,
        )
        self.approach = ApproachController(ApproachConfig)
        self.orbit = OrbitController(OrbitConfig)
        self.push = PushController(PushConfig)
        self.garage = GarageController(GarageConfig)
        self.reset(
            (
                cfg.INITIAL_X_CM,
                cfg.INITIAL_Y_CM,
                math.radians(cfg.INITIAL_HEADING_DEG),
            )
        )

    def reset(self, pose):
        self.patrol.reset(pose[0], pose[1], 0)
        self.nav_turn.start(self.patrol.target_heading_rad(pose))
        self.post_push_turn.reset()
        self.approach_search.reset()
        self.post_push_search.reset()
        self.approach.reset()
        self.orbit.reset()
        self.push.reset()
        self.garage.reset()
        self.state = MainTaskState.NAV_PRETURN
        self.locked_class_id = 0
        self.target_heading_rad = None
        self.orbit_fallback_target_y_px = None
        self.pushed_object_count = 0
        self.post_push_yellow_hold_elapsed_s = 0.0
        self.post_push_waypoint = None
        self.post_push_class_id = 0
        self.post_push_visual_gate = None
        self.target_search_state = None
        self.visual_target_gate_open = False
        if self.vision_receiver is not None:
            self.vision_receiver.set_yellow_line(False)

    def _transition(self, state):
        self.state = state

    def _orbit_target_search_step(self):
        """Continuously rotate in place until the locked Orbit target reappears."""
        self.target_search_state = MainTaskState.ORBIT
        return MotionStep(
            (0.0, 0.0, float(MissionConfig.SEARCH_W_RAD_S)),
            reason="orbit_spin_search",
            debug={
                "state": MainTaskState.ORBIT,
                "target_search_active": True,
            },
        )

    def _clear_approach_loss_search(self):
        self.approach_search.reset()

    def _start_approach_loss_search(self, pose, yaw_rate_rad_s, dt):
        self.target_search_state = None
        # 能进入 Approach 说明启动视觉区域门此前已经通过；搜索期间保持
        # 全类别视觉有效，避免人工恢复/测试重建状态时被旧启动门再次拦截。
        self.visual_target_gate_open = True
        self.locked_class_id = 0
        self.target_heading_rad = None
        if self.vision_receiver is not None:
            self.vision_receiver.unlock_target()
        started = self.approach_search.start(pose)
        if started.failed:
            return started
        self._transition(MainTaskState.APPROACH_SEARCH)
        return self.approach_search.step(pose, yaw_rate_rad_s, dt)

    def _resume_approach_from_search(
        self, target, target_data, pose, tof_distance_mm, yaw_rate_rad_s, dt
    ):
        started = self._start_target_approach(
            target, "approach_search_target_locked_starting_approach"
        )
        if started.failed:
            return started
        return self._run_approach_controller(
            target,
            target_data,
            pose,
            tof_distance_mm,
            yaw_rate_rad_s,
            dt,
        )

    def _approach_loss_search_step(
        self, target, pose, tof_distance_mm, yaw_rate_rad_s, dt
    ):
        target_data = self._target_event(target, pose)
        if target_data is not None:
            return self._resume_approach_from_search(
                target,
                target_data,
                pose,
                tof_distance_mm,
                yaw_rate_rad_s,
                dt,
            )
        return self.approach_search.step(pose, yaw_rate_rad_s, dt)

    def _enter_orbit(self, pose, tof_distance_mm, orbit_target_y_px):
        """接收 Approach 完成事件并切换到 Orbit。"""
        self.target_search_state = None
        self.orbit_fallback_target_y_px = float(orbit_target_y_px)
        self.orbit.start_from_approach(
            pose[2],
            self.target_heading_rad,
            tof_distance_mm,
            self.orbit_fallback_target_y_px,
            class_id=self.locked_class_id,
        )
        self._transition(MainTaskState.ORBIT)

    def _run_approach_controller(
        self,
        target_for_controller,
        target_data,
        pose,
        tof_distance_mm,
        yaw_rate_rad_s,
        dt,
    ):
        result = self.approach.step(
            target_for_controller,
            tof_distance_mm,
            dt=dt,
        )
        if result.done:
            d0 = result.debug.get("orbit_radius_mm", tof_distance_mm)
            if d0 is None:
                return MotionStep.stop(
                    "approach_done_without_tof", failed=True
                )
            entry_y = result.debug.get("orbit_target_y_px")
            if entry_y is None and target_data is not None:
                entry_y = target_data[1]
            if entry_y is None:
                return MotionStep.stop(
                    "approach_done_without_target_y", failed=True
                )
            self._enter_orbit(pose, d0, entry_y)
            return MotionStep.stop(
                "approach_to_orbit",
                debug={
                    # 入轨瞬间必须绕过 S 曲线立即将前移目标清零，避免
                    # Approach 的残余前移使真实半径小于冻结的 d0。
                    "immediate_command": True,
                    "state": self.state,
                    "entry_tof_mm": self.orbit.entry_tof_mm,
                    "control_tof_mm": self.orbit.control_tof_mm,
                    "entry_center_radius_mm": self.orbit.entry_center_radius_mm,
                    "orbit_center_radius_mm": self.orbit.control_center_radius_mm,
                },
            )
        if result.failed and result.reason == "spin_search":
            return self._start_approach_loss_search(
                pose, yaw_rate_rad_s, dt
            )
        return result

    def _start_target_approach(self, target, reason):
        """Main reacts to a vision event by starting the approach controller."""
        if self.vision_receiver is None:
            return MotionStep.stop("vision_unavailable", failed=True)
        event = self.vision_receiver.lock_target(
            target, MissionConfig.TARGET_CLASS_IDS
        )
        if event is None:
            return MotionStep.stop("target_lock_rejected", failed=True)
        self.locked_class_id = event["class_id"]
        self.target_heading_rad = math.radians(
            MissionConfig.CLASS_HEADING_DEG[self.locked_class_id]
        )
        self.approach.reset()
        self.orbit.reset()
        self.push.reset()
        self._clear_approach_loss_search()
        self.post_push_search.reset()
        self.target_search_state = None
        self._transition(MainTaskState.APPROACH)
        return MotionStep.stop(
            reason,
            debug={
                "state": self.state,
                "locked_class_id": self.locked_class_id,
            },
        )

    def _start_post_push_turn(self, pose):
        """黄线停车后准备左转；本状态全程忽略视觉目标。"""
        self.post_push_waypoint = MissionConfig.POST_PUSH_WAYPOINT_BY_CLASS[
            self.locked_class_id
        ]
        self.post_push_class_id = self.locked_class_id
        self.post_push_search.reset()
        self.post_push_turn.start(pose[2])
        self._transition(MainTaskState.POST_PUSH_TURN)

    def _start_post_push_yellow_hold(self):
        """黄线首帧命中后开始硬停保持，期间不读取物体识别。"""
        self.post_push_yellow_hold_elapsed_s = 0.0
        self._transition(MainTaskState.POST_PUSH_YELLOW_HOLD)

    def _finish_push_round(self):
        """记录一次已结束推行，并统一进入黄线后的硬停阶段。"""
        self.pushed_object_count += 1
        self._start_post_push_yellow_hold()

    def _start_garage(self):
        """任务数量达到上限后，交由新版 GarageController 回库。"""
        self.garage.start(self.vision_receiver)
        self._transition(MainTaskState.GARAGE)

    def _start_post_push_navigation(self, pose):
        """转完后设置返场视觉保护，并准备前往本轮类别对应的搜物坐标。"""
        pushed_class_id = self.post_push_class_id
        axis, direction, distance_cm = (
            MissionConfig.POST_PUSH_VISUAL_GATE_BY_CLASS[pushed_class_id]
        )
        axis_index = 0 if axis == "x" else 1
        self.post_push_visual_gate = {
            "axis": axis,
            "direction": float(direction),
            "distance_cm": float(distance_cm),
            "start_coordinate_cm": float(pose[axis_index]),
        }
        self.patrol = CoordinatePatrolController(
            (self.post_push_waypoint,), NavigationConfig
        )
        self.patrol.reset(pose[0], pose[1], 0)
        self.nav_turn.start(self.patrol.target_heading_rad(pose))
        self.locked_class_id = 0
        self.target_heading_rad = None
        if self.vision_receiver is not None:
            self.vision_receiver.unlock_target()
        self._transition(MainTaskState.POST_PUSH_NAV_PRETURN)

    def _post_push_visual_gate_open(self, pose):
        """Return true once the post-push world-coordinate guard is crossed."""
        gate = self.post_push_visual_gate
        if gate is None:
            return True
        axis_index = 0 if gate["axis"] == "x" else 1
        travelled_cm = (
            float(pose[axis_index]) - gate["start_coordinate_cm"]
        ) * gate["direction"]
        if travelled_cm >= gate["distance_cm"]:
            self.post_push_visual_gate = None
            return True
        return False

    def _target_event(self, target, pose):
        """Return the target event produced by the vision boundary."""
        if self.vision_receiver is None:
            return None
        if not self.visual_target_gate_open:
            if (
                float(pose[0]) > MissionConfig.VISUAL_ENABLE_MIN_X_CM
                and float(pose[1]) > MissionConfig.VISUAL_ENABLE_MIN_Y_CM
            ):
                self.visual_target_gate_open = True
            else:
                return None
        if not self._post_push_visual_gate_open(pose):
            return None
        event = self.vision_receiver.target_event(
            target,
            MissionConfig.TARGET_CLASS_IDS,
            self.locked_class_id,
        )
        if event is None:
            return None
        return event["x"], event["y"], event["class_id"]

    def step(
        self,
        target,
        pose,
        tof_distance_mm=None,
        hazard=None,
        yaw_rate_rad_s=0.0,
        dt=0.02,
    ):
        dt = clamp(float(dt), 0.001, 0.1)

        if self.state == MainTaskState.COMPLETE:
            return MotionStep.stop(
                "push_complete",
                done=True,
                debug={"hard_stop": True, "state": self.state},
            )

        if self.state == MainTaskState.GARAGE:
            result = self.garage.step(pose, hazard, yaw_rate_rad_s, dt)
            if result.done:
                if self.vision_receiver is not None:
                    self.vision_receiver.set_yellow_line(False)
                self._transition(MainTaskState.COMPLETE)
                return MotionStep.stop(
                    result.reason,
                    done=True,
                    debug={"hard_stop": True, "state": self.state},
                )
            return result

        if self.state == MainTaskState.NAV_PRETURN:
            result = self.nav_turn.step(pose[2], yaw_rate_rad_s, dt)
            if result.failed:
                return result
            if result.done:
                self._transition(MainTaskState.NAVIGATE)
                return MotionStep.stop(
                    "navigation_heading_reached_starting_patrol",
                    debug={
                        "state": self.state,
                        "suppress_feedforward_w": True,
                    },
                )
            return result

        if self.state == MainTaskState.NAVIGATE:
            result = self.patrol.step(pose, yaw_rate_rad_s)
            if result.failed:
                return result
            if result.done:
                self._transition(MainTaskState.WAIT_TARGET)
                return MotionStep.stop(
                    "waypoint_reached_waiting_for_target",
                    debug={"state": self.state},
                )
            return result

        # 黄线停车到转满 180 度期间不读取/锁定任何物体识别结果。
        if self.state == MainTaskState.POST_PUSH_YELLOW_HOLD:
            self.post_push_yellow_hold_elapsed_s += dt
            if (
                self.post_push_yellow_hold_elapsed_s
                >= PushConfig.YELLOW_STOP_DELAY_S
            ):
                if self.pushed_object_count >= MissionConfig.TOTAL_OBJECTS_TO_PUSH:
                    self._start_garage()
                    return MotionStep.stop(
                        "push_rounds_complete_starting_garage",
                        debug={
                            "hard_stop": True,
                            "state": self.state,
                            "pushed_object_count": self.pushed_object_count,
                        },
                    )
                self._start_post_push_turn(pose)
                return MotionStep.stop(
                    "push_yellow_hold_complete_starting_counterclockwise_turn",
                    debug={"hard_stop": True, "state": self.state},
                )
            return MotionStep.stop(
                "push_yellow_line_holding",
                debug={
                    "hard_stop": True,
                    "state": self.state,
                    "hold_elapsed_s": self.post_push_yellow_hold_elapsed_s,
                },
            )

        if self.state == MainTaskState.POST_PUSH_TURN:
            result = self.post_push_turn.step(
                pose[2], yaw_rate_rad_s, dt
            )
            if result.done:
                self._start_post_push_navigation(pose)
                return MotionStep.stop(
                    "counterclockwise_180_complete_starting_navigation",
                    debug={
                        "hard_stop": True,
                        "state": self.state,
                        "next_waypoint": self.post_push_waypoint,
                    },
                )
            return result

        # 转完后才恢复全类别识别。去下一搜物点途中一旦发现物体，立即
        # 中断坐标导航并重用既有的靠近、绕行、推行流程。
        if self.state in (
            MainTaskState.POST_PUSH_NAV_PRETURN,
            MainTaskState.POST_PUSH_NAVIGATE,
            MainTaskState.POST_PUSH_POINT_SEARCH,
        ):
            target_data = self._target_event(target, pose)
            if target_data is not None:
                return self._start_target_approach(
                    target, "post_push_target_locked_starting_approach"
                )
            if self.state == MainTaskState.POST_PUSH_NAV_PRETURN:
                result = self.nav_turn.step(pose[2], yaw_rate_rad_s, dt)
                if result.failed:
                    return result
                if result.done:
                    self._transition(MainTaskState.POST_PUSH_NAVIGATE)
                    return MotionStep.stop(
                        "post_push_navigation_heading_reached_starting_patrol",
                        debug={
                            "state": self.state,
                            "suppress_feedforward_w": True,
                        },
                    )
                return result

            if self.state == MainTaskState.POST_PUSH_POINT_SEARCH:
                result = self.post_push_search.step(
                    pose, yaw_rate_rad_s, dt
                )
                if result.done:
                    self._transition(MainTaskState.WAIT_TARGET)
                    return MotionStep.stop(
                        "post_push_forward_distance_complete_waiting_for_target",
                        debug={
                            "state": self.state,
                            "hard_stop": True,
                            "forward_progress_cm": result.debug.get(
                                "forward_progress_cm"
                            ),
                        },
                    )
                return result

            result = self.patrol.step(pose, yaw_rate_rad_s)
            if result.failed:
                return result
            if result.done:
                heading_deg = (
                    MissionConfig.POST_PUSH_FORWARD_HEADING_DEG_BY_CLASS[
                        self.post_push_class_id
                    ]
                )
                started = self.post_push_search.start(pose, heading_deg)
                if started.failed:
                    return started
                self._transition(MainTaskState.POST_PUSH_POINT_SEARCH)
                return MotionStep.stop(
                    "post_push_waypoint_reached_starting_point_search",
                    debug={
                        "state": self.state,
                        "pushed_class_id": self.post_push_class_id,
                        "forward_heading_deg": heading_deg,
                        "hard_stop": True,
                    },
                )
            return result

        if self.state == MainTaskState.APPROACH_SEARCH:
            return self._approach_loss_search_step(
                target,
                pose,
                tof_distance_mm,
                yaw_rate_rad_s,
                dt,
            )

        target_data = self._target_event(target, pose)
        if self.state == MainTaskState.WAIT_TARGET:
            if target_data is None:
                return MotionStep.stop(
                    "waiting_for_valid_target",
                    debug={"hard_stop": True, "state": self.state},
                )
            return self._start_target_approach(
                target, "target_locked_starting_approach"
            )

        # 已锁定后，其他类别或越界帧按“本帧没有目标”处理，交给原控制器渐停。
        if target_data is None:
            target_for_controller = {"found": False}
        else:
            target_for_controller = target

        if self.state == MainTaskState.APPROACH:
            return self._run_approach_controller(
                target_for_controller,
                target_data,
                pose,
                tof_distance_mm,
                yaw_rate_rad_s,
                dt,
            )

        if self.state == MainTaskState.PUSH:
            result = self.push.step(
                target_for_controller,
                tof_distance_mm,
                pose[2],
                hazard=hazard,
                yaw_rate_rad_s=yaw_rate_rad_s,
                dt=dt,
            )
            if result.done:
                if self.vision_receiver is not None:
                    self.vision_receiver.set_yellow_line(False)
                    self.vision_receiver.unlock_target()
                self.push.reset()
                if result.reason in (
                    "push_yellow_line_hard_stop",
                    "push_duration_complete",
                ):
                    self._finish_push_round()
                    debug = dict(result.debug)
                    debug.update(
                        {
                            "hard_stop": True,
                            "state": self.state,
                            "locked_class_id": self.locked_class_id,
                            "pushed_object_count": self.pushed_object_count,
                        }
                    )
                    return MotionStep.stop(
                        (
                            result.reason
                            if result.reason == "push_yellow_line_hard_stop"
                            else result.reason + "_starting_post_push_hold"
                        ),
                        debug=debug,
                    )
                self._transition(MainTaskState.COMPLETE)
                return MotionStep.stop(
                    result.reason,
                    done=True,
                    debug={"hard_stop": True, "state": self.state},
                )
            if result.failed:
                if self.vision_receiver is not None:
                    self.vision_receiver.set_yellow_line(False)
            return result

        if not self.orbit.active:
            if target_data is None:
                if self.target_search_state == MainTaskState.ORBIT:
                    return self._orbit_target_search_step()
            else:
                if tof_distance_mm is None:
                    return MotionStep.stop("orbit_recovery_waiting_for_tof")
                self.orbit.start_from_approach(
                    pose[2],
                    self.target_heading_rad,
                    tof_distance_mm,
                    self.orbit_fallback_target_y_px,
                    class_id=self.locked_class_id,
                )
                self.target_search_state = None

        result = self.orbit.step(
            target_for_controller,
            tof_distance_mm,
            pose[2],
            yaw_rate_rad_s=yaw_rate_rad_s,
            dt=dt,
        )
        if result.failed and result.reason == "spin_search":
            return self._orbit_target_search_step()
        if result.failed:
            return result
        if result.done:
            self.push.start(
                self.target_heading_rad,
                self.locked_class_id,
            )
            if self.vision_receiver is not None:
                self.vision_receiver.set_yellow_line(True)
            self._transition(MainTaskState.PUSH)
            return MotionStep.stop(
                "orbit_to_push",
                debug={
                    "state": self.state,
                    "locked_class_id": self.locked_class_id,
                },
            )
        return result


def _startup_delay():
    """为操作员留出清场和保持 IMU 静止的时间。"""
    print(
        "Hold vehicle still. Test starts in {} ms.".format(
            MissionConfig.START_DELAY_MS
        )
    )
    _sleep_ms(MissionConfig.START_DELAY_MS)


def main():
    """硬件入口：每 20 ms 轮询视觉并刷新电机命令，不执行文件写入。"""
    motor = None
    sender = None
    try:
        odometry = OdometrySystem()
        motor = MotorSystem(odometry=odometry)
        vision = VisionReceiver(
            uart_id=MissionConfig.MAIN_CAMERA_UART_ID,
            baud=MissionConfig.MAIN_CAMERA_BAUD,
            timeout_ms=MissionConfig.MAIN_CAMERA_TIMEOUT_MS,
        )
        tof_sensor = ToFSensor(
            timeout_ms=MissionConfig.TOF_TIMEOUT_MS,
            valid_min_mm=MissionConfig.TOF_VALID_MIN_MM,
            valid_max_mm=MissionConfig.TOF_VALID_MAX_MM,
        )
        controller = MainTaskController(vision)
        if MissionConfig.FEEDFORWARD_ENABLED:
            sender = FeedforwardSender(
                period_ms=MissionConfig.FEEDFORWARD_TX_PERIOD_MS
            )

        motor.start()
        motor.hard_stop()
        print(
            "push yellow config: hazard_type={}".format(
                PushConfig.HAZARD_YELLOW
            )
        )
        if sender is None:
            _startup_delay()
        else:
            sender.hold_zero_for(MissionConfig.START_DELAY_MS)
        odometry.reset_position(cfg.INITIAL_X_CM, cfg.INITIAL_Y_CM)
        heading_reset_id = odometry.request_heading_reset(
            math.radians(cfg.INITIAL_HEADING_DEG)
        )
        heading_reset_start_ms = _ticks_ms()
        while not odometry.heading_reset_completed(heading_reset_id):
            if (
                _ticks_diff(_ticks_ms(), heading_reset_start_ms)
                > MissionConfig.INITIAL_HEADING_RESET_TIMEOUT_MS
            ):
                raise RuntimeError("initial heading reset timed out")
            _sleep_ms(1)
        controller.reset(odometry.get_pose())

        last_control_ms = _ticks_ms() - MissionConfig.CONTROL_PERIOD_MS
        previous_state = None
        previous_reason = None
        previous_push_hazard = None
        suppress_feedforward_w = False
        while True:
            now_ms = _ticks_ms()
            if (
                _ticks_diff(now_ms, last_control_ms)
                >= MissionConfig.CONTROL_PERIOD_MS
            ):
                dt = max(
                    0.001,
                    min(_ticks_diff(now_ms, last_control_ms) / 1000.0, 0.1),
                )
                last_control_ms = now_ms
                vision.poll(now_ms)
                tof_distance_mm = tof_sensor.update(now_ms)
                target, hazard = vision.get_data()
                odometry_state = odometry.get_state()
                pose = odometry.get_pose()
                result = controller.step(
                    target,
                    pose,
                    tof_distance_mm=tof_distance_mm,
                    hazard=hazard,
                    yaw_rate_rad_s=odometry_state["yaw_rate_rad_s"],
                    dt=dt,
                )

                motor.apply_motion_step(result)
                suppress_feedforward_w = bool(
                    result.debug.get("suppress_feedforward_w", False)
                )

                if controller.state != previous_state:
                    print("state={} reason={}".format(controller.state, result.reason))
                    previous_state = controller.state
                # 仅在控制原因变化时输出，避免高频串口打印影响 20 ms 控制周期。
                if result.reason != previous_reason:
                    phase = result.debug.get("phase", "-")
                    print(
                        "reason={} phase={} tof={} target={} cmd=({:.1f},{:.1f},{:.2f})".format(
                            result.reason,
                            phase,
                            tof_distance_mm,
                            target,
                            result.command[0],
                            result.command[1],
                            result.command[2],
                        )
                    )
                    previous_reason = result.reason

                # 仅在 Push 阶段的黄线数据变化时打印，直接确认车端实际收到
                # 的 (hazard_type, y)。None 表示该帧没有有效 hazard。
                if controller.state == MainTaskState.PUSH:
                    push_hazard = result.debug.get("push_hazard")
                    if push_hazard != previous_push_hazard:
                        print("push_hazard={}".format(push_hazard))
                        previous_push_hazard = push_hazard
                else:
                    previous_push_hazard = None
            if sender is not None:
                feedforward_output_scale = (
                    PushConfig.WIRELESS_FEEDFORWARD_SCALE
                    if controller.state == MainTaskState.PUSH
                    else 1.0
                )
                sender.send_blended_motion_if_due(
                    motor,
                    odometry,
                    MissionConfig.FEEDFORWARD_MEASURED_WEIGHT,
                    now_ms,
                    straight_without_w=suppress_feedforward_w,
                    output_scale=feedforward_output_scale,
                )
            _sleep_ms(1)
    except KeyboardInterrupt:
        print("test stopped")
    except Exception as error:
        print("test error: {}".format(repr(error)))
        try:
            import sys
            sys.print_exception(error)
        except Exception:
            pass
    finally:
        if motor is not None:
            motor.hard_stop()
        if sender is not None:
            sender.send_zero_frames()
        if motor is not None:
            motor.stop()


if __name__ == "__main__":
    main()

"""On-car square-path test for checking odometry coordinate integration.

Place the car at the lower-left corner of a clear 1 m x 1 m square, select
TEST_MODE below, and run this file on the main car controller.

FORWARD:
    The car turns to face each next waypoint and drives forward.

LATERAL_INWARD:
    The car faces the inside of the square on every edge and moves laterally.

ROTATE_CCW:
    The car turns counterclockwise once in place and reports coordinate drift.

UART0 emits one CSV-like ODOM record every 150 ms. Coordinates are centimetres.
"""

import math
import time

try:
    from machine import UART
except ImportError:
    UART = None

from main_config import NavigationConfig
from motor import MotorSystem
from navigation import (
    CoordinatePatrolController,
    CounterclockwiseTurnController,
    HeadingTurnController,
)
from odometry import OdometrySystem


MODE_FORWARD = "FORWARD"
MODE_LATERAL_INWARD = "LATERAL_INWARD"
MODE_ROTATE_CCW = "ROTATE_CCW"

# Change only this line to select the test mode.
TEST_MODE = MODE_FORWARD

CONTROL_PERIOD_MS = 20
UART_PERIOD_MS = 150
UART_ID = 0
UART_BAUD = 115200
START_DELAY_MS = 2000
HEADING_RESET_TIMEOUT_MS = 500
SEGMENT_TIMEOUT_MS = 15000
FULL_TURN_RAD = 2.0 * math.pi


class SquareNavigationConfig(NavigationConfig):
    """Moderate speeds for observing a repeatable 1 m square."""

    POSITION_TOLERANCE_CM = 3.0
    CROSS_TRACK_TOLERANCE_CM = 4.0
    PATH_FAST_DISTANCE_CM = 40.0
    PATH_SLOW_DISTANCE_CM = 15.0
    PATH_FAST_SPEED_CM_S = 40.0
    PATH_MID_SPEED_CM_S = 28.0
    PATH_SLOW_SPEED_CM_S = 18.0
    PATH_CROSS_MAX_SPEED_CM_S = 25.0
    PATH_MAX_SPEED_CM_S = 45.0
    TRANSLATE_MAX_W_RAD_S = 1.0


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
        time.sleep(float(milliseconds) / 1000.0)


def _mode_waypoints(mode):
    if mode == MODE_FORWARD:
        # Each heading follows the direction of travel.
        return (
            (100.0, 0.0, 0.0),
            (100.0, 100.0, 90.0),
            (0.0, 100.0, 180.0),
        )
    if mode == MODE_LATERAL_INWARD:
        # Bottom edge faces north, right edge faces west, top edge faces south.
        # The resulting translation is lateral while the car faces the square.
        return (
            (100.0, 0.0, 90.0),
            (100.0, 100.0, 180.0),
            (0.0, 100.0, -90.0),
        )
    raise ValueError("unknown TEST_MODE: {}".format(mode))


def _uart_write(uart, line):
    uart.write(line + "\r\n")


def _event(uart, name, phase, segment_index, waypoint, pose, detail=""):
    _uart_write(
        uart,
        (
            "EVENT,name={},mode={},phase={},segment={},target_x={:.2f},"
            "target_y={:.2f},x_cm={:.2f},y_cm={:.2f},heading_deg={:.2f},{}"
        ).format(
            name,
            TEST_MODE,
            phase,
            segment_index + 1,
            waypoint[0],
            waypoint[1],
            pose[0],
            pose[1],
            math.degrees(pose[2]),
            detail,
        ),
    )


def _send_odometry(
    uart,
    elapsed_ms,
    phase,
    segment_index,
    waypoint,
    pose,
    odometry_state,
):
    _uart_write(
        uart,
        (
            "ODOM,time_ms={},mode={},phase={},segment={},target_x={:.2f},"
            "target_y={:.2f},x_cm={:.3f},y_cm={:.3f},heading_deg={:.3f},"
            "body_vx_cm_s={:.3f},body_vy_cm_s={:.3f},yaw_rate_rad_s={:.4f}"
        ).format(
            elapsed_ms,
            TEST_MODE,
            phase,
            segment_index + 1,
            waypoint[0],
            waypoint[1],
            pose[0],
            pose[1],
            math.degrees(pose[2]),
            odometry_state["body_vx_cm_s"],
            odometry_state["body_vy_cm_s"],
            odometry_state["yaw_rate_rad_s"],
        ),
    )


def _reset_initial_pose(odometry, heading_rad):
    odometry.reset_position(0.0, 0.0)
    request_id = odometry.request_heading_reset(heading_rad)
    start_ms = _ticks_ms()
    while not odometry.heading_reset_completed(request_id):
        if _ticks_diff(_ticks_ms(), start_ms) > HEADING_RESET_TIMEOUT_MS:
            raise RuntimeError("initial heading reset timed out")
        _sleep_ms(1)


def _run_rotation_test():
    """Turn once in place and report the odometry position drift."""
    uart = UART(UART_ID)
    uart.init(UART_BAUD)
    waypoint = (0.0, 0.0, 360.0)
    odometry = None
    motor = None
    try:
        odometry = OdometrySystem()
        motor = MotorSystem(odometry=odometry)
        turn = CounterclockwiseTurnController(SquareNavigationConfig)
        motor.start()
        motor.hard_stop()
        _uart_write(
            uart,
            "EVENT,name=STARTUP_HOLD,mode={},duration_ms={}".format(
                TEST_MODE, START_DELAY_MS
            ),
        )
        _sleep_ms(START_DELAY_MS)
        _reset_initial_pose(odometry, 0.0)
        turn.start(odometry.get_pose()[2])

        start_ms = _ticks_ms()
        last_control_ms = start_ms - CONTROL_PERIOD_MS
        last_uart_ms = start_ms - UART_PERIOD_MS
        _event(uart, "TEST_START", "ROTATE", 0, waypoint, odometry.get_pose())

        while True:
            now_ms = _ticks_ms()
            pose = odometry.get_pose()
            state = odometry.get_state()
            if _ticks_diff(now_ms, last_uart_ms) >= UART_PERIOD_MS:
                _send_odometry(
                    uart,
                    _ticks_diff(now_ms, start_ms),
                    "ROTATE",
                    0,
                    waypoint,
                    pose,
                    state,
                )
                last_uart_ms = now_ms
            if _ticks_diff(now_ms, last_control_ms) < CONTROL_PERIOD_MS:
                _sleep_ms(1)
                continue

            dt = max(
                0.001,
                min(_ticks_diff(now_ms, last_control_ms) / 1000.0, 0.1),
            )
            last_control_ms = now_ms
            result = turn.step(
                pose[2], state["yaw_rate_rad_s"], dt, angle_rad=FULL_TURN_RAD
            )
            if result.failed:
                raise RuntimeError(result.reason)
            if not result.done:
                motor.apply_motion_step(result)
                continue

            motor.hard_stop()
            final_pose = odometry.get_pose()
            drift_cm = math.sqrt(final_pose[0] ** 2 + final_pose[1] ** 2)
            _event(
                uart,
                "TEST_COMPLETE",
                "STOP",
                0,
                waypoint,
                final_pose,
                "rotation_coordinate_drift_cm={:.3f}".format(drift_cm),
            )
            return
    except KeyboardInterrupt:
        _uart_write(uart, "EVENT,name=INTERRUPTED,mode={}".format(TEST_MODE))
    except Exception as error:
        _uart_write(
            uart,
            "EVENT,name=ERROR,mode={},detail={}".format(TEST_MODE, repr(error)),
        )
        try:
            import sys

            sys.print_exception(error)
        except Exception:
            pass
    finally:
        if motor is not None:
            motor.hard_stop()
            motor.stop()


def main():
    if UART is None:
        raise RuntimeError("machine.UART is required; run this test on the car")

    if TEST_MODE == MODE_ROTATE_CCW:
        return _run_rotation_test()

    waypoints = _mode_waypoints(TEST_MODE)
    initial_heading_rad = math.radians(waypoints[0][2])
    uart = UART(UART_ID)
    uart.init(UART_BAUD)
    odometry = None
    motor = None

    try:
        odometry = OdometrySystem()
        motor = MotorSystem(odometry=odometry)
        patrol = CoordinatePatrolController(waypoints, SquareNavigationConfig)
        turn = HeadingTurnController(SquareNavigationConfig)

        motor.start()
        motor.hard_stop()
        _uart_write(
            uart,
            "EVENT,name=STARTUP_HOLD,mode={},duration_ms={}".format(
                TEST_MODE, START_DELAY_MS
            ),
        )
        _sleep_ms(START_DELAY_MS)
        _reset_initial_pose(odometry, initial_heading_rad)

        patrol.reset(0.0, 0.0, 0)
        turn.start(math.radians(waypoints[0][2]))
        phase = "TURN"
        segment_index = 0
        test_start_ms = _ticks_ms()
        segment_start_ms = test_start_ms
        last_control_ms = test_start_ms - CONTROL_PERIOD_MS
        last_uart_ms = test_start_ms - UART_PERIOD_MS

        _event(
            uart,
            "TEST_START",
            phase,
            segment_index,
            waypoints[segment_index],
            odometry.get_pose(),
        )

        while segment_index < len(waypoints):
            now_ms = _ticks_ms()

            if _ticks_diff(now_ms, last_uart_ms) >= UART_PERIOD_MS:
                _send_odometry(
                    uart,
                    _ticks_diff(now_ms, test_start_ms),
                    phase,
                    segment_index,
                    waypoints[segment_index],
                    odometry.get_pose(),
                    odometry.get_state(),
                )
                last_uart_ms = now_ms

            if _ticks_diff(now_ms, last_control_ms) < CONTROL_PERIOD_MS:
                _sleep_ms(1)
                continue

            dt = max(
                0.001,
                min(_ticks_diff(now_ms, last_control_ms) / 1000.0, 0.1),
            )
            last_control_ms = now_ms
            pose = odometry.get_pose()
            state = odometry.get_state()

            if _ticks_diff(now_ms, segment_start_ms) > SEGMENT_TIMEOUT_MS:
                raise RuntimeError(
                    "segment {} {} timed out".format(segment_index + 1, phase)
                )

            if phase == "TURN":
                result = turn.step(pose[2], state["yaw_rate_rad_s"], dt)
                if result.failed:
                    raise RuntimeError(result.reason)
                if result.done:
                    motor.hard_stop()
                    phase = "TRANSLATE"
                    segment_start_ms = now_ms
                    _event(
                        uart,
                        "TURN_DONE",
                        phase,
                        segment_index,
                        waypoints[segment_index],
                        pose,
                    )
                else:
                    motor.apply_motion_step(result)
                continue

            result = patrol.step(pose, state["yaw_rate_rad_s"])
            if result.failed:
                raise RuntimeError(result.reason)
            if not result.done:
                motor.apply_motion_step(result)
                continue

            motor.hard_stop()
            _event(
                uart,
                "WAYPOINT_REACHED",
                phase,
                segment_index,
                waypoints[segment_index],
                pose,
                "position_error_cm={:.3f}".format(
                    math.sqrt(
                        (waypoints[segment_index][0] - pose[0]) ** 2
                        + (waypoints[segment_index][1] - pose[1]) ** 2
                    )
                ),
            )

            segment_index += 1
            if segment_index >= len(waypoints):
                break

            patrol.advance(pose[0], pose[1])
            turn.start(math.radians(waypoints[segment_index][2]))
            phase = "TURN"
            segment_start_ms = now_ms

        final_pose = odometry.get_pose()
        _event(
            uart,
            "TEST_COMPLETE",
            "STOP",
            len(waypoints) - 1,
            waypoints[-1],
            final_pose,
            "final_error_cm={:.3f}".format(
                math.sqrt(final_pose[0] ** 2 + (100.0 - final_pose[1]) ** 2)
            ),
        )
    except KeyboardInterrupt:
        if uart is not None:
            _uart_write(uart, "EVENT,name=INTERRUPTED,mode={}".format(TEST_MODE))
    except Exception as error:
        if uart is not None:
            _uart_write(
                uart,
                "EVENT,name=ERROR,mode={},detail={}".format(
                    TEST_MODE, repr(error)
                ),
            )
        try:
            import sys

            sys.print_exception(error)
        except Exception:
            pass
    finally:
        if motor is not None:
            motor.hard_stop()
            motor.stop()


if __name__ == "__main__":
    main()

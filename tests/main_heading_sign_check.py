"""On-car check for the heading sign during a positive (CCW) turn command.

Run this file directly on the main car.  It does not import or run main.py and
does not change any mission configuration.  The car commands a low-speed
positive angular velocity for a short time, then reports whether heading
increased or decreased and whether the current full-turn search accumulator
would make progress.

Safety:
    Put the car on a flat, clear floor before running this file.  Keep enough
    space around it and be ready to interrupt the script.  The motors are hard
    stopped on completion, interruption, or error.
"""

import math
import os
import sys
import time


try:
    PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if PROJECT_DIR not in sys.path:
        sys.path.insert(0, PROJECT_DIR)
except Exception:
    if ".." not in sys.path:
        sys.path.append("..")
    if "/" not in sys.path:
        sys.path.append("/")

from motor import MotorSystem
from odometry import OdometrySystem


# Positive w is defined as counterclockwise by motor.py/navigation.py.
TEST_W_RAD_S = 0.70
TEST_DURATION_S = 2.5
START_DELAY_S = 5
CONTROL_PERIOD_MS = 20
PRINT_PERIOD_MS = 200
MIN_VALID_TURN_DEG = 15.0


def _ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.monotonic() * 1000.0)


def _ticks_diff(new_value, old_value):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(new_value, old_value)
    return new_value - old_value


def _sleep_ms(delay_ms):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(delay_ms)
    else:
        time.sleep(delay_ms / 1000.0)


def _normalize_angle(angle_rad):
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


def classify_heading_change(signed_change_deg, minimum_deg=MIN_VALID_TURN_DEG):
    if signed_change_deg >= minimum_deg:
        return "INCREASING"
    if signed_change_deg <= -minimum_deg:
        return "DECREASING"
    return "INCONCLUSIVE"


def _print_result(signed_rad, positive_rad, negative_rad, unwrapped_rad):
    signed_deg = math.degrees(signed_rad)
    positive_deg = math.degrees(positive_rad)
    negative_deg = math.degrees(negative_rad)
    unwrapped_deg = math.degrees(unwrapped_rad)
    direction = classify_heading_change(signed_deg)

    print("\n=== RESULT ===")
    print("normalized_signed_change_deg={:.2f}".format(signed_deg))
    print("unwrapped_change_deg={:.2f}".format(unwrapped_deg))
    print("positive_progress_deg={:.2f}".format(positive_deg))
    print("negative_progress_deg={:.2f}".format(negative_deg))
    print("heading_direction={}".format(direction))

    if direction == "INCREASING":
        print("DIAGNOSIS: heading increases during the positive turn command.")
        print("The one-circle accumulator sign is correct in this test.")
    elif direction == "DECREASING":
        print("DIAGNOSIS: heading decreases during the positive turn command.")
        print("If the physical motion was counterclockwise: BUG_REPRODUCED.")
        print("The current search only adds positive deltas, so its one-circle")
        print("progress can stay near zero and spin forever.")
    else:
        print("DIAGNOSIS: no reliable heading change was measured.")
        print("Check that the car physically turned and inspect the IMU/odometry.")


def main():
    motor = None
    print("=== MAIN CAR HEADING SIGN CHECK ===")
    print("Place the car on a flat, clear floor.")
    print("Expected physical motion: COUNTERCLOCKWISE.")
    print("The car will rotate at {:.2f} rad/s for {:.1f} s.".format(
        TEST_W_RAD_S,
        TEST_DURATION_S,
    ))

    try:
        odometry = OdometrySystem()
        motor = MotorSystem(odometry=odometry)

        print("Starting motor system and calibrating IMU; keep the car still...")
        motor.start()
        motor.hard_stop()

        for remaining in range(START_DELAY_S, 0, -1):
            print("Turn starts in {}...".format(remaining))
            _sleep_ms(1000)

        initial = odometry.get_state()
        last_heading = initial["heading_rad"]
        initial_unwrapped = initial["heading_unwrapped_rad"]
        signed_progress = 0.0
        positive_progress = 0.0
        negative_progress = 0.0

        start_ms = _ticks_ms()
        last_print_ms = start_ms - PRINT_PERIOD_MS
        print("time_s,heading_deg,unwrapped_delta_deg,yaw_rate_rad_s")

        while _ticks_diff(_ticks_ms(), start_ms) < int(TEST_DURATION_S * 1000):
            now_ms = _ticks_ms()
            motor.move(0.0, 0.0, TEST_W_RAD_S)

            state = odometry.get_state()
            heading = state["heading_rad"]
            delta = _normalize_angle(heading - last_heading)
            signed_progress += delta
            if delta > 0.0:
                positive_progress += delta
            elif delta < 0.0:
                negative_progress += -delta
            last_heading = heading

            if _ticks_diff(now_ms, last_print_ms) >= PRINT_PERIOD_MS:
                elapsed_s = _ticks_diff(now_ms, start_ms) / 1000.0
                unwrapped_delta = (
                    state["heading_unwrapped_rad"] - initial_unwrapped
                )
                print("{:.2f},{:.2f},{:.2f},{:.3f}".format(
                    elapsed_s,
                    math.degrees(heading),
                    math.degrees(unwrapped_delta),
                    state["yaw_rate_rad_s"],
                ))
                last_print_ms = now_ms

            _sleep_ms(CONTROL_PERIOD_MS)

        motor.hard_stop()
        _sleep_ms(300)
        final_state = odometry.get_state()
        final_unwrapped_delta = (
            final_state["heading_unwrapped_rad"] - initial_unwrapped
        )
        _print_result(
            signed_progress,
            positive_progress,
            negative_progress,
            final_unwrapped_delta,
        )

    except KeyboardInterrupt:
        print("Test interrupted by user.")
    except Exception as error:
        print("TEST ERROR: {}".format(error))
        try:
            sys.print_exception(error)
        except Exception:
            pass
    finally:
        if motor is not None:
            motor.hard_stop()
            motor.stop()
        print("Motors stopped.")


if __name__ == "__main__":
    main()

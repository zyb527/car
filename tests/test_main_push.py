import math
import os
import sys
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import main_push  # noqa: E402
from main_config import PushConfig  # noqa: E402


class FastPushConfig(PushConfig):
    PUSH_RAMP_S = 0.1
    PUSH_DURATION_S = 0.3
    TARGET_LOSS_TIMEOUT_S = 0.1


class FastForcedAvoidanceConfig(main_push.PushAvoidanceTestConfig):
    FORCED_AVOIDANCE_DELAY_S = 0.04
    AVOID_RETURN_STABLE_S = 0.0


def target(found=True, x=160.0, y=170.0, class_id=1):
    return {"found": found, "x": x, "y": y, "class_id": class_id}


class MainPushTests(unittest.TestCase):
    def test_waits_for_target_before_starting_push(self):
        session = main_push.PushTestSession(FastPushConfig)

        result = session.step(None, None, 0.0, dt=0.02)

        self.assertEqual(session.state, main_push.WAIT_TARGET)
        self.assertEqual(result.command, (0.0, 0.0, 0.0))
        self.assertEqual(result.reason, "push_wait_target")

    def test_starts_push_with_current_heading_and_forward_command(self):
        session = main_push.PushTestSession(FastPushConfig)
        heading = math.radians(15.0)

        result = session.step(target(), None, heading, dt=0.02)

        self.assertEqual(session.state, main_push.PUSHING)
        self.assertAlmostEqual(session.controller.target_heading_rad, heading)
        self.assertGreater(result.command[1], 0.0)

    def test_locks_initial_target_x_and_y_as_push_references(self):
        session = main_push.PushTestSession(FastPushConfig)

        session.step(target(x=142.5, y=88.0), None, 0.0, dt=0.02)

        self.assertEqual(session.initial_target_x_px, 142.5)
        self.assertEqual(session.initial_target_y_px, 88.0)
        self.assertEqual(session.config.TARGET_CENTER_X_PX, 142.5)
        self.assertEqual(session.config.TARGET_Y_PX, 88.0)
        self.assertEqual(PushConfig.TARGET_CENTER_X_PX, 160.0)
        self.assertEqual(PushConfig.TARGET_Y_PX, 170.0)

    def test_obstacle_uses_existing_avoidance_state_machine(self):
        session = main_push.PushTestSession(FastPushConfig)

        result = session.step(
            target(),
            {"found": True, "type": 1, "x": 100.0, "y": 90.0},
            0.0,
            dt=0.02,
        )

        self.assertEqual(session.state, main_push.PUSHING)
        self.assertEqual(result.reason, "push_running_avoid_track")
        self.assertLess(result.command[2], 0.0)

    def test_completed_push_finishes_test_session(self):
        session = main_push.PushTestSession(FastPushConfig)
        result = None
        for _ in range(20):
            result = session.step(target(), None, 0.0, dt=0.02)
            if result.done:
                break

        self.assertTrue(result.done)
        self.assertEqual(session.state, main_push.FINISHED)
        self.assertEqual(result.reason, "push_duration_complete")

    def test_forced_avoidance_starts_after_delay_and_stops_after_return(self):
        session = main_push.PushTestSession(FastForcedAvoidanceConfig)

        session.step(target(), None, 0.0, dt=0.02)
        result = session.step(target(), None, 0.0, dt=0.02)
        self.assertEqual(session.forced_phase, "OUTBOUND")
        self.assertEqual(result.reason, "push_running_avoid_track")
        self.assertGreater(result.command[2], 0.0)

        # 模拟主车已经转到45°，下一帧清除模拟障碍并开始回正。
        result = session.step(target(), None, math.radians(45.0), dt=0.02)
        self.assertEqual(session.forced_phase, "RETURN")
        self.assertEqual(result.reason, "push_running_avoid_clear_hold")

        # 清障保持被设为零，下一帧开始向原航向回转。
        result = session.step(target(), None, math.radians(45.0), dt=0.02)
        self.assertEqual(result.reason, "push_running_avoid_return")
        self.assertLess(result.command[2], 0.0)

        # 里程计回到原航向后，测试会自动完成并停车。
        result = session.step(target(), None, 0.0, dt=0.02)
        self.assertTrue(result.done)
        self.assertEqual(
            result.reason,
            "push_forced_avoidance_complete",
        )

    def test_old_sender_fallback_keeps_turning_w(self):
        class Motor:
            def get_limited_physical_command(self):
                return (10.0, 20.0, -0.7)

        class OldSender:
            def __init__(self):
                self.command = None

            def send_motor_command(self, motor):
                raise AssertionError("fallback should not call old method")

            def send(self, vx, vy, w):
                self.command = (vx, vy, w)

        sender = OldSender()
        command = main_push._send_motor_feedforward(
            sender,
            Motor(),
            (0.0, 20.0, -0.7),
        )

        self.assertEqual(command, (10.0, 20.0, -0.7))
        self.assertEqual(sender.command, (10.0, 20.0, -0.7))


if __name__ == "__main__":
    unittest.main()

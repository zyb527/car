import os
import sys
import unittest

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from vision import VisionReceiver


class MockUART:
    def __init__(self):
        self.buffer = b""
        self.written = b""

    def init(self, baud):
        pass

    def write(self, data):
        self.written += data

    def any(self):
        return len(self.buffer) > 0

    def read(self):
        ret = self.buffer
        self.buffer = b""
        return ret


class VisionReceiverTests(unittest.TestCase):
    def test_parses_combined_frame(self):
        receiver = VisionReceiver(timeout_ms=500)
        receiver.uart = MockUART()
        
        # target_found,x,y,id, hazard_found,x,y,type
        receiver.uart.buffer = b"1,160.0,170.0,2,1,145.0,52.0,1\n"
        receiver.poll(now_ms=100)
        
        target, hazard = receiver.get_data()
        self.assertTrue(target["found"])
        self.assertEqual(target["x"], 160.0)
        self.assertEqual(target["class_id"], 2)
        
        self.assertTrue(hazard["found"])
        self.assertEqual(hazard["x"], 145.0)
        self.assertEqual(hazard["type"], 1)

    def test_handles_partial_and_split_frames(self):
        receiver = VisionReceiver(timeout_ms=500)
        receiver.uart = MockUART()
        
        # Half frame
        receiver.uart.buffer = b"1,160.0,170.0,2,"
        receiver.poll(now_ms=100)
        
        target, hazard = receiver.get_data()
        self.assertIsNone(target)
        self.assertIsNone(hazard)
        
        # Second half
        receiver.uart.buffer = b"0,0.0,0.0,0\n"
        receiver.poll(now_ms=110)
        target, hazard = receiver.get_data()
        self.assertTrue(target["found"])
        self.assertFalse(hazard["found"])

    def test_camera_timeout_clears_all(self):
        receiver = VisionReceiver(timeout_ms=500)
        receiver.uart = MockUART()
        
        receiver.uart.buffer = b"1,160,170,2,1,145,52,1\n"
        receiver.poll(now_ms=100)
        
        self.assertIsNotNone(receiver.get_data()[0])
        self.assertIsNotNone(receiver.get_data()[1])
        
        # No new frames for 600ms
        receiver.poll(now_ms=710)
        target, hazard = receiver.get_data()
        
        self.assertIsNone(target)
        self.assertIsNone(hazard)

    def test_target_timeout_within_grace_period(self):
        receiver = VisionReceiver(timeout_ms=500)
        receiver.uart = MockUART()
        
        # Find target
        receiver.uart.buffer = b"1,160,170,2,0,0,0,0\n"
        receiver.poll(now_ms=100)
        
        # Set filter (triggers grace period)
        receiver.set_target_filter(2)
        # Verify it wrote to UART
        self.assertEqual(receiver.uart.written, b'\xFF\xFF\x04\x02\x00')
        
        # Frame arrives but target not found, at 400ms (300ms diff, >500ms from 100 but within 800ms grace)
        # Actually target is timed out since 100ms.
        # now_ms = 700. target_found_ms = 100 (diff 600, > 500 timeout).
        # switch_ms = roughly 100.
        receiver.filter_switch_ms = 150 # mock
        receiver.uart.buffer = b"0,160,170,2,0,0,0,0\n"
        receiver.poll(now_ms=700) # Switch diff = 550 (< 800)
        
        target, hazard = receiver.get_data()
        self.assertIsNotNone(target)
        self.assertTrue(target["found"]) # Keeps previous state due to grace period!
        
        # Wait until grace period expires
        receiver.uart.buffer = b"0,160,170,2,0,0,0,0\n"
        receiver.poll(now_ms=1000) # Switch diff = 850 (> 800)
        target, hazard = receiver.get_data()
        self.assertFalse(target["found"])

if __name__ == "__main__":
    unittest.main()

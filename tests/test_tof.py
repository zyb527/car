import os
import sys
import unittest

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from tof import ToFSensor


class MockSensor:
    def __init__(self):
        self.value = 0
        self.should_raise = False

    def read(self):
        if self.should_raise:
            raise Exception("Read error")
        return self.value


class ToFSensorTests(unittest.TestCase):
    def setUp(self):
        self.tof = ToFSensor(timeout_ms=300, valid_min_mm=20, valid_max_mm=1500)
        self.mock_sensor = MockSensor()
        self.tof.sensor = self.mock_sensor

    def test_valid_reading(self):
        self.mock_sensor.value = 100
        self.tof.update(now_ms=100)
        
        self.assertTrue(self.tof.found)
        self.assertEqual(self.tof.get_distance_or_default(999), 100)

    def test_filters_out_of_range(self):
        # Too small
        self.mock_sensor.value = 10
        self.tof.update(now_ms=100)
        self.assertFalse(self.tof.found)
        self.assertEqual(self.tof.get_distance_or_default(999), 999)
        
        # Too large
        self.mock_sensor.value = 2000
        self.tof.update(now_ms=110)
        self.assertFalse(self.tof.found)
        
        # Valid
        self.mock_sensor.value = 500
        self.tof.update(now_ms=120)
        self.assertTrue(self.tof.found)
        self.assertEqual(self.tof.get_distance_or_default(999), 500)

    def test_timeout_clears_found_state(self):
        self.mock_sensor.value = 200
        self.tof.update(now_ms=100)
        self.assertTrue(self.tof.found)
        
        # Exception during read, but within timeout
        self.mock_sensor.should_raise = True
        self.tof.update(now_ms=200)
        self.assertTrue(self.tof.found) # Still found
        self.assertEqual(self.tof.get_distance_or_default(999), 200)
        
        # Timeout elapsed
        self.tof.update(now_ms=450) # 350ms diff > 300ms
        self.assertFalse(self.tof.found)
        self.assertEqual(self.tof.get_distance_or_default(999), 999)


if __name__ == "__main__":
    unittest.main()

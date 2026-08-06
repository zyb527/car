"""主车 ToF 通信模块。

通过 DL1X 传感器读取距离，过滤异常值，
维护最近一次有效测距并在超时后重置。
"""

import time

def _ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.monotonic() * 1000.0)

def _ticks_diff(new_value, old_value):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(new_value, old_value)
    return new_value - old_value


class ToFSensor:
    """通过 DL1X 读取距离，维护最近一次有效测距。"""

    def __init__(self, timeout_ms=300, valid_min_mm=20, valid_max_mm=1500):
        self.timeout_ms = timeout_ms
        self.valid_min_mm = valid_min_mm
        self.valid_max_mm = valid_max_mm
        
        self.sensor = None
        try:
            from seekfree import DL1X
            self.sensor = DL1X()
        except ImportError:
            pass

        self.last_distance_mm = 0
        self.found = False
        self.last_seen_ms = 0

    def update(self, now_ms=None):
        if now_ms is None:
            now_ms = _ticks_ms()

        valid_data_received = False
        
        if self.sensor:
            try:
                distance_mm = int(self.sensor.read())
                if distance_mm < 5:
                    pass
                elif self.valid_min_mm <= distance_mm <= self.valid_max_mm:
                    self.last_distance_mm = distance_mm
                    self.last_seen_ms = now_ms
                    self.found = True
                    valid_data_received = True
            except Exception:
                pass

        if not valid_data_received:
            if self.found and _ticks_diff(now_ms, self.last_seen_ms) > self.timeout_ms:
                self.found = False
        return self.last_distance_mm if self.found else None

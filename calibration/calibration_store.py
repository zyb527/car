"""MicroPython 与电脑端 Python 均可使用的仅追加 JSON 行存储。"""

import time

try:
    import ujson as json
except ImportError:
    import json


DEFAULT_LOG_PATH = "calibration_data.txt"


def ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.monotonic() * 1000.0)


class CalibrationLog:
    def __init__(self, test_name, path=DEFAULT_LOG_PATH, batch_size=2):
        self.test_name = test_name
        self.path = path
        self.session_start_ms = ticks_ms()
        self.batch_size = max(1, int(batch_size))
        self.pending_lines = []
        self.file = open(path, "a")
        self.write("session_start", test_name=test_name)

    def write(self, record_type, **values):
        record = {
            "type": record_type,
            "test": self.test_name,
            "session_start_ms": self.session_start_ms,
            "time_ms": ticks_ms(),
        }
        for key in values:
            record[key] = values[key]
        self.pending_lines.append(json.dumps(record) + "\n")
        if len(self.pending_lines) >= self.batch_size or record_type != "sample":
            self.flush()
        return record

    def flush(self):
        if self.file is None or not self.pending_lines:
            return
        # 不要拼接大量 JSON 字符串：即使总剩余内存足够，MicroPython 也可能
        # 无法分配一块足够大的连续缓冲区。
        for line in self.pending_lines:
            self.file.write(line)
        self.file.flush()
        self.pending_lines = []

    def close(self, status="complete", error=None):
        if self.file is None:
            return
        self.write("session_end", status=status, error=error)
        self.flush()
        self.file.close()
        self.file = None


def load_records(path=DEFAULT_LOG_PATH):
    records = []
    with open(path, "r") as source:
        for line in source:
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except Exception:
        # 断电只可能导致最后一行 JSON 不完整。
                pass
    return records

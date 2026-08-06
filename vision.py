"""主车视觉通信模块。

负责与前向主摄像头通信，接收八字段组合帧，
解析数据并进行超时判断与滤镜锁定控制。
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

class VisionReceiver:
    """接收主摄组合帧并提供格式化数据。"""

    def __init__(self, uart_id=7, baud=115200, timeout_ms=500):
        self.timeout_ms = timeout_ms
        self.uart = None
        try:
            # 兼容 MicroPython UART 接口
            from machine import UART
            self.uart = UART(uart_id)
            self.uart.init(baud)
        except Exception:
            pass
            
        self.rx_buffer = b""
        
        self.target_data = None
        self.hazard_data = None
        
        self.last_frame_ms = 0
        self.last_target_found_ms = 0
        
        self.current_filter_cid = -1
        self.filter_switch_ms = 0
        self.yellow_line_enabled = None
        self.frame_sequence = 0

    def set_target_filter(self, class_id):
        """发送命令让摄像头锁定/过滤特定的类别ID。0 表示不限制。"""
        if self.current_filter_cid == class_id:
            return
            
        self.current_filter_cid = class_id
        self.filter_switch_ms = _ticks_ms()
        # 类别切换后，上一类别的缓存不能作为新一轮搜物结果使用。
        # 新帧抵达前保持 None，避免停车/转向后把刚推过的物体再次锁定。
        self.target_data = None
        
        if self.uart:
            # 兼容 car141929 的过滤指令格式: FF FF 04 <class_id> 00
            cmd = bytearray(b'\xFF\xFF\x04\x00\x00')
            cmd[3] = class_id & 0xFF
            try:
                self.uart.write(cmd)
            except Exception:
                pass

    def set_yellow_line(self, enabled):
        """发送命令开启或关闭摄像头黄线识别。"""
        enabled = bool(enabled)
        if self.yellow_line_enabled == enabled:
            return
        self.yellow_line_enabled = enabled
        if self.uart:
            cmd = bytearray(b'\xFF\xFF\x10\x01\x00') if enabled else bytearray(b'\xFF\xFF\x10\x00\x00')
            try:
                self.uart.write(cmd)
            except Exception:
                pass

    def target_event(self, target, allowed_class_ids, locked_class_id=0):
        """Validate one target frame and return a normalized target event."""
        if not target or not target.get("found", False):
            return None
        try:
            x = float(target["x"])
            y = float(target["y"])
            class_id = int(target["class_id"])
        except (KeyError, TypeError, ValueError):
            return None
        if (
            class_id not in allowed_class_ids
            or x != x
            or y != y
            or x < 0.0
            or x >= 320.0
            or y < 0.0
            or y >= 240.0
        ):
            return None
        if locked_class_id and class_id != locked_class_id:
            return None
        return {"x": x, "y": y, "class_id": class_id}

    def lock_target(self, target, allowed_class_ids):
        """Lock a newly found target and command the camera class filter."""
        event = self.target_event(target, allowed_class_ids)
        if event is None:
            return None
        self.set_target_filter(event["class_id"])
        return event

    def unlock_target(self):
        """Restore all-class object recognition for the next search stage."""
        self.set_target_filter(0)

    def poll(self, now_ms=None):
        """读取串口并更新最新状态。"""
        if now_ms is None:
            now_ms = _ticks_ms()
            
        # 1. 尝试从串口读取所有数据
        if self.uart and self.uart.any():
            try:
                data = self.uart.read()
                if data:
                    self.rx_buffer += data
                    # 防护：防止无定界符时撑爆内存
                    if len(self.rx_buffer) > 1024:
                        self.rx_buffer = b""
            except Exception:
                pass

        # 2. 分割换行符提取完整帧
        lines = []
        while b'\n' in self.rx_buffer and len(lines) < 20:
            idx = self.rx_buffer.find(b'\n')
            line = self.rx_buffer[:idx+1]
            self.rx_buffer = self.rx_buffer[idx+1:]
            lines.append(line)

        # 3. 解析最新有效帧
        frame_received = False
        for line in lines:
            try:
                s_line = line.decode('latin1').strip()
                if not s_line:
                    continue
                parts = s_line.split(',')
                if len(parts) == 8:
                    frame_received = True
                    self.frame_sequence += 1
                    t_found = int(parts[0]) >= 1
                    t_x = float(parts[1])
                    t_y = float(parts[2])
                    t_id = int(parts[3])
                    
                    h_found = int(parts[4]) >= 1
                    h_x = float(parts[5])
                    h_y = float(parts[6])
                    h_type = int(parts[7])
                    
                    # 只有找到目标时，才更新目标的位置信息
                    # 找不到时依赖于超时机制来清空
                    if t_found:
                        if self.target_data is None:
                            self.target_data = {}
                        self.target_data["found"] = True
                        self.target_data["x"] = t_x
                        self.target_data["y"] = t_y
                        self.target_data["class_id"] = t_id
                        self.last_target_found_ms = now_ms
                        
                    # 障碍物信息（无论是 True 还是 False）立即更新
                    if self.hazard_data is None:
                        self.hazard_data = {}
                    self.hazard_data["found"] = h_found
                    self.hazard_data["hazard_found"] = h_found
                    self.hazard_data["hazard_type"] = h_type
                    self.hazard_data["x"] = h_x
                    self.hazard_data["y"] = h_y
                    self.hazard_data["frame_ms"] = now_ms
                    self.hazard_data["frame_sequence"] = self.frame_sequence
            except Exception:
                pass
                
        if frame_received:
            self.last_frame_ms = now_ms
            
        # 4. 超时处理
        # 4.1 整个摄像头掉线/无响应超时
        frame_diff = _ticks_diff(now_ms, self.last_frame_ms)
        if frame_diff > self.timeout_ms:
            self.target_data = None
            self.hazard_data = None
        else:
            # 4.2 目标丢失超时
            target_diff = _ticks_diff(now_ms, self.last_target_found_ms)
            switch_diff = _ticks_diff(now_ms, self.filter_switch_ms)
            
            # 如果刚切换目标锁定（800ms以内），给摄像头一点时间，无视丢帧
            if switch_diff < 800:
                pass
            elif target_diff > self.timeout_ms:
                if self.target_data:
                    self.target_data["found"] = False

    def get_data(self):
        """返回解析好的 (target, hazard) 字典。"""
        return self.target_data, self.hazard_data

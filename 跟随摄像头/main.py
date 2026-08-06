"""OpenART 跟随摄像头入口。

输出紧凑协议：
x_cm,y_cm,theta_deg
0
"""

import sensor
import time
import machine
import pyb

import config
from ir_tracker import IRTracker
from ir_tracker import ReportScheduler
from ir_tracker import format_measurement_line


sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.set_framerate(config.SENSOR_FRAME_RATE)
sensor.set_hmirror(config.H_MIRROR)
sensor.set_vflip(config.V_FLIP)
sensor.set_auto_whitebal(False)
sensor.set_auto_gain(False)
sensor.set_auto_exposure(False, exposure_us=config.EXPOSURE_US)
sensor.skip_frames(time=config.SENSOR_SKIP_MS)

uart = machine.UART(config.UART_ID, config.BAUD)
status_led = pyb.LED(1)
tracker = IRTracker(config.TRACKER_CONFIG)
reporter = ReportScheduler(
    found_interval_ms=config.TX_MIN_INTERVAL_MS,
    lost_interval_ms=config.LOST_REPORT_INTERVAL_MS,
)

# 即使现场只更新了 main.py，仍能使用默认调试设置；避免缺少新配置项而退出。
debug_print_enabled = getattr(config, "DEBUG_PRINT", True)
debug_print_interval_ms = int(getattr(config, "DEBUG_PRINT_INTERVAL_MS", 200))


last_led_ms = time.ticks_ms()
last_debug_print_ms = last_led_ms
if debug_print_enabled:
    print("follow camera started")

while True:
    now_ms = time.ticks_ms()
    img = None

    try:
        img = sensor.snapshot()
        blobs = img.find_blobs(
            [config.IR_THRESHOLD],
            roi=config.ROI,
            pixels_threshold=config.BLOB_PIXELS_MIN,
            area_threshold=config.BLOB_AREA_MIN,
            merge=False,
        )
        candidates = []
        for detected_blob in blobs or ():
            candidates.append(
                {
                    "cx": detected_blob.cx(),
                    "cy": detected_blob.cy(),
                    "pixels": detected_blob.pixels(),
                    "w": detected_blob.w(),
                    "h": detected_blob.h(),
                }
            )
        measurement = tracker.update(candidates)
    except Exception:
        # 任一帧处理异常都立即失效，不继续保留旧目标。
        measurement = tracker.update([])

    found = measurement.get("found", False)
    if reporter.should_send(found, now_ms, time.ticks_diff):
        try:
            uart.write(format_measurement_line(measurement).encode())
        except Exception:
            # UART 故障不改变识别状态；LED 常亮用于现场提示。
            status_led.on()

    if config.DEBUG_DRAW and img is not None:
        img.draw_rectangle(config.ROI, color=127)
        img.draw_cross(
            int(config.REF_CX),
            int(config.REF_CY),
            color=127,
            size=12,
        )
        if found:
            x1 = int(round(measurement["x1"]))
            y1 = int(round(measurement["y1"]))
            x2 = int(round(measurement["x2"]))
            y2 = int(round(measurement["y2"]))
            mid_x = int(round(measurement["mid_x"]))
            mid_y = int(round(measurement["mid_y"]))
            img.draw_circle(x2, y2, 5, color=255)
            if measurement.get("mode") == "pair":
                img.draw_circle(x1, y1, 5, color=255)
                img.draw_line(x1, y1, x2, y2, color=255, thickness=2)
                img.draw_cross(mid_x, mid_y, color=255, size=8)
            img.draw_string(
                4,
                4,
                "%s X%.1f Y%.1f T%.1f"
                % (
                    "P" if measurement.get("mode") == "pair" else "S",
                    measurement["target_x_cm"],
                    measurement["target_y_cm"],
                    measurement["theta_deg"],
                ),
                color=255,
            )
        else:
            img.draw_string(4, 4, "IR LOST", color=127)

    if (
        debug_print_enabled
        and time.ticks_diff(now_ms, last_debug_print_ms)
        >= debug_print_interval_ms
    ):
        if found:
            print(
                "mode=%s target=(%.3f,%.3f) theta=%.2f px=(%.1f,%.1f)"
                % (
                    measurement.get("mode", "unknown"),
                    measurement["target_x_cm"],
                    measurement["target_y_cm"],
                    measurement["theta_deg"],
                    measurement["x2"],
                    measurement["y2"],
                )
            )
        else:
            print("IR LOST")
        last_debug_print_ms = now_ms

    if time.ticks_diff(now_ms, last_led_ms) >= 500:
        status_led.toggle()
        last_led_ms = now_ms

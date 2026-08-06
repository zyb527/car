"""在 OpenMV IDE 中显示双红外灯像素坐标。

使用方法：
1. 将本文件、config.py 和 ir_tracker.py 放在同一目录。
2. 在 OpenMV IDE 中打开并运行本文件。
3. 查看 Frame Buffer 中的 P1/P2 标注，以及串口终端中的 CSV 数据。

本脚本只用于标定，不发送跟随 UART 数据。
"""

import sensor
import time

import config
from ir_tracker import select_best_pair


PRINT_INTERVAL_MS = 100


sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.set_hmirror(config.H_MIRROR)
sensor.set_vflip(config.V_FLIP)
sensor.set_auto_whitebal(False)
sensor.set_auto_gain(False)
sensor.set_auto_exposure(False, exposure_us=config.EXPOSURE_US)
sensor.skip_frames(time=config.SENSOR_SKIP_MS)

print(
    "x1,y1,x2,y2,mid_x,mid_y,distance_px,"
    "line_angle_deg,quality"
)

last_print_ms = time.ticks_ms()

while True:
    now_ms = time.ticks_ms()
    img = sensor.snapshot()
    img.draw_rectangle(config.ROI, color=127)
    img.draw_cross(
        int(round(config.REF_CX)),
        int(round(config.REF_CY)),
        color=127,
        size=12,
    )

    blobs = img.find_blobs(
        [config.IR_THRESHOLD],
        roi=config.ROI,
        pixels_threshold=config.BLOB_PIXELS_MIN,
        area_threshold=config.BLOB_AREA_MIN,
        merge=False,
    )

    candidates = []
    for blob in blobs or ():
        img.draw_rectangle(blob.rect(), color=127)
        candidates.append(
            {
                "cx": blob.cx(),
                "cy": blob.cy(),
                "pixels": blob.pixels(),
                "w": blob.w(),
                "h": blob.h(),
            }
        )

    measurement = select_best_pair(
        candidates,
        config.TRACKER_CONFIG,
        previous=None,
    )

    if measurement is None:
        img.draw_string(2, 2, "IR PAIR NOT FOUND", color=127)
        continue

    x1 = int(measurement["x1"])
    y1 = int(measurement["y1"])
    x2 = int(measurement["x2"])
    y2 = int(measurement["y2"])
    mid_x = float(measurement["mid_x"])
    mid_y = float(measurement["mid_y"])
    mid_x_int = int(round(mid_x))
    mid_y_int = int(round(mid_y))

    img.draw_circle(x1, y1, 6, color=255)
    img.draw_circle(x2, y2, 6, color=255)
    img.draw_cross(x1, y1, color=255, size=8)
    img.draw_cross(x2, y2, color=255, size=8)
    img.draw_line(x1, y1, x2, y2, color=255, thickness=2)
    img.draw_cross(mid_x_int, mid_y_int, color=255, size=10)

    img.draw_string(
        2,
        2,
        "P1(%d,%d) P2(%d,%d)" % (x1, y1, x2, y2),
        color=255,
    )
    img.draw_string(
        2,
        14,
        "M(%.1f,%.1f) D%.1f"
        % (
            mid_x,
            mid_y,
            measurement["distance_px"],
        ),
        color=255,
    )
    img.draw_string(
        2,
        26,
        "A%.2f Q%d"
        % (
            measurement["line_angle_deg"],
            measurement["quality"],
        ),
        color=255,
    )

    if time.ticks_diff(now_ms, last_print_ms) >= PRINT_INTERVAL_MS:
        print(
            "%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%d"
            % (
                x1,
                y1,
                x2,
                y2,
                mid_x,
                mid_y,
                measurement["distance_px"],
                measurement["line_angle_deg"],
                measurement["quality"],
            )
        )
        last_print_ms = now_ms

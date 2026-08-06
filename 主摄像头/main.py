# main.py
import sensor, image, time, machine
from vision_model import Tracker, CLASS_IDS, COLORS
from vision_classic import detect_yellow_border

# ================= 资源配置 =================
SHOW_TEXT_ENABLED = True
LCD_DISPLAY_ENABLED =True

# ================= ID 定义 =================
CLASS_ID_YELLOW = 6

# ================= 硬件初始化 =================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)  # 160x120
sensor.set_brightness(0)
sensor.set_contrast(0)
sensor.set_saturation(0)
sensor.set_auto_whitebal(False, rgb_gain_db=(1.0, 1.0, 1.0))
sensor.set_auto_gain(False, gain_db=10.0)
sensor.set_auto_exposure(False, exposure_us=1800)
sensor.set_framerate(60)
sensor.skip_frames(time=1000)

uart = machine.UART(12, 115200)

lcd = None
disp_img = None
if LCD_DISPLAY_ENABLED:
    try:
        import seekfree
        lcd = seekfree.IPS200(3)
        lcd.full()
        disp_img = image.Image(320, 240, sensor.RGB565)
    except Exception as e:
        print("LCD Init failed:", e)

# ================= 业务变量 =================
yellow_line_enabled = False
enable_car_brick = False
target_class_filter = 0
force_rescan_enabled = True

tracker = Tracker()
clock = time.clock()

# 发送限制
TX_MIN_INTERVAL_MS = 20 # 50Hz max
last_sent_ms = 0

def send_8_fields(t_found, t_x, t_y, t_id, h_found, h_x, h_y, h_type):
    global last_sent_ms
    now = time.ticks_ms()
    if time.ticks_diff(now, last_sent_ms) < TX_MIN_INTERVAL_MS:
        return

    # 根据要求发送8位：t_found, t_x, t_y, t_id, h_found, h_x, h_y, h_type
    # 因为小车的坐标系是 320x240 的倍数，所以从 160x120 转过去需乘以 2.0
    data_str = "%d,%.1f,%.1f,%d,%d,%.1f,%.1f,%d\n" % (
        1 if t_found else 0,
        t_x * 2.0,
        t_y * 2.0,
        t_id,
        1 if h_found else 0,
        h_x * 2.0,
        h_y * 2.0,
        h_type
    )
    try:
        uart.write(data_str.encode())
        last_sent_ms = now
    except Exception as e:
        print("UART Tx Err:", e)

while True:
    clock.tick()
    img = sensor.snapshot()

    # === 1. 串口读取 (非阻塞) ===
    if uart.any():
        try:
            received_data = uart.read()
            if received_data:
                if b'\xFF\xFF\x03\x01\x00' in received_data: force_rescan_enabled = False
                if b'\xFF\xFF\x03\x00\x00' in received_data: force_rescan_enabled = True

                if b'\xFF\xFF\x10\x01\x00' in received_data: yellow_line_enabled = True
                if b'\xFF\xFF\x10\x00\x00' in received_data: yellow_line_enabled = False

                if b'\xFF\xFF\x05\x01\x00' in received_data: enable_car_brick = True
                if b'\xFF\xFF\x05\x00\x00' in received_data: enable_car_brick = False

                # 目标过滤指令解析
                idx = received_data.find(b'\xFF\xFF\x04')
                if idx != -1 and idx + 4 < len(received_data):
                    target_class_filter = received_data[idx + 3]
        except Exception:
            pass

    # === 2. 视觉检测 ===
    # 模型推断（包含平滑追踪逻辑与独立的砖块检测）
    target_tuple, brick_tuple, shown_candidate, brick_blob, status = tracker.process_frame(img, enable_car_brick, target_class_filter)

    t_found, t_x, t_y, t_id = target_tuple
    h_found_b, h_x_b, h_y_b, h_type_b = brick_tuple

    # 传统色块：黄线检测
    h_found_y, h_x_y, h_y_y, yellow_blob = False, 0.0, 0.0, None
    if yellow_line_enabled:
        cx, cy, yellow_blob = detect_yellow_border(img)
        if cx is not None:
            h_found_y, h_x_y, h_y_y = True, cx, cy

    # 合并 hazard (如果有黄线，则优先黄线，否则砖块)
    if h_found_y:
        h_found, h_x, h_y, h_type = True, h_x_y, h_y_y, CLASS_ID_YELLOW
    elif h_found_b:
        h_found, h_x, h_y, h_type = True, h_x_b, h_y_b, h_type_b
    else:
        h_found, h_x, h_y, h_type = False, 0.0, 0.0, 0

    # === 3. 发送数据 ===
    send_8_fields(t_found, t_x, t_y, t_id, h_found, h_x, h_y, h_type)

    # === 4. 绘图与反馈 ===
    # === 4. 绘图与反馈 ===
    # 绘制 Target 候选框
    if shown_candidate:
        color = COLORS.get(tracker.lock_name, (255, 0, 0))
        img.draw_rectangle((shown_candidate['x'], shown_candidate['y'], shown_candidate['w'], shown_candidate['h']), color=color, thickness=2)
        if SHOW_TEXT_ENABLED:
            img.draw_string(shown_candidate['x'], max(0, shown_candidate['y'] - 15), tracker.lock_name, color=color, scale=2)

    # 绘制 Target 平滑中心十字
    if t_found:
        color = COLORS.get(tracker.lock_name, (255, 255, 255))
        img.draw_cross(int(t_x), int(t_y), color=color, size=5, thickness=2)

    # 绘制 Brick (障碍物) 框
    if brick_blob:
        color = COLORS["brick"]
        img.draw_rectangle((brick_blob['x'], brick_blob['y'], brick_blob['w'], brick_blob['h']), color=color, thickness=2)
        if SHOW_TEXT_ENABLED:
            img.draw_string(brick_blob['x'], max(0, brick_blob['y'] - 15), "brick", color=color, scale=2)
        img.draw_cross(int(brick_blob['cx']), int(brick_blob['cy']), color=color, size=5, thickness=2)

    # 绘制 黄线 (边界) 框
    if yellow_blob:
        color = (255, 255, 0)
        img.draw_rectangle(yellow_blob.rect(), color=color, thickness=2)
        if SHOW_TEXT_ENABLED:
            img.draw_string(yellow_blob.x(), max(0, yellow_blob.y() - 15), "yellow", color=color, scale=2)
        img.draw_cross(int(yellow_blob.cx()), int(yellow_blob.cy()), color=color, size=5, thickness=2)

    # 刷新 LCD (屏幕 320x240，摄像头 160x120 居左上，右侧空白区域显示识别 ID 及坐标)
    if LCD_DISPLAY_ENABLED and lcd is not None:
        try:
            if disp_img is None:
                disp_img = image.Image(320, 240, sensor.RGB565)
            disp_img.clear()
            disp_img.draw_image(img, 0, 0)

            # 在 320x240 LCD 右侧空白区域 (x=165~320) 显示详细信息
            disp_img.draw_string(165, 5, "--- TARGET ---", color=(255, 255, 255), scale=1)
            if t_found:
                target_name = tracker.lock_name if tracker.lock_name else "Obj"
                disp_img.draw_string(165, 20, "ID: %d (%s)" % (t_id, target_name), color=(0, 255, 0), scale=1)
                disp_img.draw_string(165, 35, "X : %d" % int(t_x), color=(0, 255, 255), scale=1)
                disp_img.draw_string(165, 50, "Y : %d" % int(t_y), color=(0, 255, 255), scale=1)
            else:
                disp_img.draw_string(165, 20, "ID: None", color=(128, 128, 128), scale=1)
                disp_img.draw_string(165, 35, "X : --", color=(128, 128, 128), scale=1)
                disp_img.draw_string(165, 50, "Y : --", color=(128, 128, 128), scale=1)

            disp_img.draw_string(165, 75, "--- HAZARD ---", color=(255, 255, 255), scale=1)
            if h_found:
                h_name = "yellow" if h_type == CLASS_ID_YELLOW else ("brick" if h_type == 7 else "hazard")
                disp_img.draw_string(165, 90, "ID: %d (%s)" % (h_type, h_name), color=(255, 0, 255), scale=1)
                disp_img.draw_string(165, 105, "X : %d" % int(h_x), color=(0, 255, 255), scale=1)
                disp_img.draw_string(165, 120, "Y : %d" % int(h_y), color=(0, 255, 255), scale=1)
            else:
                disp_img.draw_string(165, 90, "ID: None", color=(128, 128, 128), scale=1)
                disp_img.draw_string(165, 105, "X : --", color=(128, 128, 128), scale=1)
                disp_img.draw_string(165, 120, "Y : --", color=(128, 128, 128), scale=1)

            disp_img.draw_string(165, 145, "--- STATUS ---", color=(255, 255, 255), scale=1)
            disp_img.draw_string(165, 160, "FPS: %.1f" % clock.fps(), color=(255, 255, 255), scale=1)
            disp_img.draw_string(165, 175, "State: %s" % status, color=(255, 255, 255), scale=1)

            lcd.show_image(disp_img, 320, 240, zoom=0)
        except Exception:
            pass

    # 让出 CPU 给后台
    #time.sleep_ms(5)

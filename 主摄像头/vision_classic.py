# vision_classic.py
# 统一的黄色阈值配置
yellow_thresholds = [(47, 98, -30, -3, 65, 92)]

def detect_yellow_border(img):
    """
    长条形黄块检测 (纯色块识别)
    返回: 最大黄线的 cx, cy 和 blob 对象；若未检测到返回 None, None, None
    """
    blobs_found = []
    blobs = img.find_blobs(yellow_thresholds, pixels_threshold=25, area_threshold=25, merge=True)

    for blob in blobs:
        w = blob.w()
        h = blob.h()
        # 限制条件：矩形的一边至少大于另一边的两倍
        if w >= 4 * h or h >= 4 * w:
            blobs_found.append(blob)

    if blobs_found:
        max_blob = max(blobs_found, key=lambda b: b.pixels())
        return max_blob.cx(), max_blob.cy(), max_blob

    return None, None, None


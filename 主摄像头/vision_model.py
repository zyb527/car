# vision_model.py
import tf, image, math

# ================= 配置常量 =================
MODEL_PATH = '/sd/yolo3_iou_smartcar_final_with_post_processing.tflite'
MODEL_SIZE = 112

# 类别定义
LABELS = ['redbag', 'bluebag', 'ball', 'brownbear', 'whitebear', 'brick']

CLASS_IDS = {
    "redbag": 1,
    "bluebag": 2,
    "ball": 3,
    "brownbear": 4,
    "whitebear": 5,
    "brick": 7
}

COLORS = {
    "redbag": (255, 0, 0),     
    "bluebag": (0, 0, 255),    
    "ball": (0, 255, 0),       
    "brownbear": (165, 42, 42),
    "whitebear": (255, 255, 255),
    "brick": (128, 0, 128)     
}

# 优先级：bag(3) > ball(2) > bear(1)
PRIORITY = {"redbag": 3, "bluebag": 3, "ball": 2, "brownbear": 1, "whitebear": 1}
PRIORITY_ORDER = ["redbag", "bluebag", "ball", "brownbear", "whitebear"]

# 手动屏蔽列表：只需修改本行后重新运行主摄像头程序。
# 留空表示不屏蔽；示例：{"ball"}；多个：{"ball", "bluebag", "brick"}。
# 可用名称："redbag"、"bluebag"、"ball"、"brownbear"、"whitebear"、"brick"。
MANUALLY_DISABLED_LABELS = set()  # 示例：{"ball", "bluebag"}

# 锁定与追踪参数
ACQUIRE_SCORE = 0.60
MAINTAIN_SCORE = 0.30
# 蓝沙包在当前模型下置信度偏低，只对该类别放宽获取和保持门槛。
# 其他任务物体继续使用上面的全局门槛。
ACQUIRE_SCORE_BY_CLASS = {"bluebag": 0.55}
MAINTAIN_SCORE_BY_CLASS = {"bluebag": 0.25}
PREEMPT_SCORE = 0.60
PREEMPT_FRAMES = 2
MAX_HOLD_FRAMES = 6
POSITION_ALPHA = 0.65
VELOCITY_ALPHA = 0.50
HOLD_VELOCITY_DECAY = 0.85
ASSOC_MIN_DISTANCE = 35
# 红沙包贴着推杆时可能被模型同时误识别为砖块。若二者中心距离小于此值，
# 忽略该帧的砖块，避免将红沙包坐标作为障碍物发送。
BRICK_RED_BAG_REJECT_DISTANCE_PX = 25

def clamp(value, low, high):
    return max(low, min(value, high))

def is_brick_near_redbag(brick_blob, redbag_blob):
    """Return True when a brick overlaps the red-bag false-positive area."""
    if brick_blob is None or redbag_blob is None:
        return False

    dx = brick_blob['cx'] - redbag_blob['cx']
    dy = brick_blob['cy'] - redbag_blob['cy']
    return math.sqrt(dx * dx + dy * dy) < BRICK_RED_BAG_REJECT_DISTANCE_PX

class Tracker:
    def __init__(self):
        try:
            self.net = tf.load(MODEL_PATH)
            self.model_loaded = True
        except Exception as e:
            print("Model load failed:", e)
            self.net = None
            self.model_loaded = False
        
        self.lock_name = None
        self.smooth_x = 0.0
        self.smooth_y = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.last_w = 0
        self.last_h = 0
        self.miss_frames = 0
        
        self.pending_switch_name = None
        self.pending_switch_frames = 0
        
        self.status = "SEARCH"
        self.shown_candidate = None
        # 当前锁定类别在本帧的原始模型分数，仅供 LCD 诊断显示。
        # None 表示本帧没有输出该类别候选。
        self.target_score = None

    def get_candidates(self, img, enable_car_brick):
        if not self.net:
            return {}, None
            
        scale_ratio = MODEL_SIZE / img.width()
        img_small = img.copy(x_scale=scale_ratio, y_scale=scale_ratio)
        infer_img = image.Image(MODEL_SIZE, MODEL_SIZE, img.format())
        offset_y = (MODEL_SIZE - img_small.height()) // 2
        infer_img.draw_image(img_small, 0, offset_y)
        
        best_by_class = {}
        brick_blob = None
        
        for obj in tf.detect(self.net, infer_img):
            label_idx = int(obj[4])
            score = obj[5]
            if label_idx < 0 or label_idx >= len(LABELS):
                continue
            
            mapped_name = LABELS[label_idx]
            if mapped_name in MANUALLY_DISABLED_LABELS:
                continue
                
            x1 = int((obj[0] * MODEL_SIZE) / scale_ratio)
            y1 = int((obj[1] * MODEL_SIZE - offset_y) / scale_ratio)
            x2 = int((obj[2] * MODEL_SIZE) / scale_ratio)
            y2 = int((obj[3] * MODEL_SIZE - offset_y) / scale_ratio)
            
            x1 = clamp(x1, 0, img.width() - 1)
            y1 = clamp(y1, 0, img.height() - 1)
            x2 = clamp(x2, x1 + 1, img.width())
            y2 = clamp(y2, y1 + 1, img.height())
            
            candidate = {
                'name': mapped_name,
                'score': score,
                'x': x1, 'y': y1,
                'w': x2 - x1, 'h': y2 - y1,
                'cx': (x1 + x2) / 2.0,
                'cy': (y1 + y2) / 2.0,
            }
            
            if mapped_name == "brick":
                if enable_car_brick:
                    if brick_blob is None or score > brick_blob['score']:
                        brick_blob = candidate
            else:
                old = best_by_class.get(mapped_name)
                if old is None or score > old['score']:
                    best_by_class[mapped_name] = candidate

        # 红沙包贴近推杆时，模型可能在同一位置额外给出 brick。此砖块不应
        # 进入 hazard 通道，也不应向小车发送坐标。
        if is_brick_near_redbag(brick_blob, best_by_class.get('redbag')):
            brick_blob = None

        return best_by_class, brick_blob

    def choose_acquire_candidate(self, best_by_class, target_filter_name=None):
        order = [target_filter_name] if target_filter_name else PRIORITY_ORDER
        for name in order:
            candidate = best_by_class.get(name)
            acquire_score = ACQUIRE_SCORE_BY_CLASS.get(name, ACQUIRE_SCORE)
            if candidate is not None and candidate['score'] >= acquire_score:
                return candidate
        return None

    def candidate_is_near_prediction(self, candidate):
        distance = math.sqrt((candidate['cx'] - self.smooth_x) ** 2 +
                             (candidate['cy'] - self.smooth_y) ** 2)
        object_size = max(self.last_w, self.last_h)
        allowed_distance = max(ASSOC_MIN_DISTANCE, object_size * 1.2)
        return distance <= allowed_distance

    def start_lock(self, candidate):
        self.lock_name = candidate['name']
        self.smooth_x = candidate['cx']
        self.smooth_y = candidate['cy']
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.last_w = candidate['w']
        self.last_h = candidate['h']
        self.miss_frames = 0

    def update_from_detection(self, candidate):
        predicted_x = self.smooth_x + self.velocity_x
        predicted_y = self.smooth_y + self.velocity_y
        new_x = POSITION_ALPHA * candidate['cx'] + (1.0 - POSITION_ALPHA) * predicted_x
        new_y = POSITION_ALPHA * candidate['cy'] + (1.0 - POSITION_ALPHA) * predicted_y
        
        measured_vx = new_x - self.smooth_x
        measured_vy = new_y - self.smooth_y
        self.velocity_x = VELOCITY_ALPHA * measured_vx + (1.0 - VELOCITY_ALPHA) * self.velocity_x
        self.velocity_y = VELOCITY_ALPHA * measured_vy + (1.0 - VELOCITY_ALPHA) * self.velocity_y
        self.smooth_x = new_x
        self.smooth_y = new_y
        self.last_w = candidate['w']
        self.last_h = candidate['h']
        self.miss_frames = 0

    def predict_during_hold(self):
        self.smooth_x = clamp(self.smooth_x + self.velocity_x, 0, 159)
        self.smooth_y = clamp(self.smooth_y + self.velocity_y, 0, 119)
        self.velocity_x *= HOLD_VELOCITY_DECAY
        self.velocity_y *= HOLD_VELOCITY_DECAY

    def process_frame(self, img, enable_car_brick=False, target_filter_id=0):
        target_filter_name = None
        for name, cid in CLASS_IDS.items():
            if cid == target_filter_id:
                target_filter_name = name
                break

        # 手动变更屏蔽列表后，即使当前仍锁定该类别，也立即停止发送它的坐标。
        if self.lock_name in MANUALLY_DISABLED_LABELS:
            self.lock_name = None
            self.miss_frames = 0
            self.pending_switch_name = None
            self.pending_switch_frames = 0
                
        if target_filter_id != 0 and target_filter_name is None:
            self.lock_name = None
            
        candidates, brick_blob = self.get_candidates(img, enable_car_brick)
        
        self.shown_candidate = None
        self.target_score = None
        self.status = 'SEARCH'
        
        if target_filter_name and self.lock_name != target_filter_name:
            self.lock_name = None

        if self.lock_name is None:
            new_candidate = self.choose_acquire_candidate(candidates, target_filter_name)
            if new_candidate is not None:
                self.start_lock(new_candidate)
                self.shown_candidate = new_candidate
                self.status = 'ACQUIRE'
        else:
            higher_candidate = None
            if target_filter_name is None:
                for name in PRIORITY_ORDER:
                    candidate = candidates.get(name)
                    if (candidate is not None and 
                        PRIORITY[name] > PRIORITY[self.lock_name] and 
                        candidate['score'] >= PREEMPT_SCORE):
                        higher_candidate = candidate
                        break
            
            if higher_candidate is not None:
                if self.pending_switch_name == higher_candidate['name']:
                    self.pending_switch_frames += 1
                else:
                    self.pending_switch_name = higher_candidate['name']
                    self.pending_switch_frames = 1
            else:
                self.pending_switch_name = None
                self.pending_switch_frames = 0
                
            if self.pending_switch_frames >= PREEMPT_FRAMES:
                self.start_lock(higher_candidate)
                self.shown_candidate = higher_candidate
                self.status = 'SWITCH'
                self.pending_switch_name = None
                self.pending_switch_frames = 0
            else:
                current_candidate = candidates.get(self.lock_name)
                if current_candidate is not None:
                    self.target_score = current_candidate['score']
                maintain_score = MAINTAIN_SCORE_BY_CLASS.get(
                    self.lock_name, MAINTAIN_SCORE
                )
                if (current_candidate is not None and 
                    current_candidate['score'] >= maintain_score and 
                    self.candidate_is_near_prediction(current_candidate)):
                    self.update_from_detection(current_candidate)
                    self.shown_candidate = current_candidate
                    self.status = 'DETECT'
                else:
                    self.miss_frames += 1
                    if self.miss_frames <= MAX_HOLD_FRAMES:
                        self.predict_during_hold()
                        self.status = 'HOLD'
                    else:
                        self.lock_name = None
                        self.miss_frames = 0
                        self.pending_switch_name = None
                        self.pending_switch_frames = 0
                        
                        new_candidate = self.choose_acquire_candidate(candidates, target_filter_name)
                        if new_candidate is not None:
                            self.start_lock(new_candidate)
                            self.shown_candidate = new_candidate
                            self.status = 'REACQUIRE'
                        else:
                            self.status = 'LOST'

        # ACQUIRE/SWITCH/REACQUIRE 的候选已经通过门槛，记录实际分数。
        # HOLD 时则保留上面记录的当前类别原始分数（可能低于保持门槛）。
        if self.shown_candidate is not None:
            self.target_score = self.shown_candidate['score']
                            
        target_found = (self.lock_name is not None)
        target_cx = self.smooth_x
        target_cy = self.smooth_y
        target_class_id = CLASS_IDS.get(self.lock_name, 0) if self.lock_name else 0
        
        brick_found = (brick_blob is not None)
        brick_cx = brick_blob['cx'] if brick_found else 0.0
        brick_cy = brick_blob['cy'] if brick_found else 0.0
        brick_class_id = CLASS_IDS["brick"] if brick_found else 0
        
        return (target_found, target_cx, target_cy, target_class_id), (brick_found, brick_cx, brick_cy, brick_class_id), self.shown_candidate, brick_blob, self.status

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

# 锁定与追踪参数
ACQUIRE_SCORE = 0.60
MAINTAIN_SCORE = 0.30
PREEMPT_SCORE = 0.60
PREEMPT_FRAMES = 2
MAX_HOLD_FRAMES = 5
POSITION_ALPHA = 0.65
VELOCITY_ALPHA = 0.50
HOLD_VELOCITY_DECAY = 0.85
ASSOC_MIN_DISTANCE = 35

def clamp(value, low, high):
    return max(low, min(value, high))

class Tracker:
    def __init__(self):
        try:
            self.net = tf.load(MODEL_PATH)
        except Exception as e:
            print("Model load failed:", e)
            self.net = None
        
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

        return best_by_class, brick_blob

    def choose_acquire_candidate(self, best_by_class, target_filter_name=None):
        order = [target_filter_name] if target_filter_name else PRIORITY_ORDER
        for name in order:
            candidate = best_by_class.get(name)
            if candidate is not None and candidate['score'] >= ACQUIRE_SCORE:
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
                
        if target_filter_id != 0 and target_filter_name is None:
            self.lock_name = None
            
        candidates, brick_blob = self.get_candidates(img, enable_car_brick)
        
        self.shown_candidate = None
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
                if (current_candidate is not None and 
                    current_candidate['score'] >= MAINTAIN_SCORE and 
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
                            
        target_found = (self.lock_name is not None)
        target_cx = self.smooth_x
        target_cy = self.smooth_y
        target_class_id = CLASS_IDS.get(self.lock_name, 0) if self.lock_name else 0
        
        brick_found = (brick_blob is not None)
        brick_cx = brick_blob['cx'] if brick_found else 0.0
        brick_cy = brick_blob['cy'] if brick_found else 0.0
        brick_class_id = CLASS_IDS["brick"] if brick_found else 0
        
        return (target_found, target_cx, target_cy, target_class_id), (brick_found, brick_cx, brick_cy, brick_class_id), self.shown_candidate, brick_blob, self.status

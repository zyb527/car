"""新版盲走与黄线回库逻辑。"""

import math
from control import MotionStep, PIDController, normalize_angle

class GarageState:
    TURN_180 = "TURN_180"
    BLIND_FORWARD = "BLIND_FORWARD"
    BACKWARD_FIND_YELLOW = "BACKWARD_FIND_YELLOW"
    FORWARD_FIND_YELLOW = "FORWARD_FIND_YELLOW"
    STOP_AT_YELLOW = "STOP_AT_YELLOW"
    LATERAL_MOVE = "LATERAL_MOVE"

class GarageController:
    def __init__(self, config):
        self.config = config
        self.heading_pid = PIDController(
            self.config.HEADING_KP,
            0.0,
            self.config.HEADING_KD,
            output_limit=self.config.HEADING_MAX_W,
            integral_limit=1.5,
        )
        self.reset()
        
    def reset(self):
        self.active = False
        self.state = None
        self.start_x = 0.0
        self.elapsed_s = 0.0
        self.vision_receiver = None

    def start(self, vision_receiver):
        self.active = True
        self.state = GarageState.TURN_180
        self.elapsed_s = 0.0
        self.vision_receiver = vision_receiver
        self.heading_pid.reset()
        
    def step(self, pose, hazard, yaw_rate_rad_s, dt):
        if not self.active:
            return MotionStep.stop("not_started")
            
        self.elapsed_s += dt
        current_x, current_y, current_heading = pose
        target_heading = math.pi # 180 degrees
        
        # 仅在 TURN_180 时使用上层的 PID 控制器
        heading_error = normalize_angle(target_heading - current_heading)
        
        if self.state != GarageState.TURN_180:
            w = 0.0
        else:
            w = self.heading_pid.update(heading_error, dt)
        
        if self.state == GarageState.TURN_180:
            # 补偿底层 6.0cm/s 死区，确保能够转动
            if abs(heading_error) > self.config.HEADING_TOLERANCE_RAD:
                if w > 0 and w < 0.75:
                    w = 0.75
                elif w < 0 and w > -0.75:
                    w = -0.75
                    
            if abs(heading_error) < self.config.HEADING_TOLERANCE_RAD and abs(yaw_rate_rad_s) < self.config.HEADING_RATE_TOLERANCE_RAD_S:
                self.start_x = current_x
                if current_x > 270.0:
                    self.state = GarageState.BLIND_FORWARD
                    if self.vision_receiver:
                        self.vision_receiver.set_yellow_line(False)
                elif current_x < 50.0:
                    self.state = GarageState.BACKWARD_FIND_YELLOW
                    if self.vision_receiver:
                        self.vision_receiver.set_yellow_line(True)
                else:
                    self.state = GarageState.FORWARD_FIND_YELLOW
                    if self.vision_receiver:
                        self.vision_receiver.set_yellow_line(True)
                return MotionStep((0.0, 0.0, 0.0), reason="turn_complete")
            return MotionStep((0.0, 0.0, w), reason="turning_180")
            
        if self.state == GarageState.BLIND_FORWARD:
            distance_moved = abs(current_x - self.start_x)
            if distance_moved >= 50.0:
                self.state = GarageState.FORWARD_FIND_YELLOW
                if self.vision_receiver:
                    self.vision_receiver.set_yellow_line(True)
                return MotionStep((0.0, 0.0, w), reason="blind_forward_complete")
                
            vy = self.config.FORWARD_SPEED_CM_S
            return MotionStep((0.0, vy, w), reason="blind_forwarding")
            
        elif self.state == GarageState.BACKWARD_FIND_YELLOW:
            found_yellow = False
            if hazard and getattr(hazard, "get", lambda x, y=None: None)("hazard_type") == 6:
                if hazard.get("y", 0) > 100:
                    found_yellow = True
            
            if found_yellow:
                self.state = GarageState.STOP_AT_YELLOW
                self.elapsed_s = 0.0
                if self.vision_receiver:
                    self.vision_receiver.set_yellow_line(False)
                return MotionStep((0.0, 0.0, w), reason="yellow_found_backward")
                
            distance_moved = abs(current_x - self.start_x)
            if distance_moved >= 50.0:
                self.state = GarageState.FORWARD_FIND_YELLOW
                return MotionStep((0.0, 0.0, w), reason="backward_50_complete")
                
            vy = -self.config.FORWARD_SPEED_CM_S
            return MotionStep((0.0, vy, w), reason="backward_finding_yellow")
            
        elif self.state == GarageState.FORWARD_FIND_YELLOW:
            found_yellow = False
            if hazard and getattr(hazard, "get", lambda x, y=None: None)("hazard_type") == 6:
                if hazard.get("y", 0) > 100:
                    found_yellow = True
            
            if found_yellow:
                self.state = GarageState.STOP_AT_YELLOW
                self.elapsed_s = 0.0
                if self.vision_receiver:
                    self.vision_receiver.set_yellow_line(False)
                return MotionStep((0.0, 0.0, w), reason="yellow_found_forward")
            
            vy = self.config.FORWARD_SPEED_CM_S
            return MotionStep((0.0, vy, w), reason="forward_finding_yellow")
            
        elif self.state == GarageState.STOP_AT_YELLOW:
            if self.elapsed_s >= getattr(self.config, "STOP_WAIT_S", 0.5):
                self.state = GarageState.LATERAL_MOVE
                self.elapsed_s = 0.0
                return MotionStep((0.0, 0.0, w), reason="stop_complete")
            return MotionStep((0.0, 0.0, w), reason="stopping_at_yellow")
            
        elif self.state == GarageState.LATERAL_MOVE:
            # 停止条件：世界坐标 Y < -50 或超时
            if current_y < self.config.LATERAL_MAX_Y_CM or self.elapsed_s >= self.config.LATERAL_TIMEOUT_S:
                self.active = False
                return MotionStep.stop("garage_done", done=True)
                
            vx = -self.config.LATERAL_SPEED_CM_S  # 车体左侧
            return MotionStep((vx, 0.0, w), reason="lateral_moving")
            
        return MotionStep.stop("unknown_garage_state", failed=True)

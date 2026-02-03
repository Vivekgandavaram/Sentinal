Person-Following Robot with NS312 PIR Sensor
Complete working version with all features
"""

import sys
import time
import json
import threading
import socket
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
from functools import lru_cache
import base64
from collections import deque

import cv2
import numpy as np

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except:
    GPIO_AVAILABLE = False
    print("Warning: RPi.GPIO not available")

try:
    from picamera2 import Picamera2
    from picamera2.devices import IMX500
    from picamera2.devices.imx500 import NetworkIntrinsics, postprocess_nanodet_detection
    CAMERA_AVAILABLE = True
except:
    CAMERA_AVAILABLE = False
    print("Warning: Picamera2 not available")

# CONFIG
LEFT_MOTOR_PIN1 = 12
LEFT_MOTOR_PIN2 = 13
RIGHT_MOTOR_PIN1 = 17
RIGHT_MOTOR_PIN2 = 19
NS312_SENSOR_PIN = 27

PWM_FREQUENCY = 1000
INITIAL_SPEED = 60
CENTER_TOLERANCE = 50
TARGET_PERSON_WIDTH = 350
WIDTH_TOLERANCE = 60
MIN_PERSON_WIDTH = 80
DETECTION_THRESHOLD = 0.55

TURN_SPEED_FACTOR = 0.7
APPROACH_SPEED_FACTOR = 0.5
FOLLOW_SPEED_FACTOR = 0.8
SEARCH_SPEED_FACTOR = 0.6

SEARCH_ROTATION_TIME = 12.0
MAX_NO_DETECTION_FRAMES = 10
NS312_CHECK_INTERVAL = 0.5
NS312_PRESENCE_DEBOUNCE = 0.3

SERVER_PORT = 8080

# STATE
class RobotState:
    def __init__(self):
        self.auto_mode = False
        self.speed_level = INITIAL_SPEED
        self.left_speed = 0
        self.right_speed = 0
        self.person_detected = False
        self.person_centered = False
        self.person_distance_ok = False
        self.detection_count = 0
        self.running = True
        self.current_action = "Idle"
        self.is_searching = False
        self.search_start_time = 0
        self.has_completed_search = False
        self.ns312_presence = False
        self.ns312_person_behind = False
        self.robot_stationary_time = 0
        self.robot_was_moving = True

state = RobotState()
state_lock = threading.Lock()

left_motor_pwm1 = None
left_motor_pwm2 = None
right_motor_pwm1 = None
right_motor_pwm2 = None

picam2 = None
imx500 = None
intrinsics = None

detection_lock = threading.Lock()
last_detections = []

preview_lock = threading.Lock()
cached_preview_frame = None
last_preview_time = 0
PREVIEW_CACHE_TIME = 0.15

ns312_last_state = False
ns312_last_transition_time = 0

class MovementTracker:
    def __init__(self, max_history=5):
        self.position_history = deque(maxlen=max_history)
        self.robot_moving = False
    
    def update(self, center_x, is_robot_stationary):
        if is_robot_stationary:
            self.position_history.append((center_x, time.time()))
            self.robot_moving = False
        else:
            self.position_history.clear()
            self.robot_moving = True
    
    def get_movement(self):
        if self.robot_moving or len(self.position_history) < 3:
            return 0
        positions = [p[0] for p in self.position_history]
        time_span = self.position_history[-1][1] - self.position_history[0][1]
        if time_span < 0.3:
            return 0
        return positions[-1] - positions[0]

movement_tracker = MovementTracker()

# NS312 SENSOR
def setup_ns312():
    if not GPIO_AVAILABLE:
        return False
    try:
        GPIO.setup(NS312_SENSOR_PIN, GPIO.IN)
        print(f"✓ NS312 on GPIO{NS312_SENSOR_PIN}")
        return True
    except Exception as e:
        print(f"✗ NS312 failed: {e}")
        return False

def read_ns312_sensor():
    if not GPIO_AVAILABLE:
        return False
    try:
        return GPIO.input(NS312_SENSOR_PIN) == 0
    except:
        return False

def check_ns312_with_debounce():
    global ns312_last_state, ns312_last_transition_time
    current_reading = read_ns312_sensor()
    current_time = time.time()
    
    if current_reading != ns312_last_state:
        if (current_time - ns312_last_transition_time) > NS312_PRESENCE_DEBOUNCE:
            ns312_last_state = current_reading
            ns312_last_transition_time = current_time
            return current_reading
    return ns312_last_state

# MOTORS
def setup_motors():
    global left_motor_pwm1, left_motor_pwm2, right_motor_pwm1, right_motor_pwm2
    
    if not GPIO_AVAILABLE:
        return
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    for pin in [LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2]:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
    
    left_motor_pwm1 = GPIO.PWM(LEFT_MOTOR_PIN1, PWM_FREQUENCY)
    left_motor_pwm2 = GPIO.PWM(LEFT_MOTOR_PIN2, PWM_FREQUENCY)
    right_motor_pwm1 = GPIO.PWM(RIGHT_MOTOR_PIN1, PWM_FREQUENCY)
    right_motor_pwm2 = GPIO.PWM(RIGHT_MOTOR_PIN2, PWM_FREQUENCY)
    
    for pwm in [left_motor_pwm1, left_motor_pwm2, right_motor_pwm1, right_motor_pwm2]:
        pwm.start(0)
    
    print("✓ Motors initialized")

def set_left_motor(speed):
    if not GPIO_AVAILABLE or left_motor_pwm1 is None:
        return
    speed = max(-100, min(100, speed))
    if speed > 0:
        left_motor_pwm1.ChangeDutyCycle(0)
        left_motor_pwm2.ChangeDutyCycle(abs(speed))
    elif speed < 0:
        left_motor_pwm1.ChangeDutyCycle(abs(speed))
        left_motor_pwm2.ChangeDutyCycle(0)
    else:
        left_motor_pwm1.ChangeDutyCycle(0)
        left_motor_pwm2.ChangeDutyCycle(0)

def set_right_motor(speed):
    if not GPIO_AVAILABLE or right_motor_pwm1 is None:
        return
    speed = max(-100, min(100, speed))
    if speed > 0:
        right_motor_pwm1.ChangeDutyCycle(abs(speed))
        right_motor_pwm2.ChangeDutyCycle(0)
    elif speed < 0:
        right_motor_pwm1.ChangeDutyCycle(0)
        right_motor_pwm2.ChangeDutyCycle(abs(speed))
    else:
        right_motor_pwm1.ChangeDutyCycle(0)
        right_motor_pwm2.ChangeDutyCycle(0)

def move_forward(speed=None):
    if speed is None:
        with state_lock:
            speed = state.speed_level
    with state_lock:
        state.left_speed = speed
        state.right_speed = speed
    set_left_motor(speed)
    set_right_motor(speed)

def move_backward(speed=None):
    if speed is None:
        with state_lock:
            speed = state.speed_level
    with state_lock:
        state.left_speed = -speed
        state.right_speed = -speed
    set_left_motor(-speed)
    set_right_motor(-speed)

def rotate_left(speed=None):
    if speed is None:
        with state_lock:
            speed = state.speed_level
    turn_speed = int(speed * TURN_SPEED_FACTOR)
    with state_lock:
        state.left_speed = -turn_speed
        state.right_speed = turn_speed
    set_left_motor(-turn_speed)
    set_right_motor(turn_speed)

def rotate_right(speed=None):
    if speed is None:
        with state_lock:
            speed = state.speed_level
    turn_speed = int(speed * TURN_SPEED_FACTOR)
    with state_lock:
        state.left_speed = turn_speed
        state.right_speed = -turn_speed
    set_left_motor(turn_speed)
    set_right_motor(-turn_speed)

def stop_motors():
    with state_lock:
        state.left_speed = 0
        state.right_speed = 0
    set_left_motor(0)
    set_right_motor(0)

def is_robot_stationary():
    with state_lock:
        return state.left_speed == 0 and state.right_speed == 0

# CAMERA
class Detection:
    def __init__(self, coords, category, conf, metadata):
        self.category = category
        self.conf = conf
        self.box = imx500.convert_inference_coords(coords, metadata, picam2)

def setup_camera():
    global picam2, imx500, intrinsics
    
    if not CAMERA_AVAILABLE:
        return False
    
    try:
        model_path = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"
        imx500 = IMX500(model_path)
        intrinsics = imx500.network_intrinsics
        
        if not intrinsics:
            intrinsics = NetworkIntrinsics()
            intrinsics.task = "object detection"
        
        try:
            with open("/usr/share/imx500-models/coco_labels.txt", "r") as f:
                intrinsics.labels = f.read().splitlines()
        except:
            intrinsics.labels = ["person", "bicycle", "car", "motorcycle", "airplane"]
        
        intrinsics.update_with_defaults()
        
        picam2 = Picamera2(imx500.camera_num)
        config = picam2.create_preview_configuration(
            controls={"FrameRate": intrinsics.inference_rate},
            buffer_count=4
        )
        
        imx500.show_network_fw_progress_bar()
        picam2.start(config, show_preview=False)
        
        if intrinsics.preserve_aspect_ratio:
            imx500.set_auto_aspect_ratio()
        
        print("✓ Camera initialized")
        return True
    except Exception as e:
        print(f"✗ Camera failed: {e}")
        return False

@lru_cache
def get_labels():
    if intrinsics and intrinsics.labels:
        labels = intrinsics.labels
        if intrinsics.ignore_dash_labels:
            labels = [label for label in labels if label and label != "-"]
        return labels
    return ["person"]

def parse_detections(metadata):
    global last_detections
    
    if not CAMERA_AVAILABLE or imx500 is None:
        with detection_lock:
            return list(last_detections)
    
    try:
        np_outputs = imx500.get_outputs(metadata, add_batch=True)
        if np_outputs is None:
            with detection_lock:
                return list(last_detections)
        
        input_w, input_h = imx500.get_input_size()
        
        if intrinsics.postprocess == "nanodet":
            boxes, scores, classes = postprocess_nanodet_detection(
                outputs=np_outputs[0], conf=DETECTION_THRESHOLD, iou_thres=0.65, max_out_dets=10
            )[0]
            from picamera2.devices.imx500.postprocess import scale_boxes
            boxes = scale_boxes(boxes, 1, 1, input_h, input_w, False, False)
        else:
            boxes, scores, classes = np_outputs[0][0], np_outputs[1][0], np_outputs[2][0]
            if intrinsics.bbox_normalization:
                boxes = boxes / input_h
            if intrinsics.bbox_order == "xy":
                boxes = boxes[:, [1, 0, 3, 2]]
            boxes = np.array_split(boxes, 4, axis=1)
            boxes = zip(*boxes)
        
        detections = [Detection(box, cat, score, metadata) 
                     for box, score, cat in zip(boxes, scores, classes)
                     if score > DETECTION_THRESHOLD]
        
        with detection_lock:
            last_detections = detections
        
        return detections
    except Exception as e:
        with detection_lock:
            return list(last_detections)

def find_closest_person(detections, frame_width):
    if not detections:
        return None
    
    labels = get_labels()
    camera_center_x = frame_width // 2
    persons = []
    
    for detection in detections:
        label = labels[int(detection.category)]
        if label.lower() == "person":
            x, y, w, h = detection.box
            if w >= MIN_PERSON_WIDTH:
                persons.append({
                    'x': x, 'y': y, 'w': w, 'h': h,
                    'center_x': x + w // 2, 'center_y': y + h // 2,
                    'area': w * h, 'conf': detection.conf
                })
    
    if not persons:
        return None
    
    person = max(persons, key=lambda p: p['area'])
    x_offset = person['center_x'] - camera_center_x
    width_diff = person['w'] - TARGET_PERSON_WIDTH
    
    return {
        'centered': abs(x_offset) <= CENTER_TOLERANCE,
        'distance_ok': abs(width_diff) <= WIDTH_TOLERANCE,
        'x_offset': x_offset,
        'width': person['w'],
        'width_diff': width_diff,
        'person': person,
        'num_persons': len(persons)
    }

def capture_preview_frame():
    global cached_preview_frame, last_preview_time
    
    if not CAMERA_AVAILABLE or picam2 is None:
        return None
    
    current_time = time.time()
    
    with preview_lock:
        if cached_preview_frame and (current_time - last_preview_time) < PREVIEW_CACHE_TIME:
            return cached_preview_frame
    
    try:
        frame = picam2.capture_array()
        
        with detection_lock:
            detections = list(last_detections)
        
        if len(frame.shape) == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
        elif frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)
        
        labels = get_labels()
        for detection in detections:
            label = labels[int(detection.category)]
            if label.lower() == "person":
                x, y, w, h = detection.box
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, f"Person {detection.conf:.2f}", (x, y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        with state_lock:
            action = state.current_action
            ns312_status = state.ns312_presence
        
        cv2.putText(frame, f"Action: {action}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
        
        h, w = frame.shape[:2]
        ns312_color = (0, 255, 0) if ns312_status else (255, 0, 0)
        ns312_text = "NS312: YES" if ns312_status else "NS312: NO"
        cv2.putText(frame, ns312_text, (10, h - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, ns312_color, 2)
        
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        jpg_as_text = base64.b64encode(buffer).decode('utf-8')
        
        with preview_lock:
            cached_preview_frame = jpg_as_text
            last_preview_time = current_time
        
        return jpg_as_text
    except Exception as e:
        return None

# NS312 MONITOR
def ns312_monitor():
    print("✓ NS312 monitor started")
    
    while state.running:
        try:
            with state_lock:
                auto_mode = state.auto_mode
                person_detected = state.person_detected
                left_speed = state.left_speed
                right_speed = state.right_speed
                robot_was_moving = state.robot_was_moving
            
            robot_moving = (left_speed != 0 or right_speed != 0)
            
            if robot_was_moving and not robot_moving:
                with state_lock:
                    state.robot_stationary_time = time.time()
                    state.robot_was_moving = False
            elif not robot_was_moving and robot_moving:
                with state_lock:
                    state.robot_was_moving = True
            
            presence = check_ns312_with_debounce()
            
            if not auto_mode:
                with state_lock:
                    state.ns312_presence = presence
                    if presence:
                        state.current_action = "Person detected behind (NS312)"
            else:
                if not person_detected:
                    time_since_stop = time.time() - state.robot_stationary_time if not robot_moving else 0
                    if not robot_moving and time_since_stop > 0.2 and presence:
                        with state_lock:
                            state.ns312_person_behind = True
                            state.current_action = "Person behind! Rotating 180"
                    else:
                        with state_lock:
                            state.ns312_person_behind = False
                
                with state_lock:
                    state.ns312_presence = presence
            
            time.sleep(NS312_CHECK_INTERVAL)
        except Exception as e:
            time.sleep(0.5)

# AUTONOMOUS CONTROL
def auto_loop():
    print("✓ Autonomous control started")
    
    if not CAMERA_AVAILABLE or picam2 is None:
        print("✗ Camera unavailable")
        return
    
    config = picam2.camera_config
    frame_width = config["main"]["size"][0]
    no_detection_count = 0
    ns312_rotation_done = False
    
    while state.running:
        try:
            with state_lock:
                if not state.auto_mode:
                    state.current_action = "Manual Mode"
                    state.is_searching = False
                    state.has_completed_search = False
                    ns312_rotation_done = False
                    movement_tracker.position_history.clear()
                    time.sleep(0.1)
                    continue
                speed = state.speed_level
                is_searching = state.is_searching
                ns312_person_behind = state.ns312_person_behind
            
            # Always check camera first
            metadata = picam2.capture_metadata()
            detections = parse_detections(metadata)
            person_info = find_closest_person(detections, frame_width)
            
            # Stop search/rotation if person detected
            if person_info and (is_searching or ns312_person_behind):
                stop_motors()
                ns312_rotation_done = False
                no_detection_count = 0
                with state_lock:
                    state.is_searching = False
                    state.has_completed_search = False
                    state.ns312_person_behind = False
                    state.person_detected = True
                    state.current_action = "Person found! Following"
                time.sleep(0.2)
                continue
            
            # Handle NS312 rotation only if no person detected
            if ns312_person_behind and not ns312_rotation_done and not person_info:
                ns312_rotation_done = True
                with state_lock:
                    state.current_action = "Rotating 180 to face person"
                rotate_right(int(speed * SEARCH_SPEED_FACTOR))
                time.sleep(6.0)
                stop_motors()
                with state_lock:
                    state.current_action = "180 complete - Searching"
                time.sleep(0.5)
                continue
            
            # Continue with normal follow/search
            if is_searching:
                elapsed = time.time() - state.search_start_time
                if elapsed >= SEARCH_ROTATION_TIME:
                    stop_motors()
                    with state_lock:
                        state.is_searching = False
                        state.has_completed_search = True
                        state.current_action = "Search completed"
                else:
                    rotate_right(int(speed * SEARCH_SPEED_FACTOR))
                time.sleep(0.05)
                continue
            
            if person_info:
                no_detection_count = 0
                ns312_rotation_done = False
                with state_lock:
                    state.has_completed_search = False
                    state.ns312_person_behind = False
                    state.person_detected = True
                    state.person_centered = person_info['centered']
                    state.person_distance_ok = person_info['distance_ok']
                    state.detection_count += 1
                
                robot_stationary = is_robot_stationary()
                movement_tracker.update(person_info['person']['center_x'], robot_stationary)
                movement = movement_tracker.get_movement()
                
                if person_info['distance_ok']:
                    if person_info['centered']:
                        if abs(movement) > 20 and robot_stationary:
                            with state_lock:
                                state.current_action = "Following"
                            move_forward(int(speed * FOLLOW_SPEED_FACTOR))
                        else:
                            with state_lock:
                                state.current_action = "Tracking"
                            stop_motors()
                    else:
                        if person_info['x_offset'] > 0:
                            with state_lock:
                                state.current_action = "Turn right"
                            rotate_right(speed)
                        else:
                            with state_lock:
                                state.current_action = "Turn left"
                            rotate_left(speed)
                elif person_info['width_diff'] < -WIDTH_TOLERANCE:
                    if person_info['centered']:
                        with state_lock:
                            state.current_action = "Approach"
                        move_forward(speed)
                    else:
                        turn_factor = min(abs(person_info['x_offset']) / frame_width, 0.5)
                        if person_info['x_offset'] > 0:
                            left_spd = speed
                            right_spd = int(speed * (1 - turn_factor))
                        else:
                            left_spd = int(speed * (1 - turn_factor))
                            right_spd = speed
                        with state_lock:
                            state.left_speed = left_spd
                            state.right_speed = right_spd
                            state.current_action = "Approach + turn"
                        set_left_motor(left_spd)
                        set_right_motor(right_spd)
                else:
                    with state_lock:
                        state.current_action = "Back up"
                    move_backward(int(speed * APPROACH_SPEED_FACTOR))
            else:
                no_detection_count += 1
                
                if no_detection_count >= MAX_NO_DETECTION_FRAMES:
                    with state_lock:
                        has_searched = state.has_completed_search
                    
                    if not has_searched:
                        with state_lock:
                            state.is_searching = True
                            state.search_start_time = time.time()
                            state.has_completed_search = False
                            state.current_action = "Searching 360"
                        rotate_right(int(speed * SEARCH_SPEED_FACTOR))
                    else:
                        stop_motors()
                        with state_lock:
                            state.current_action = "Not found"
                    
                    with state_lock:
                        state.person_detected = False
                        state.person_centered = False
                        state.person_distance_ok = False
                    
                    movement_tracker.position_history.clear()
            
            time.sleep(0.05)
        except Exception as e:
            stop_motors()
            time.sleep(0.1)

# WEB SERVER
HTML = """<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robot Controller with NS312</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { font-family: Arial, sans-serif; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; padding: 20px; min-height: 100vh; }
        .container { max-width: 800px; margin: 0 auto; }
        h1 { text-align: center; margin-bottom: 20px; }
        .card { background: rgba(255,255,255,0.15); padding: 20px; margin: 20px 0; border-radius: 20px; backdrop-filter: blur(10px); }
        .preview { width: 100%; background: black; border-radius: 12px; margin: 15px 0; }
        .action { text-align: center; font-size: 24px; font-weight: bold; color: #4CAF50; margin: 15px 0; }
        .status { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin: 20px 0; }
        .stat { background: rgba(255,255,255,0.1); padding: 15px; border-radius: 12px; text-align: center; }
        .toggle-container { display: flex; justify-content: space-between; align-items: center; }
        .toggle-switch { position: relative; width: 60px; height: 34px; }
        .toggle-switch input { opacity: 0; }
        .slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background: rgba(255,255,255,0.3); border-radius: 34px; transition: 0.4s; }
        .slider:before { position: absolute; content: ""; height: 26px; width: 26px; left: 4px; bottom: 4px; background: white; border-radius: 50%; transition: 0.4s; }
        input:checked + .slider { background: #4CAF50; }
        input:checked + .slider:before { transform: translateX(26px); }
        .indicator { display: inline-block; width: 12px; height: 12px; border-radius: 50%; margin-right: 8px; }
        .indicator.on { background: #4CAF50; }
        .indicator.off { background: #f44336; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Robot Controller with NS312</h1>
        
        <div class="card">
            <h3 style="text-align: center;">Live Preview</h3>
            <img id="preview" class="preview" src="" alt="Camera">
            <div class="action" id="action">Idle</div>
        </div>
        
        <div class="card">
            <div class="toggle-container">
                <h3>Auto Follow Mode</h3>
                <label class="toggle-switch">
                    <input type="checkbox" id="autoToggle" onchange="toggleAuto()">
                    <span class="slider"></span>
                </label>
            </div>
            <div class="status">
                <div class="stat">
                    <div>Person Detected</div>
                    <div id="ps"><span class="indicator off"></span>No</div>
                </div>
                <div class="stat">
                    <div>Centered</div>
                    <div id="cs"><span class="indicator off"></span>No</div>
                </div>
                <div class="stat">
                    <div>Distance OK</div>
                    <div id="ds"><span class="indicator off"></span>No</div>
                </div>
                <div class="stat">
                    <div>NS312</div>
                    <div id="ns"><span class="indicator off"></span>No</div>
                </div>
            </div>
        </div>
    </div>
    
    <script>
        function toggleAuto() {
            fetch('/auto?e=' + (document.getElementById('autoToggle').checked ? '1' : '0'));
        }
        
        function update() {
            fetch('/status').then(r => r.json()).then(d => {
                document.getElementById('ps').innerHTML = d.pd ? '<span class="indicator on"></span>Yes' : '<span class="indicator off"></span>No';
                document.getElementById('cs').innerHTML = d.pc ? '<span class="indicator on"></span>Yes' : '<span class="indicator off"></span>No';
                document.getElementById('ds').innerHTML = d.pd2 ? '<span class="indicator on"></span>Yes' : '<span class="indicator off"></span>No';
                document.getElementById('ns').innerHTML = d.ns ? '<span class="indicator on"></span>Yes' : '<span class="indicator off"></span>No';
                document.getElementById('action').textContent = d.ca;
                document.getElementById('autoToggle').checked = d.am;
            });
        }
        
        function preview() {
            fetch('/preview').then(r => r.json()).then(d => {
                if(d.f) document.getElementById('preview').src = 'data:image/jpeg;base64,' + d.f;
            });
        }
        
        setInterval(update, 500);
        setInterval(preview, 200);
        update();
        preview();
    </script>
</body>
</html>"""

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass
    
    def do_GET(self):
        p = urlparse(self.path).path
        q = parse_qs(urlparse(self.path).query)
        
        if p == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(HTML.encode())
        
        elif p == '/status':
            with state_lock:
                s = {
                    'am': state.auto_mode,
                    'pd': state.person_detected,
                    'pc': state.person_centered,
                    'pd2': state.person_distance_ok,
                    'ca': state.current_action,
                    'ns': state.ns312_presence
                }
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(s).encode())
        
        elif p == '/auto':
            enabled = q.get('e', ['0'])[0] == '1'
            with state_lock:
                state.auto_mode = enabled
                state.is_searching = False
                state.has_completed_search = False
                if not enabled:
                    state.current_action = "Manual Mode"
            if not enabled:
                stop_motors()
            movement_tracker.position_history.clear()
            
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b'OK')
        
        elif p == '/preview':
            frame_data = capture_preview_frame()
            response = {'f': frame_data if frame_data else None}
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(response).encode())
        
        else:
            self.send_response(404)
            self.end_headers()

def run_server():
    server_address = ('', SERVER_PORT)
    httpd = HTTPServer(server_address, Handler)
    print(f"✓ Web server on port {SERVER_PORT}")
    
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        print(f"✓ Access robot at: http://{local_ip}:{SERVER_PORT}")
    except:
        print(f"✓ Access robot at: http://<raspberry-pi-ip>:{SERVER_PORT}")
    
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n✓ Server stopped")

def cleanup():
    print("\nCleaning up...")
    with state_lock:
        state.running = False
    
    stop_motors()
    
    if GPIO_AVAILABLE:
        if left_motor_pwm1:
            left_motor_pwm1.stop()
        if left_motor_pwm2:
            left_motor_pwm2.stop()
        if right_motor_pwm1:
            right_motor_pwm1.stop()
        if right_motor_pwm2:
            right_motor_pwm2.stop()
        GPIO.cleanup()
    
    if picam2 is not None:
        picam2.stop()
    
    print("✓ Cleanup complete")

def main():
    print("=" * 60)
    print("Person-Following Robot with NS312 PIR Sensor")
    print("=" * 60)
    print()
    
    setup_motors()
    ns312_ok = setup_ns312()
    camera_ok = setup_camera()
    
    if not camera_ok:
        print("⚠ Camera not initialized\n")
    if not ns312_ok:
        print("⚠ NS312 not initialized\n")
    
    auto_thread = threading.Thread(target=auto_loop, daemon=True)
    auto_thread.start()
    
    ns312_thread = threading.Thread(target=ns312_monitor, daemon=True)
    ns312_thread.start()
    
    try:
        run_server()
    except KeyboardInterrupt:
        print("\n✓ Shutting down...")
    finally:
        cleanup()

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
SAFE MediaPipe-based teleoperation for SO-101 robot
Maps human arm movements to 6-motor robot control with LIMITED RANGES
"""

import cv2
import mediapipe as mp
import numpy as np
import time
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorNormMode
import json
from pathlib import Path

# ==================== SAFETY CONFIGURATION ====================
PORT = "/dev/tty.usbmodem58FD0170541"
ROBOT_ID = "dabrius"
CAMERA_ID = 0  # Change if using external webcam

# Update rate (Hz) - lower = less power consumption
UPDATE_RATE = 10  # 10 Hz instead of max speed
UPDATE_INTERVAL = 1.0 / UPDATE_RATE

# ⚠️ SAFETY LIMITS: Adjust these based on your robot's safe range
# Start SMALL and increase gradually after testing!
SAFE_LIMITS = {
    "shoulder_pan": {"min": -40, "max": 40},    # Limited horizontal rotation
    "shoulder_lift": {"min": -30, "max": 50},   # Limited vertical movement
    "elbow_flex": {"min": -45, "max": 45},      # Limited elbow bend
    "wrist_flex": {"min": -35, "max": 35},      # Limited wrist pitch
    "wrist_roll": {"min": -40, "max": 40},      # Limited wrist rotation
    "gripper": {"min": -100, "max": 100}        # Full range for gripper (safe)
}

# Dead zone: movements smaller than this are ignored (prevents jitter)
DEAD_ZONE = 3.0  # degrees
PAUSE_THRESHOLD = 15.0  # If arm drops below this angle, pause mode

# Smoothing factor (0 = no smoothing, 0.9 = very smooth but laggy)
SMOOTHING = 0.8  # Higher = safer, slower movements

# Emergency stop: press 'SPACE' to freeze robot
EMERGENCY_STOP = False

# Motor hardware configuration (from calibration)
MOTOR_CONFIG = {
    "shoulder_pan": {"id": 1, "model": "sts3215", "min": 679, "max": 3411},
    "shoulder_lift": {"id": 2, "model": "sts3215", "min": 519, "max": 3042},
    "elbow_flex": {"id": 3, "model": "sts3215", "min": 969, "max": 3196},
    "wrist_flex": {"id": 4, "model": "sts3215", "min": 816, "max": 3168},
    "wrist_roll": {"id": 5, "model": "sts3215", "min": 552, "max": 3795},
    "gripper": {"id": 6, "model": "sts3215", "min": 1999, "max": 3443}
}

# ==================== HELPER FUNCTIONS ====================

def calculate_angle(a, b, c):
    """Calculate angle between 3 points (in degrees)"""
    a = np.array(a)
    b = np.array(b)
    c = np.array(c)
    
    radians = np.arctan2(c[1]-b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
    angle = np.abs(radians * 180.0 / np.pi)
    
    if angle > 180.0:
        angle = 360 - angle
    
    return angle

def calculate_distance(p1, p2):
    """Calculate distance between 2 points"""
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def map_range(value, in_min, in_max, out_min, out_max):
    """Map value from one range to another with clamping"""
    value = np.clip(value, in_min, in_max)
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def apply_safety_limits(value, motor_name):
    """Apply safety limits to motor position"""
    limits = SAFE_LIMITS[motor_name]
    return np.clip(value, limits["min"], limits["max"])

def apply_deadzone(current, target, deadzone):
    """Apply dead zone to prevent jitter"""
    if abs(target - current) < deadzone:
        return current
    return target

def smooth_value(current, target, factor):
    """Smooth transition between values"""
    return current * factor + target * (1 - factor)

# ==================== CALIBRATION LOADER ====================

class CalibrationData:
    def __init__(self, data):
        self.id = data["id"]
        self.drive_mode = data.get("drive_mode", 0)
        self.homing_offset = data.get("homing_offset", 0)
        self.range_min = data["range_min"]
        self.range_max = data["range_max"]

class SimpleCalibration:
    def __init__(self, motors_data):
        self.data = {name: CalibrationData(data) for name, data in motors_data.items()}
    
    def __getitem__(self, key):
        return self.data.get(key)

def load_calibration():
    """Load calibration from file or use defaults"""
    calib_file = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so101_follower" / f"{ROBOT_ID}.json"
    
    if calib_file.exists():
        with open(calib_file) as f:
            return json.load(f)
    else:
        print("⚠️  Using default calibration values")
        return {name: {"id": config["id"], "range_min": config["min"], "range_max": config["max"]}
                for name, config in MOTOR_CONFIG.items()}

# ==================== SAFE ROBOT CONTROLLER ====================

class SafeRobotController:
    def __init__(self):
        calib_data = load_calibration()
        calibration = SimpleCalibration(calib_data)
        
        motors = {
            name: Motor(config["id"], config["model"], MotorNormMode.RANGE_M100_100)
            for name, config in MOTOR_CONFIG.items()
        }
        
        self.bus = FeetechMotorsBus(port=PORT, motors=motors, calibration=calibration)
        self.bus.connect()
        
        # Smoothed positions
        self.positions = {name: 0.0 for name in MOTOR_CONFIG.keys()}
        self.paused = False
        
        # Error tracking
        self.error_count = {name: 0 for name in MOTOR_CONFIG.keys()}
        self.voltage_errors = 0
        
        # Rate limiting
        self.last_update_time = time.time()
        
        print("✓ Robot connected!")
        print("\n⚠️  SAFETY LIMITS ACTIVE:")
        for name, limits in SAFE_LIMITS.items():
            print(f"   {name}: {limits['min']} to {limits['max']}")
        print()
    
    def update(self, target_positions, paused=False, emergency_stop=False):
        """Update robot positions with safety limits, smoothing and dead zone"""
        self.paused = paused or emergency_stop
        
        if self.paused:
            return
        
        # Rate limiting to prevent voltage issues
        current_time = time.time()
        if current_time - self.last_update_time < UPDATE_INTERVAL:
            return  # Skip this update
        self.last_update_time = current_time
        
        for name, target in target_positions.items():
            # Apply safety limits FIRST
            target = apply_safety_limits(target, name)
            
            # Apply dead zone
            target = apply_deadzone(self.positions[name], target, DEAD_ZONE)
            
            # Smooth transition (extra safe)
            self.positions[name] = smooth_value(self.positions[name], target, SMOOTHING)
            
            # Send to robot with error handling
            try:
                self.bus.write("Goal_Position", name, self.positions[name])
                self.error_count[name] = 0  # Reset error count on success
            except RuntimeError as e:
                self.error_count[name] += 1
                
                if "voltage error" in str(e).lower():
                    self.voltage_errors += 1
                    if self.voltage_errors % 10 == 1:  # Print every 10 errors
                        print(f"⚠️  Voltage error on {name}. Check power supply!")
                        print(f"   Total voltage errors: {self.voltage_errors}")
                    
                    # Skip this motor for now to reduce load
                    if self.voltage_errors > 50:
                        print(f"❌ Too many voltage errors ({self.voltage_errors}). Stopping.")
                        raise
                else:
                    # Other error - print and continue
                    if self.error_count[name] == 1:
                        print(f"⚠️  Error on {name}: {e}")
                
                # If motor keeps failing, skip it
                if self.error_count[name] > 5:
                    print(f"⚠️  Motor {name} disabled due to repeated errors")
                    continue
    
    def go_to_neutral(self):
        """Slowly move to neutral position (all motors at 0)"""
        print("🏠 Moving to neutral position...")
        target = {name: 0.0 for name in MOTOR_CONFIG.keys()}
        
        # Slow movement to neutral
        for _ in range(50):
            self.update(target, paused=False, emergency_stop=False)
            time.sleep(0.05)
        
        print("✓ Neutral position reached")
    
    def disconnect(self):
        self.bus.disconnect()
        print("✓ Robot disconnected")

# ==================== TELEOPERATION SYSTEM ====================

class TeleoperationSystem:
    def __init__(self):
        # MediaPipe setup
        self.mp_pose = mp.solutions.pose
        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils
        
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.7, min_tracking_confidence=0.7)
        self.hands = self.mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)
        
        # Robot controller
        self.robot = SafeRobotController()
        
        # Video capture
        self.cap = cv2.VideoCapture(CAMERA_ID)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        # Emergency stop flag
        self.emergency_stop = False
        
        print("✓ Teleoperation system ready!")
        print("\n📋 CONTROLS:")
        print("  • Right arm horizontal = Robot neutral")
        print("  • Open hand = Gripper open")
        print("  • Closed hand = Gripper closed")
        print("  • Drop arm down = PAUSE mode")
        print("  • SPACE = EMERGENCY STOP")
        print("  • 'n' = Go to neutral position")
        print("  • 'q' = Quit\n")
        print("⚠️  Start with SMALL movements to test limits!\n")
    
    def process_frame(self):
        """Process one frame and return motor positions"""
        ret, frame = self.cap.read()
        if not ret:
            return None, None, None
        
        # Flip for mirror view
        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process pose and hands
        pose_results = self.pose.process(rgb_frame)
        hand_results = self.hands.process(rgb_frame)
        
        # Initialize target positions
        targets = {name: 0.0 for name in MOTOR_CONFIG.keys()}
        paused = False
        
        h, w, _ = frame.shape
        
        # ============ POSE DETECTION ============
        if pose_results.pose_landmarks:
            landmarks = pose_results.pose_landmarks.landmark
            
            # Get key points (right arm)
            shoulder = landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value]
            elbow = landmarks[self.mp_pose.PoseLandmark.RIGHT_ELBOW.value]
            wrist = landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST.value]
            hip = landmarks[self.mp_pose.PoseLandmark.RIGHT_HIP.value]
            
            # Convert to pixel coordinates
            shoulder_pt = [shoulder.x * w, shoulder.y * h]
            elbow_pt = [elbow.x * w, elbow.y * h]
            wrist_pt = [wrist.x * w, wrist.y * h]
            hip_pt = [hip.x * w, hip.y * h]
            
            # Calculate angles
            shoulder_angle = calculate_angle(hip_pt, shoulder_pt, elbow_pt)
            elbow_angle = calculate_angle(shoulder_pt, elbow_pt, wrist_pt)
            
            # Check pause condition (arm dropped)
            if shoulder_angle < PAUSE_THRESHOLD:
                paused = True
                cv2.putText(frame, "[ PAUSED ]", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 165, 255), 3)
            else:
                # Map angles to robot motors with REDUCED ranges
                # Shoulder Pan (horizontal rotation): use shoulder x-position
                targets["shoulder_pan"] = map_range(shoulder.x, 0.35, 0.65, 
                                                   SAFE_LIMITS["shoulder_pan"]["min"], 
                                                   SAFE_LIMITS["shoulder_pan"]["max"])
                
                # Shoulder Lift: arm elevation
                targets["shoulder_lift"] = map_range(shoulder_angle, 40, 140, 
                                                    SAFE_LIMITS["shoulder_lift"]["min"], 
                                                    SAFE_LIMITS["shoulder_lift"]["max"])
                
                # Elbow Flex
                targets["elbow_flex"] = map_range(elbow_angle, 60, 160, 
                                                 SAFE_LIMITS["elbow_flex"]["min"], 
                                                 SAFE_LIMITS["elbow_flex"]["max"])
            
            # Draw pose
            self.mp_draw.draw_landmarks(frame, pose_results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)
        
        # ============ HAND DETECTION ============
        if hand_results.multi_hand_landmarks and not paused:
            for hand_landmarks in hand_results.multi_hand_landmarks:
                # Get key points
                wrist = hand_landmarks.landmark[0]
                thumb_tip = hand_landmarks.landmark[4]
                index_tip = hand_landmarks.landmark[8]
                middle_tip = hand_landmarks.landmark[12]
                
                # Wrist Flex: hand pitch (y-position relative to elbow)
                if pose_results.pose_landmarks:
                    elbow = pose_results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_ELBOW.value]
                    wrist_angle = (wrist.y - elbow.y) * 150  # Reduced scale
                    targets["wrist_flex"] = apply_safety_limits(wrist_angle, "wrist_flex")
                
                # Wrist Roll: hand rotation (x-position of fingers)
                roll_angle = (middle_tip.x - wrist.x) * 150  # Reduced scale
                targets["wrist_roll"] = apply_safety_limits(roll_angle, "wrist_roll")
                
                # Gripper: distance between thumb and index
                thumb_pt = [thumb_tip.x * w, thumb_tip.y * h]
                index_pt = [index_tip.x * w, index_tip.y * h]
                distance = calculate_distance(thumb_pt, index_pt)
                
                # Map distance to gripper position (close = small distance)
                targets["gripper"] = map_range(distance, 20, 100, 
                                              SAFE_LIMITS["gripper"]["min"], 
                                              SAFE_LIMITS["gripper"]["max"])
                
                # Draw hand
                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
        
        return frame, targets, paused
    
    def draw_ui(self, frame, targets, paused):
        """Draw UI overlay with motor positions and safety status"""
        h, w, _ = frame.shape
        
        # Semi-transparent panel
        overlay = frame.copy()
        cv2.rectangle(overlay, (w - 380, 0), (w, 470), (0, 0, 0), -1)
        frame = cv2.addWeighted(frame, 0.7, overlay, 0.3, 0)
        
        # Title
        cv2.putText(frame, "SAFE TELEOPERATION", (w - 370, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Emergency stop indicator
        if self.emergency_stop:
            cv2.putText(frame, "!!! EMERGENCY STOP !!!", (50, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
        
        # Voltage error indicator
        if self.robot.voltage_errors > 0:
            error_color = (0, 100, 255) if self.robot.voltage_errors < 20 else (0, 0, 255)
            cv2.putText(frame, f"Voltage Errors: {self.robot.voltage_errors}", 
                       (w - 370, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, error_color, 2)
        
        # Motor positions
        y_offset = 80
        for i, (name, value) in enumerate(targets.items()):
            actual_value = self.robot.positions[name]
            limits = SAFE_LIMITS[name]
            
            # Color coding
            if self.emergency_stop or paused:
                color = (100, 100, 100)
            elif abs(actual_value - limits["max"]) < 5 or abs(actual_value - limits["min"]) < 5:
                color = (0, 165, 255)  # Orange when near limits
            else:
                color = (0, 255, 0)  # Green when safe
            
            # Motor name
            cv2.putText(frame, name.upper(), (w - 370, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            
            # Limit indicators
            cv2.putText(frame, f"[{limits['min']}, {limits['max']}]", 
                       (w - 370, y_offset + 15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)
            
            # Position bar
            bar_x = w - 370
            bar_y = y_offset + 25
            bar_w = 330
            bar_h = 12
            
            # Background bar
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (50, 50, 50), -1)
            
            # Safe range indicators (vertical lines)
            safe_min_x = int(bar_x + (limits["min"] + 100) * bar_w / 200)
            safe_max_x = int(bar_x + (limits["max"] + 100) * bar_w / 200)
            cv2.line(frame, (safe_min_x, bar_y), (safe_min_x, bar_y + bar_h), (255, 100, 0), 2)
            cv2.line(frame, (safe_max_x, bar_y), (safe_max_x, bar_y + bar_h), (255, 100, 0), 2)
            
            # Position indicator
            pos_x = int(bar_x + (actual_value + 100) * bar_w / 200)
            cv2.circle(frame, (pos_x, bar_y + bar_h // 2), 6, color, -1)
            
            # Value text
            cv2.putText(frame, f"{actual_value:.1f}", (w - 70, y_offset + 15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            
            y_offset += 55
        
        return frame
    
    def run(self):
        """Main teleoperation loop"""
        try:
            while True:
                frame, targets, paused = self.process_frame()
                
                if frame is None:
                    break
                
                # Update robot
                if targets:
                    self.robot.update(targets, paused, self.emergency_stop)
                
                # Draw UI
                frame = self.draw_ui(frame, targets or {}, paused)
                
                # Show frame
                cv2.imshow("Safe MediaPipe Robot Teleoperation", frame)
                
                # Handle keyboard
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    break
                elif key == ord(' '):  # SPACE = emergency stop
                    self.emergency_stop = not self.emergency_stop
                    if self.emergency_stop:
                        print("🛑 EMERGENCY STOP ACTIVATED")
                    else:
                        print("✓ Emergency stop released")
                elif key == ord('n'):  # N = neutral position
                    self.robot.go_to_neutral()
        
        except KeyboardInterrupt:
            print("\n⚠️  Interrupted")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        print("\n🏠 Returning to neutral position before shutdown...")
        self.robot.go_to_neutral()
        
        self.cap.release()
        cv2.destroyAllWindows()
        self.robot.disconnect()
        print("✓ Cleanup complete")

# ==================== MAIN ====================

if __name__ == "__main__":
    print("🤖 Starting SAFE MediaPipe Teleoperation System...\n")
    print("⚠️  IMPORTANT SAFETY NOTES:")
    print("   1. Start with SMALL movements")
    print("   2. Test each motor individually")
    print("   3. Keep EMERGENCY STOP (SPACE) ready")
    print("   4. Increase limits in SAFE_LIMITS if needed")
    print("   5. CHECK POWER SUPPLY VOLTAGE (voltage errors can occur)")
    print(f"   6. Update rate: {UPDATE_RATE} Hz (reduce if voltage issues)\n")
    
    try:
        system = TeleoperationSystem()
        system.run()
    except RuntimeError as e:
        if "voltage error" in str(e).lower():
            print("\n" + "="*60)
            print("❌ STOPPED DUE TO VOLTAGE ERRORS")
            print("="*60)
            print("\n💡 SOLUTIONS:")
            print("   1. Check power supply/battery voltage")
            print("   2. Reduce UPDATE_RATE (currently {})".format(UPDATE_RATE))
            print("   3. Increase SMOOTHING (currently {})".format(SMOOTHING))
            print("   4. Use fewer motors simultaneously")
            print("   5. Check for mechanical obstructions")
            print("\n⚡ Power supply should be 12V with sufficient current")
        else:
            raise
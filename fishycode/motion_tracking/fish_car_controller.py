import cv2
import numpy as np
import serial
import time
import math
from hokuyolx import HokuyoLX
print(">>> FILE LOADED fish_car_controller.py <<<", flush=True)

# ============================================
# CONFIGURATION
# ============================================

# Serial Setup
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 19200

# Camera Setup
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# HSV Color Bounds for Fish Detection [0, 170, 25]
LOWER_HSV = np.array([23, 98, 162])
UPPER_HSV = np.array([179, 255, 255])

# LIDAR Setup
LIDAR_IP = "192.168.0.10"
LIDAR_PORT = 10940
LIDAR_STOP_DISTANCE = 0.7  # meters
LIDAR_FRONT_ANGLE_DEG = 30  # +/- degrees around center

# Virtual Bounding Box Setup
BOUNDING_BOX_RADIUS = 5.0  # meters - maximum distance from starting position
BOUNDING_BOX_WARNING_RADIUS = 4.0  # meters - start slowing down
ENABLE_BOUNDING_BOX = True

# Movement Smoothing
SMOOTHING_FACTOR = 0.05  # 0.0 = no smoothing, 1.0 = maximum smoothing
MIN_FISH_DETECTION_AREA = 100  # minimum contour area to consider valid

# Loop Timing
LOOP_DELAY = 0.02  # seconds between iterations

# ============================================
# GLOBAL STATE
# ============================================

class CarState:
    def __init__(self):
        self.start_x = 0.0
        self.start_y = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.heading = 0.0  # radians
        self.distance_traveled = 0.0
        self.last_command_time = time.time()
        self.smoothed_fish_x = FRAME_WIDTH // 2
        self.smoothed_fish_y = FRAME_HEIGHT // 2
        self.lidar_obstacle_detected = False
        self.boundary_violation = False
        
    def update_position(self, speed, steering_angle, dt):
        """Estimate position based on movement (dead reckoning)"""
        # Simple kinematic model
        if abs(speed) > 0.01:
            if abs(steering_angle - 90) < 5:  # Straight
                dx = speed * dt * math.cos(self.heading)
                dy = speed * dt * math.sin(self.heading)
            else:
                # Turning - simplified arc model
                turn_rate = (steering_angle - 90) * 0.01  # radians per second
                self.heading += turn_rate * dt
                dx = speed * dt * math.cos(self.heading)
                dy = speed * dt * math.sin(self.heading)
            
            self.current_x += dx
            self.current_y += dy
            self.distance_traveled += math.sqrt(dx*dx + dy*dy)
    
    def get_distance_from_start(self):
        """Calculate distance from starting position"""
        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        return math.sqrt(dx*dx + dy*dy)
    
    def is_within_boundary(self):
        """Check if car is within allowed boundary"""
        if not ENABLE_BOUNDING_BOX:
            return True
        return self.get_distance_from_start() < BOUNDING_BOX_RADIUS
    
    def get_boundary_factor(self):
        """Get factor (0-1) for how close to boundary (1 = at edge)"""
        if not ENABLE_BOUNDING_BOX:
            return 0.0
        distance = self.get_distance_from_start()
        if distance < BOUNDING_BOX_WARNING_RADIUS:
            return 0.0
        elif distance >= BOUNDING_BOX_RADIUS:
            return 1.0
        else:
            # Linear interpolation between warning and max radius
            return (distance - BOUNDING_BOX_WARNING_RADIUS) / (BOUNDING_BOX_RADIUS - BOUNDING_BOX_WARNING_RADIUS)

# ============================================
# INITIALIZATION
# ============================================

def initialize_systems():
    """Initialize all hardware systems"""
    print("Initializing Fish Car Controller...")
    
    # Initialize serial connection to Arduino
    try:
        arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Allow Arduino to reset
        print(f"✓ Serial connected to Arduino on {SERIAL_PORT}")
    except serial.SerialException as e:
        print(f"✗ Error: Could not connect to Arduino: {e}")
        return None, None, None
    
    # Initialize camera
    camera = cv2.VideoCapture(CAMERA_INDEX)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    
    if not camera.isOpened():
        print("✗ Error: Camera not detected")
        arduino.close()
        return None, None, None
    
    actual_width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"✓ Camera opened with resolution {actual_width}x{actual_height}")
    
    # Initialize LIDAR
    try:
        lidar = HokuyoLX(addr=(LIDAR_IP, LIDAR_PORT))
        # Test scan
        ts, scan = lidar.get_dist()
        if scan is not None:
            print(f"✓ LIDAR connected at {LIDAR_IP}:{LIDAR_PORT} ({len(scan)} beams)")
        else:
            print("⚠ LIDAR connected but no scan data received")
    except Exception as e:
        print(f"✗ Error: Could not connect to LIDAR: {e}")
        camera.release()
        arduino.close()
        return None, None, None
        # print("lidar disabled - continuing without it")
       # lidar = None
    
    print("\n=== System Ready ===")
    print(f"Bounding Box: {'ENABLED' if ENABLE_BOUNDING_BOX else 'DISABLED'}")
    if ENABLE_BOUNDING_BOX:
        print(f"  Max Radius: {BOUNDING_BOX_RADIUS}m")
        print(f"  Warning Radius: {BOUNDING_BOX_WARNING_RADIUS}m")
    print(f"LIDAR Stop Distance: {LIDAR_STOP_DISTANCE}m")
    print("Press 'q' to quit\n")
    
    return arduino, camera, lidar

# ============================================
# LIDAR PROCESSING
# ============================================

def check_lidar_obstacle(lidar):
    """Check if there's an obstacle in front using LIDAR"""
    try:
        ts, scan = lidar.get_dist()
        
        if scan is None or len(scan) == 0:
            return False
        
        n = len(scan)
        deg_per_beam = 270.0 / n  # UST-10LX has 270° FOV
        beams_each_side = int(LIDAR_FRONT_ANGLE_DEG / deg_per_beam)
        
        mid = n // 2
        start = max(0, mid - beams_each_side)
        end = min(n, mid + beams_each_side)
        
        front_beams = scan[start:end]
        
        # Convert mm to meters and filter valid readings
        valid_distances = [d / 1000.0 for d in front_beams if 0 < d < 30000]
        
        if not valid_distances:
            return False
        
        closest = min(valid_distances)
        return closest < LIDAR_STOP_DISTANCE
        
    except Exception as e:
        print(f"LIDAR error: {e}")
        return False

# ============================================
# FISH TRACKING
# ============================================

def detect_fish(frame, state):
    """Detect fish position in frame with smoothing"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    fish_detected = False
    raw_x, raw_y = state.smoothed_fish_x, state.smoothed_fish_y
    
    if contours:
        # Find largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        if area > MIN_FISH_DETECTION_AREA:
            x, y, w, h = cv2.boundingRect(largest_contour)
            raw_x = x + w // 2
            raw_y = y + h // 2
            fish_detected = True
            
            # Draw debug overlays
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (raw_x, raw_y), 5, (0, 0, 255), -1)
    
    # Apply smoothing
    if fish_detected:
        state.smoothed_fish_x = int(SMOOTHING_FACTOR * state.smoothed_fish_x + (1 - SMOOTHING_FACTOR) * raw_x)
        state.smoothed_fish_y = int(SMOOTHING_FACTOR * state.smoothed_fish_y + (1 - SMOOTHING_FACTOR) * raw_y)
    
    # Draw smoothed position
    cv2.circle(frame, (state.smoothed_fish_x, state.smoothed_fish_y), 8, (255, 0, 255), 2)
    
    return frame, mask, fish_detected

# ============================================
# COMMAND GENERATION
# ============================================

def generate_movement_command(state):
    """Generate movement command based on fish position and safety constraints"""
    
    # Check safety constraints
    obstacle_ahead = state.lidar_obstacle_detected
    boundary_factor = state.get_boundary_factor()
    at_boundary = not state.is_within_boundary()
    
    # Get fish position
    fish_x = state.smoothed_fish_x
    fish_y = state.smoothed_fish_y
    
    # Determine grid position (3x3 grid)
    col = 0 if fish_x < FRAME_WIDTH / 3 else (1 if fish_x < 2 * FRAME_WIDTH / 3 else 2)
    row = 0 if fish_y < FRAME_HEIGHT / 3 else (1 if fish_y < 2 * FRAME_HEIGHT / 3 else 2)
    
    # Base command format: "x,y,speed_factor,boundary_factor,obstacle"
    # Speed factor: 0.0-1.0 (will be applied on Arduino side)
    # Boundary factor: 0.0-1.0 (0=safe, 1=at boundary)
    # Obstacle: 0=clear, 1=obstacle detected
    
    speed_factor = 1.0
    
    # Reduce speed near boundary
    if boundary_factor > 0:
        speed_factor *= (1.0 - boundary_factor * 0.7)  # Reduce up to 70% at boundary
    
    # Stop if at boundary or obstacle ahead
    if at_boundary or obstacle_ahead:
        fish_x = FRAME_WIDTH // 2
        fish_y = FRAME_HEIGHT // 2
        speed_factor = 0.0
    
    obstacle_flag = 1 if obstacle_ahead else 0
    
    command = f"{fish_x},{fish_y},{speed_factor:.2f},{boundary_factor:.2f},{obstacle_flag}\n"
    
    return command, col, row

# ============================================
# MAIN LOOP
# ============================================

def main():
    # Initialize systems
    arduino, camera, lidar = initialize_systems()
    if arduino is None:
        return
    
    # Initialize state
    state = CarState()
    
    # Main control loop
    frame_count = 0
    last_time = time.time()
    
    try:
        while True:
            loop_start = time.time()
            
            # Read camera frame
            ret, frame = camera.read()
            if not ret:
                print("Error: Failed to grab frame")
                break
            
            # Detect fish
            frame, mask, fish_detected = detect_fish(frame, state)
            
            # Check LIDAR for obstacles
            # state.lidar_obstacle_detected = check_lidar_obstacle(lidar)
            if lidar is not None:
               state.lidar_obstacle_detected = check_lidar_obstacle(lidar)
            else:
               state.lidar_obstacle_detected = False
            
            # Generate and send command
            command, col, row = generate_movement_command(state)
            print("SENDING TO ARDUINO: ", command.strip())

            try:
                arduino.write(command.encode())
                state.last_command_time = time.time()
            except serial.SerialException as e:
                print(f"Error writing to Arduino: {e}")
            
            # Update position estimate (simplified)
            dt = time.time() - last_time
            last_time = time.time()
            # Note: Actual position tracking would need odometry or GPS
            
            # Display status
            frame_count += 1
            if frame_count % 30 == 0:  # Every ~1 second
                distance = state.get_distance_from_start()
                boundary_pct = state.get_boundary_factor() * 100
                status = "FISH DETECTED" if fish_detected else "NO FISH"
                obstacle_status = "OBSTACLE!" if state.lidar_obstacle_detected else "Clear"
                print(f"{status} | Grid: ({col},{row}) | Boundary: {boundary_pct:.0f}% | LIDAR: {obstacle_status} | Dist: {distance:.2f}m")
            
            # Draw status overlay
            status_color = (0, 255, 0)
            if state.lidar_obstacle_detected:
                status_color = (0, 0, 255)
                cv2.putText(frame, "OBSTACLE DETECTED", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            elif not state.is_within_boundary():
                status_color = (0, 165, 255)
                cv2.putText(frame, "BOUNDARY REACHED", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            elif state.get_boundary_factor() > 0.5:
                status_color = (0, 255, 255)
                cv2.putText(frame, "APPROACHING BOUNDARY", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            
            # Draw grid overlay
            cv2.line(frame, (FRAME_WIDTH // 3, 0), (FRAME_WIDTH // 3, FRAME_HEIGHT), (100, 100, 100), 1)
            cv2.line(frame, (2 * FRAME_WIDTH // 3, 0), (2 * FRAME_WIDTH // 3, FRAME_HEIGHT), (100, 100, 100), 1)
            cv2.line(frame, (0, FRAME_HEIGHT // 3), (FRAME_WIDTH, FRAME_HEIGHT // 3), (100, 100, 100), 1)
            cv2.line(frame, (0, 2 * FRAME_HEIGHT // 3), (FRAME_WIDTH, 2 * FRAME_HEIGHT // 3), (100, 100, 100), 1)
            
            # Show frames
            cv2.imshow("Fish Car - Tracking", frame)
            cv2.imshow("Fish Car - Mask", mask)
            
            # Check for quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\nShutdown requested by user")
                break
            
            # Maintain loop timing
            elapsed = time.time() - loop_start
            if elapsed < LOOP_DELAY:
                time.sleep(LOOP_DELAY - elapsed)
    
    except KeyboardInterrupt:
        print("\nShutdown requested (Ctrl+C)")
    
    finally:
        # Cleanup
        print("Cleaning up...")
        # Send stop command
        try:
            arduino.write(b"320,240,0.0,0.0,0\n")  # Center position, zero speed
            time.sleep(0.1)
        except:
            pass
        
        camera.release()
        cv2.destroyAllWindows()
        arduino.close()
        print("Program ended.")

if __name__ == "__main__":
    main()
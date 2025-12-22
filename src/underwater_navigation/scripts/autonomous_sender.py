import cv2
import numpy as np
import serial
import time

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# --- PWM CONSTANTS ---
HEAVE_NEUTRAL = 1500
SURGE_NEUTRAL = 1530
SWAY_NEUTRAL = 1500

# Safety Limits
PWM_MIN = 1300
PWM_MAX = 1700

# --- VISION SETTINGS ---
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CENTER_X = FRAME_WIDTH // 2
CENTER_Y = FRAME_HEIGHT // 2

# Underwater "Black" (Adjust V-Max if detection fails)
LOWER_BLACK = np.array([0, 0, 0])      
UPPER_BLACK = np.array([180, 255, 60]) 

# Thresholds
MIN_DETECT_AREA = 1000
STOP_DISTANCE_AREA = 60000

# Gains (Tuning)
K_SWAY = 0.3
K_HEAVE = 0.3
K_SURGE = 0.02

# --- HELPER FUNCTIONS ---
def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

def create_packet(heave, surge, sway):
    # Order: BL, FR, SL, SR, BR, FL, SwL, SwR
    values = [heave, heave, surge, surge, heave, heave, sway, sway]
    # This string goes to Serial
    return "/".join(map(str, values)) + "\n"

# --- MAIN ---
def main():
    print("--- AUTONOMOUS LOGGING STARTED ---")
    print(f"Video: {FRAME_WIDTH}x{FRAME_HEIGHT} | Serial: {SERIAL_PORT}")
    
    # 1. Connect Serial
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
    except Exception as e:
        print(f"CRITICAL ERROR: Serial Port Failed -> {e}")
        return

    # 2. Connect Camera
    cap = cv2.VideoCapture(0)
    cap.set(3, FRAME_WIDTH)
    cap.set(4, FRAME_HEIGHT)
    
    if not cap.isOpened():
        print("CRITICAL ERROR: Camera not found.")
        return

    # 3. Arming
    neutral_packet = create_packet(HEAVE_NEUTRAL, SURGE_NEUTRAL, SWAY_NEUTRAL)
    print("Arming ESCs... (Sending Neutral Once)")
    ser.write(neutral_packet.encode())
    time.sleep(2)
    
    print("-" * 60)
    print(f"{'STATUS':<15} | {'POS (X,Y)':<12} | {'AREA':<8} | {'SWAY':<6} {'HEAVE':<6} {'SURGE':<6}")
    print("-" * 60)

    try:
        while True:
            ret, frame = cap.read()
            if not ret: break

            # Image Processing
            blurred = cv2.GaussianBlur(frame, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, LOWER_BLACK, UPPER_BLACK)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Default: Neutral
            pwm_heave = HEAVE_NEUTRAL
            pwm_surge = SURGE_NEUTRAL
            pwm_sway = SWAY_NEUTRAL
            
            target_status = "SEARCHING"
            pos_str = "---"
            area_str = "---"

            if contours:
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)

                if area > MIN_DETECT_AREA:
                    target_status = "TRACKING"
                    
                    # Centroid
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                    else:
                        cx, cy = CENTER_X, CENTER_Y
                    
                    pos_str = f"{cx},{cy}"
                    area_str = f"{int(area)}"

                    # --- Control Logic ---
                    
                    # Sway (Horizontal)
                    error_x = cx - CENTER_X
                    pwm_sway = int(SWAY_NEUTRAL + (error_x * K_SWAY))

                    # Heave (Vertical)
                    # If cy > Center (Object is low), Error is (+).
                    # We need to dive. Assuming 1300 is dive/down.
                    error_y = cy - CENTER_Y
                    pwm_heave = int(HEAVE_NEUTRAL - (error_y * K_HEAVE))

                    # Surge (Distance)
                    if area < STOP_DISTANCE_AREA:
                        size_error = STOP_DISTANCE_AREA - area
                        pwm_surge = int(SURGE_NEUTRAL + (size_error * K_SURGE))
                        pwm_surge = min(1650, pwm_surge) # Safety Cap
                    else:
                        pwm_surge = SURGE_NEUTRAL
                        target_status = "HOLDING"

            # Clamp
            pwm_heave = clamp(pwm_heave, PWM_MIN, PWM_MAX)
            pwm_surge = clamp(pwm_surge, PWM_MIN, PWM_MAX)
            pwm_sway = clamp(pwm_sway, PWM_MIN, PWM_MAX)

            # --- SERIAL WRITE (Clean String Only) ---
            packet = create_packet(pwm_heave, pwm_surge, pwm_sway)
            ser.write(packet.encode())

            # --- CONSOLE LOG (Rich Info) ---
            # Using \r to overwrite the line keeps terminal clean
            log_msg = f"{target_status:<15} | {pos_str:<12} | {area_str:<8} | {pwm_sway:<6} {pwm_heave:<6} {pwm_surge:<6}"
            print(log_msg)

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        ser.write(neutral_packet.encode())
        time.sleep(2)
        ser.close()
        cap.release()
        print("Disconnected.")

if __name__ == "__main__":
    main()
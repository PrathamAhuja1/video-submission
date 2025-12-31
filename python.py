#!/usr/bin/env python3
import serial
import time

SERIAL_PORT = '/dev/ttyS4'
BAUD_RATE = 115200

# Matches the C code's channel count
CHANNELS = 8 

def send_pwm(ser, val):
    # Create array of values
    vals = [val] * CHANNELS
    
    # Format: "1500/1500/.../1500/" (Note the trailing slash!)
    msg = ""
    for v in vals:
        msg += f"{v}/"
    msg += "\n"
    
    ser.write(msg.encode('utf-8'))
    ser.flush() # Vital!
    print(f"Sent: {msg.strip()}")

def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"Opened {SERIAL_PORT}")

    try:
        # 1. ARMING
        print("Arming (Sending 1500)...")
        end_t = time.time() + 3
        while time.time() < end_t:
            send_pwm(ser, 1500)
            time.sleep(0.05)

        # 2. MOVE
        print("Moving (Sending 1600)...")
        end_t = time.time() + 3
        while time.time() < end_t:
            send_pwm(ser, 1600)
            time.sleep(0.05)

        # 3. STOP
        send_pwm(ser, 1500)
        print("Done.")

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

if __name__ == "__main__":
    main()
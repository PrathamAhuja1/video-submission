#!/usr/bin/env python3
"""
Serial Connection Test Tool
Tests the RP2040 connection using the same format as check.py
Use this to verify your serial connection works before launching ROS
"""

import serial
import time
import sys

SERIAL_PORT = '/dev/ttyS4'
BAUD_RATE = 115200
CHANNELS = 6

def send_pwm(ser, values):
    """Send PWM with correct format: 1500/1500/.../1500/\n"""
    msg = ""
    for v in values:
        msg += f"{v}/"
    msg += "\n"
    
    ser.write(msg.encode('utf-8'))
    ser.flush()
    print(f"Sent: {msg.strip()}")

def read_sensor_data(ser):
    """Read and display sensor data from RP2040"""
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            parts = line.split('/')
            if len(parts) == 3:
                try:
                    pressure = float(parts[0])
                    temp = float(parts[1])
                    depth = float(parts[2])
                    print(f"  Sensor: Depth={depth:.3f}m, P={pressure:.1f}mbar, T={temp:.1f}°C")
                except ValueError:
                    print(f"  Parse error: {line}")
            else:
                print(f"  Data: {line}")

def main():
    print("="*70)
    print("🔧 AUV Serial Connection Test")
    print("="*70)
    print(f"Port: {SERIAL_PORT}")
    print(f"Baud: {BAUD_RATE}")
    print(f"Channels: {CHANNELS}")
    print("="*70)
    
    try:
        # Open serial connection
        ser = serial.Serial(
            SERIAL_PORT, 
            BAUD_RATE, 
            timeout=0.1,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        print(f"✅ Opened {SERIAL_PORT}")
        time.sleep(2)  # Wait for RP2040 init

        # Test sequence
        tests = [
            ("Arming (Neutral)", [1500] * CHANNELS, 2.0),
            ("Test Forward", [1500, 1500, 1600, 1600, 1500, 1500], 2.0),
            ("Test Heave Up", [1400, 1400, 1500, 1500, 1400, 1400], 2.0),
            ("Test Heave Down", [1600, 1600, 1500, 1500, 1600, 1600], 2.0),
            ("Neutral Stop", [1500] * CHANNELS, 2.0),
        ]
        
        for test_name, pwm_values, duration in tests:
            print(f"\n🔵 {test_name}")
            print(f"   PWM: {pwm_values}")
            
            end_time = time.time() + duration
            while time.time() < end_time:
                send_pwm(ser, pwm_values)
                read_sensor_data(ser)
                time.sleep(0.05)
        
        # Final neutral
        print("\n🛑 Sending final neutral...")
        for _ in range(10):
            send_pwm(ser, [1500] * CHANNELS)
            time.sleep(0.05)
        
        print("\n✅ Test complete!")
        
    except serial.SerialException as e:
        print(f"❌ Serial error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n⚠️ Interrupted by user")
    finally:
        if 'ser' in locals() and ser.is_open:
            # Emergency stop
            for _ in range(5):
                send_pwm(ser, [1500] * CHANNELS)
                time.sleep(0.05)
            ser.close()
            print("Serial closed")

if __name__ == "__main__":
    main()
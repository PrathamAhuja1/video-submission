#!/usr/bin/env python3
"""
Serial Bridge: ROS2 /PWM6 → RP2040 UART
FIXED: Proper format matching check.py, better error handling
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, Float32
import serial
import time


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyS4')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('depth_offset', 0.4)
        
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.depth_offset = self.get_parameter('depth_offset').value
        
        # Neutral PWM values for 6 thrusters
        self.NEUTRAL_PWM = [1500, 1500, 1500, 1500, 1500, 1500]
        
        # PWM range (relaxed validation)
        self.PWM_MIN = 1300
        self.PWM_MAX = 1700
        
        # Connect to RP2040
        self.serial = None
        self.connect_serial(port, baud)
        
        # Subscribe to PWM commands
        self.pwm_sub = self.create_subscription(
            UInt16MultiArray,
            '/PWM6',
            self.pwm_callback,
            10
        )
        
        # Publish depth telemetry
        self.depth_pub = self.create_publisher(Float32, '/depth', 10)
        
        # Statistics
        self.pwm_tx_count = 0
        self.depth_rx_count = 0
        self.last_depth_time = time.time()
        self.last_pwm_time = time.time()
        
        # Timer for reading sensor data (50Hz for responsiveness)
        self.create_timer(0.02, self.read_telemetry)
        
        # Timer for connection health check (1Hz)
        self.create_timer(1.0, self.check_connection)
        
        # Send neutral PWM periodically if no commands (safety)
        self.create_timer(0.5, self.safety_heartbeat)
        
        self.get_logger().info('='*70)
        self.get_logger().info('🔗 Serial Bridge Active (Fixed Format)')
        self.get_logger().info(f'   Port: {port} @ {baud} baud')
        self.get_logger().info(f'   Format: "1500/1500/.../1500/\\n" (trailing slash)')
        self.get_logger().info(f'   Depth Offset: {self.depth_offset:+.3f}m')
        self.get_logger().info('='*70)
    
    def connect_serial(self, port, baud):
        """Establish serial connection with proper settings"""
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
                time.sleep(0.5)
            
            self.serial = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=0.1,
                write_timeout=0.1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            
            # Clear buffers
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
            time.sleep(2)  # Wait for RP2040 initialization
            
            # Send initial neutral command
            self.send_pwm_direct(self.NEUTRAL_PWM)
            
            self.get_logger().info(f'✅ Connected to RP2040 on {port}')
            return True
            
        except serial.SerialException as e:
            self.get_logger().error(f'❌ Failed to open {port}: {e}')
            return False
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')
            return False
    
    def send_pwm_direct(self, pwm_values):
        """Send PWM with correct format matching check.py"""
        if not self.serial or not self.serial.is_open:
            return False
        
        try:
            # CRITICAL: Format must match check.py exactly
            # "1500/1500/1500/1500/1500/1500/\n" (trailing slash!)
            msg = ""
            for val in pwm_values:
                msg += f"{int(val)}/"
            msg += "\n"
            
            self.serial.write(msg.encode('utf-8'))
            self.serial.flush()  # CRITICAL: Ensure data is sent immediately
            
            return True
            
        except serial.SerialTimeoutException:
            self.get_logger().error('Serial write timeout')
            return False
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
            self.serial.close()
            self.serial = None
            return False
        except Exception as e:
            self.get_logger().error(f'Unexpected write error: {e}')
            return False
    
    def pwm_callback(self, msg: UInt16MultiArray):
        """Receive PWM commands from ROS2"""
        if not self.serial or not self.serial.is_open:
            self.get_logger().warn('Serial not connected, cannot send PWM')
            return
        
        # Validate we have exactly 6 values
        if len(msg.data) != 6:
            self.get_logger().warn(
                f'Invalid PWM: expected 6 values, got {len(msg.data)}'
            )
            return
        
        # Validate PWM range (with tolerance)
        pwm_values = []
        for i, val in enumerate(msg.data):
            # Allow slightly outside range but clamp
            clamped = max(self.PWM_MIN, min(self.PWM_MAX, int(val)))
            if abs(clamped - val) > 10:  # Warn if clamping significantly
                self.get_logger().warn(
                    f'PWM[{i}] clamped: {val} → {clamped}'
                )
            pwm_values.append(clamped)
        
        # Send PWM
        if self.send_pwm_direct(pwm_values):
            self.pwm_tx_count += 1
            self.last_pwm_time = time.time()
            
            # Periodic logging
            if self.pwm_tx_count % 100 == 0:
                self.get_logger().info(
                    f'TX[{self.pwm_tx_count}]: {pwm_values}',
                    throttle_duration_sec=5.0
                )
    
    def read_telemetry(self):
        """Read sensor data from RP2040"""
        if not self.serial or not self.serial.is_open:
            return
        
        try:
            # Read all available lines
            while self.serial.in_waiting > 0:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                
                if not line:
                    continue
                
                # Parse: pressure/temperature/depth
                parts = line.split('/')
                
                if len(parts) == 3:
                    try:
                        pressure = float(parts[0])
                        temperature = float(parts[1])
                        depth_raw = float(parts[2])
                        
                        # Apply calibration offset
                        depth = depth_raw + self.depth_offset
                        
                        # Sanity checks
                        if -0.5 <= depth <= 2.0 and 800.0 <= pressure <= 1200.0:
                            # Publish depth
                            msg = Float32()
                            msg.data = depth
                            self.depth_pub.publish(msg)
                            
                            self.depth_rx_count += 1
                            self.last_depth_time = time.time()
                            
                            # Periodic logging
                            if self.depth_rx_count % 100 == 0:
                                self.get_logger().info(
                                    f'RX[{self.depth_rx_count}]: '
                                    f'Depth={depth:.3f}m, P={pressure:.1f}mbar, '
                                    f'T={temperature:.1f}°C',
                                    throttle_duration_sec=5.0
                                )
                        else:
                            if self.depth_rx_count % 50 == 0:
                                self.get_logger().warn(
                                    f'Sensor out of range: depth={depth:.2f}m, '
                                    f'pressure={pressure:.1f}mbar'
                                )
                            
                    except ValueError:
                        self.get_logger().warn(f'Parse error: {line}')
                        
        except serial.SerialException as e:
            self.get_logger().error(f'Serial read error: {e}')
            self.serial.close()
            self.serial = None
        except Exception as e:
            self.get_logger().error(f'Unexpected read error: {e}')
    
    def check_connection(self):
        """Monitor connection health"""
        if not self.serial or not self.serial.is_open:
            self.get_logger().warn('Serial disconnected, attempting reconnect...')
            port = self.get_parameter('serial_port').value
            baud = self.get_parameter('baud_rate').value
            self.connect_serial(port, baud)
            return
        
        # Check sensor data freshness
        time_since_depth = time.time() - self.last_depth_time
        if time_since_depth > 5.0:
            self.get_logger().warn(
                f'No depth data for {time_since_depth:.1f}s'
            )
        
        # Check command freshness
        time_since_pwm = time.time() - self.last_pwm_time
        if time_since_pwm > 2.0:
            self.get_logger().debug('No PWM commands recently')
    
    def safety_heartbeat(self):
        """Send neutral PWM if no commands received (safety feature)"""
        time_since_pwm = time.time() - self.last_pwm_time
        
        # If no commands for 1 second, send neutral
        if time_since_pwm > 1.0:
            if self.serial and self.serial.is_open:
                self.send_pwm_direct(self.NEUTRAL_PWM)
    
    def destroy_node(self):
        """Cleanup: send neutral PWM and close connection"""
        self.get_logger().info('Shutting down serial bridge...')
        
        if self.serial and self.serial.is_open:
            # Send neutral PWM multiple times for safety
            for _ in range(5):
                self.send_pwm_direct(self.NEUTRAL_PWM)
                time.sleep(0.05)
            
            self.serial.close()
            self.get_logger().info('Serial connection closed safely')
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SerialBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Fatal error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
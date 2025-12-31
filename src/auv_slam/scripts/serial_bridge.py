#!/usr/bin/env python3
"""
Serial Bridge: ROS2 /PWM6 → RP2040 UART
Sends 6-channel PWM commands from ROS2 to the microcontroller
Receives depth telemetry from RP2040
UPDATED: Added depth_offset parameter for calibration
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
        self.declare_parameter('depth_offset', 0.4)  # NEW: Calibration offset
        
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.depth_offset = self.get_parameter('depth_offset').value
        
        # Neutral PWM values for 6 thrusters (matches microcontroller)
        self.NEUTRAL_PWM = [1500, 1500, 1500, 1500, 1500, 1500]
        
        # Connect to RP2040
        self.serial = None
        self.connection_attempts = 0
        self.max_connection_attempts = 5
        
        self.connect_serial(port, baud)
        
        # Subscribe to PWM commands
        self.pwm_sub = self.create_subscription(
            UInt16MultiArray,
            '/PWM6',
            self.pwm_callback,
            10
        )
        
        # Publish depth telemetry from RP2040
        self.depth_pub = self.create_publisher(Float32, '/depth', 10)
        
        # Statistics
        self.pwm_tx_count = 0
        self.depth_rx_count = 0
        self.last_depth_time = time.time()
        
        # Timer for reading sensor data (20Hz)
        self.create_timer(0.05, self.read_telemetry)
        
        # Timer for connection health check (1Hz)
        self.create_timer(1.0, self.check_connection)
        
        self.get_logger().info('='*70)
        self.get_logger().info('🔗 Serial Bridge Active (6-Channel)')
        self.get_logger().info(f'   Depth Offset: {self.depth_offset:+.3f}m')
        self.get_logger().info('='*70)
    
    def connect_serial(self, port, baud):
        """Establish serial connection with retry logic"""
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
            
            self.serial = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=0.1,
                write_timeout=0.1
            )
            time.sleep(2)  # Wait for RP2040 to initialize
            
            # Send initial neutral command
            self.send_neutral_pwm()
            
            self.get_logger().info(f'✅ Connected to RP2040 on {port} @ {baud} baud')
            self.connection_attempts = 0
            return True
            
        except serial.SerialException as e:
            self.connection_attempts += 1
            self.get_logger().error(
                f'❌ Failed to open {port} (attempt {self.connection_attempts}): {e}'
            )
            
            if self.connection_attempts >= self.max_connection_attempts:
                self.get_logger().fatal('Max connection attempts reached. Exiting.')
                raise
            
            return False
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')
            raise
    
    def send_neutral_pwm(self):
        """Send neutral PWM command to stop all thrusters"""
        if not self.serial or not self.serial.is_open:
            return
        
        try:
            pwm_str = '/'.join(str(v) for v in self.NEUTRAL_PWM) + '\n'
            self.serial.write(pwm_str.encode('utf-8'))
            self.serial.flush()
            self.get_logger().info('Sent neutral PWM: ' + pwm_str.strip())
        except Exception as e:
            self.get_logger().error(f'Failed to send neutral PWM: {e}')
    
    def pwm_callback(self, msg: UInt16MultiArray):
        """
        Send PWM values to RP2040
        Format: 1500/1500/1500/1500/1500/1500\n (6 values)
        """
        if not self.serial or not self.serial.is_open:
            self.get_logger().warn('Serial not connected, cannot send PWM')
            return
        
        # Validate we have exactly 6 values
        if len(msg.data) != 6:
            self.get_logger().warn(
                f'Invalid PWM data: expected 6 values, got {len(msg.data)}'
            )
            return
        
        # Validate PWM range (1300-1700)
        for i, val in enumerate(msg.data):
            if val < 1300 or val > 1700:
                self.get_logger().warn(
                    f'PWM[{i}] out of range: {val} (valid: 1300-1700)'
                )
                return
        
        # Format: "val0/val1/.../val5\n"
        pwm_str = '/'.join(str(int(v)) for v in msg.data) + '\n'
        
        try:
            self.serial.write(pwm_str.encode('utf-8'))
            self.serial.flush()
            
            self.pwm_tx_count += 1
            
            # Periodic logging
            if self.pwm_tx_count % 100 == 0:
                self.get_logger().debug(
                    f'TX[{self.pwm_tx_count}]: {pwm_str.strip()}',
                    throttle_duration_sec=5.0
                )
                
        except serial.SerialTimeoutException:
            self.get_logger().error('Serial write timeout')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
            self.serial.close()
            self.serial = None
        except Exception as e:
            self.get_logger().error(f'Unexpected write error: {e}')
    
    def read_telemetry(self):
        """
        Read sensor data from RP2040
        Expected format: "pressure/temperature/depth\n"
        Example: "1013.25/22.5/0.42\n"
        UPDATED: Apply depth offset calibration
        """
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
                        
                        # APPLY CALIBRATION OFFSET
                        depth = depth_raw + self.depth_offset
                        
                        # Sanity checks (with calibrated depth)
                        if -0.2 <= depth <= 2.0 and 800.0 <= pressure <= 1200.0:
                            # Publish corrected depth
                            msg = Float32()
                            msg.data = depth
                            self.depth_pub.publish(msg)
                            
                            self.depth_rx_count += 1
                            self.last_depth_time = time.time()
                            
                            # Periodic logging
                            if self.depth_rx_count % 100 == 0:
                                self.get_logger().info(
                                    f'RX[{self.depth_rx_count}]: Depth={depth:.3f}m '
                                    f'(raw={depth_raw:.3f}m + offset={self.depth_offset:.3f}m), '
                                    f'P={pressure:.1f}mbar, T={temperature:.1f}°C',
                                    throttle_duration_sec=5.0
                                )
                        else:
                            # Only warn occasionally to avoid spam
                            if self.depth_rx_count % 50 == 0:
                                self.get_logger().warn(
                                    f'Sensor values out of range: depth={depth:.2f}m '
                                    f'(raw={depth_raw:.2f}m), pressure={pressure:.1f}mbar',
                                    throttle_duration_sec=2.5
                                )
                            
                    except ValueError as e:
                        self.get_logger().warn(f'Failed to parse sensor data: {line}')
                else:
                    self.get_logger().debug(f'Unexpected format: {line}')
                    
        except serial.SerialException as e:
            self.get_logger().error(f'Serial read error: {e}')
            self.serial.close()
            self.serial = None
        except Exception as e:
            self.get_logger().error(f'Unexpected read error: {e}')
    
    def check_connection(self):
        """Periodic connection health check"""
        if not self.serial or not self.serial.is_open:
            self.get_logger().warn('Serial connection lost, attempting reconnect...')
            port = self.get_parameter('serial_port').value
            baud = self.get_parameter('baud_rate').value
            self.connect_serial(port, baud)
            return
        
        # Check if we're receiving depth data
        time_since_depth = time.time() - self.last_depth_time
        if time_since_depth > 5.0:
            self.get_logger().warn(
                f'No depth data for {time_since_depth:.1f}s - check sensor connection'
            )
    
    def destroy_node(self):
        """Cleanup: send neutral PWM and close connection"""
        self.get_logger().info('Shutting down serial bridge...')
        
        if self.serial and self.serial.is_open:
            # Send neutral PWM 3 times for safety
            for _ in range(3):
                self.send_neutral_pwm()
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
#!/usr/bin/env python3
"""
Serial Bridge: ROS2 /PWM8 → RP2040 UART
Sends PWM commands from ROS2 to the microcontroller
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
        
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        
        # Connect to RP2040
        try:
            self.serial = serial.Serial(port, baud, timeout=1)
            time.sleep(2)  # Wait for connection
            self.get_logger().info(f'✅ Connected to RP2040 on {port}')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to open {port}: {e}')
            raise
        
        # Subscribe to PWM commands
        self.pwm_sub = self.create_subscription(
            UInt16MultiArray,
            '/PWM8',
            self.pwm_callback,
            10
        )
        
        # Publish depth telemetry from RP2040
        self.depth_pub = self.create_publisher(Float32, '/depth', 10)
        
        # Timer for reading sensor data
        self.create_timer(0.05, self.read_telemetry)  # 20Hz
        
        self.get_logger().info('🔗 Serial bridge active')
    
    def pwm_callback(self, msg: UInt16MultiArray):
        """
        Send PWM values to RP2040
        Format: 1500/1500/1530/1500/1500/1480\n (6 values)
        """
        if len(msg.data) < 6:
            self.get_logger().warn(f'Not enough PWM values: {len(msg.data)}')
            return
        
        # Take first 6 values (RP2040 only has 6 ESCs)
        pwm_str = '/'.join(str(int(v)) for v in msg.data[:6]) + '\n'
        
        try:
            self.serial.write(pwm_str.encode('utf-8'))
            self.get_logger().debug(f'Sent: {pwm_str.strip()}')
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')
    
    def read_telemetry(self):
        """Read sensor data from RP2040"""
        try:
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                
                if line:
                    # Parse: pressure/temperature/depth
                    parts = line.split('/')
                    if len(parts) == 3:
                        try:
                            depth = float(parts[2])
                            msg = Float32()
                            msg.data = depth
                            self.depth_pub.publish(msg)
                        except ValueError:
                            pass
        except Exception as e:
            self.get_logger().error(f'Serial read error: {e}')
    
    def destroy_node(self):
        """Cleanup"""
        if hasattr(self, 'serial') and self.serial.is_open:
            # Send neutral PWM before closing
            neutral = '1500/1500/1530/1500/1500/1480\n'
            self.serial.write(neutral.encode('utf-8'))
            time.sleep(0.1)
            self.serial.close()
            self.get_logger().info('Serial connection closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
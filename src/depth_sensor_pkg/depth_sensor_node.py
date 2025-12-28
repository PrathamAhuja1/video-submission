#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time


class DepthSensorNode(Node):
    def __init__(self):
        super().__init__('depth_sensor_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyS4')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('publish_rate', 10.0)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Create publisher
        self.depth_publisher = self.create_publisher(Float32, '/depth', 10)
        
        # Initialize serial connection
        self.serial_connection = None
        self.connect_serial()
        
        # Create timer for reading serial data
        timer_period = 1.0 / self.publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self.read_and_publish)
        
        self.get_logger().info(f'Depth sensor node started. Reading from {self.serial_port}')
        self.get_logger().info(f'Publishing depth data to /depth topic at {self.publish_rate} Hz')
    
    def connect_serial(self):
        """Initialize serial connection with error handling"""
        try:
            self.serial_connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            time.sleep(2)  # Wait for connection to stabilize
            self.get_logger().info(f'Serial connection established on {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {self.serial_port}: {str(e)}')
            self.serial_connection = None
        except Exception as e:
            self.get_logger().error(f'Unexpected error opening serial port: {str(e)}')
            self.serial_connection = None
    
    def read_and_publish(self):
        """Read data from serial port and publish depth"""
        if self.serial_connection is None or not self.serial_connection.is_open:
            self.get_logger().warn('Serial connection not available. Attempting to reconnect...')
            self.connect_serial()
            return
        
        try:
            if self.serial_connection.in_waiting > 0:
                # Read line from serial port
                line = self.serial_connection.readline().decode('utf-8', errors='ignore').strip()
                
                if line:
                    # Parse the data: format is "Pressure/temperature/depth"
                    parts = line.split('/')
                    
                    if len(parts) == 3:
                        try:
                            depth_value = float(parts[2])
                            
                            # Create and publish message
                            msg = Float32()
                            msg.data = depth_value
                            self.depth_publisher.publish(msg)
                            
                            self.get_logger().debug(f'Published depth: {depth_value}')
                        except ValueError:
                            self.get_logger().warn(f'Invalid depth value: {parts[2]}')
                    else:
                        self.get_logger().warn(f'Unexpected data format: {line}')
        
        except serial.SerialException as e:
            self.get_logger().error(f'Serial read error: {str(e)}')
            self.serial_connection.close()
            self.serial_connection = None
        except Exception as e:
            self.get_logger().error(f'Unexpected error reading serial data: {str(e)}')
    
    def destroy_node(self):
        """Clean up resources"""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            self.get_logger().info('Serial connection closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DepthSensorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
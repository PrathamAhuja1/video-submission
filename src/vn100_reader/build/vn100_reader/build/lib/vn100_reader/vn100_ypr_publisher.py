#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from vectornav import Sensor, Registers
from geometry_msgs.msg import Vector3


class VN100YPRPublisher(Node):
    def __init__(self):
        super().__init__('vn100_ypr_publisher')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('publish_rate', 50.0)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        rate = self.get_parameter('publish_rate').value

        self.ypr_pub = self.create_publisher(Vector3, '/vn100/ypr', 10)
        self.ypr_rate_pub = self.create_publisher(Vector3, '/vn100/ypr_rate', 10)

        self.sensor = Sensor()
        try:
            self.sensor.connect(port, baud)
            self.get_logger().info(f'Connected to VN-100 on {port}')
        except Exception as e:
            self.get_logger().fatal(f'Failed to connect to VN-100: {e}')
            raise

        self.ypr_reg = Registers.Attitude.YawPitchRoll()
        self.gyro_reg = Registers.IMU.Gyro()

        self.timer = self.create_timer(1.0 / rate, self.publish_data)

    def publish_data(self):
        self.sensor.readRegister(self.ypr_reg)
        self.sensor.readRegister(self.gyro_reg)

        ypr_msg = Vector3()
        ypr_msg.x = self.ypr_reg.yaw
        ypr_msg.y = self.ypr_reg.pitch
        ypr_msg.z = self.ypr_reg.roll

        ypr_rate_msg = Vector3()
        ypr_rate_msg.x = self.gyro_reg.gyroZ
        ypr_rate_msg.y = self.gyro_reg.gyroY
        ypr_rate_msg.z = self.gyro_reg.gyroX

        self.ypr_pub.publish(ypr_msg)
        self.ypr_rate_pub.publish(ypr_rate_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VN100YPRPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

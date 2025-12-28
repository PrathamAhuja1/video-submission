#!/usr/bin/env python3
"""
PWM Mapper - Maps Twist to 6 ESC channels
Thruster Layout: [Back-Left, Front-Right, Surge-Left, Surge-Right, Back-Right, Front-Left]
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16MultiArray
import numpy as np


class PWMMapper(Node):
    def __init__(self):
        super().__init__('pwm_mapper')
        
        # PWM Configuration
        self.PWM_MIN = 1300
        self.PWM_MAX = 1700
        self.PWM_NEUTRAL = np.array([1500, 1500, 1530, 1500, 1500, 1480], dtype=np.float32)
        
        # Thruster indices
        self.BACK_LEFT = 0
        self.FRONT_RIGHT = 1
        self.SURGE_LEFT = 2
        self.SURGE_RIGHT = 3
        self.BACK_RIGHT = 4
        self.FRONT_LEFT = 5
        
        # Control gains
        self.HEAVE_GAIN = 150.0
        self.SURGE_GAIN = 150.0
        self.YAW_GAIN = 100.0
        self.PITCH_GAIN = 80.0
        
        # Subscribers & Publishers
        self.twist_sub = self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)
        self.pwm_pub = self.create_publisher(UInt16MultiArray, '/PWM8', 10)
        
        self.get_logger().info('='*70)
        self.get_logger().info('✅ PWM Mapper: 6 ESCs configured')
        self.get_logger().info('='*70)
    
    def twist_callback(self, msg: Twist):
        """Convert Twist to PWM"""
        surge = msg.linear.x
        heave = msg.linear.z
        pitch = msg.angular.y
        yaw = msg.angular.z
        
        # Start with neutral
        pwm = self.PWM_NEUTRAL.copy()
        
        # HEAVE (4 vertical thrusters)
        heave_pwm = -heave * self.HEAVE_GAIN
        pwm[self.BACK_LEFT] += heave_pwm
        pwm[self.FRONT_RIGHT] += heave_pwm
        pwm[self.BACK_RIGHT] += heave_pwm
        pwm[self.FRONT_LEFT] += heave_pwm
        
        # PITCH (differential heave)
        pitch_pwm = pitch * self.PITCH_GAIN
        pwm[self.FRONT_RIGHT] -= pitch_pwm
        pwm[self.FRONT_LEFT] -= pitch_pwm
        pwm[self.BACK_RIGHT] += pitch_pwm
        pwm[self.BACK_LEFT] += pitch_pwm
        
        # SURGE (2 surge thrusters)
        surge_pwm = surge * self.SURGE_GAIN
        pwm[self.SURGE_LEFT] += surge_pwm
        pwm[self.SURGE_RIGHT] += surge_pwm
        
        # YAW (differential surge)
        yaw_pwm = yaw * self.YAW_GAIN
        pwm[self.SURGE_LEFT] -= yaw_pwm
        pwm[self.SURGE_RIGHT] += yaw_pwm
        
        # Clamp
        pwm = np.clip(pwm, self.PWM_MIN, self.PWM_MAX).astype(np.uint16)
        
        # Publish
        msg_out = UInt16MultiArray()
        msg_out.data = pwm.tolist()
        self.pwm_pub.publish(msg_out)
        
        self.get_logger().debug(f'PWM: {pwm}', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = PWMMapper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send neutral on exit
        neutral_msg = UInt16MultiArray()
        neutral_msg.data = node.PWM_NEUTRAL.tolist()
        node.pwm_pub.publish(neutral_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
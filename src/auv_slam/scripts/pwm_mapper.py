#!/usr/bin/env python3
"""
Hardware PWM Mapper - Real Thruster Configuration
Maps Twist commands to PWM values for the actual hardware

Thruster Order: [Back-Left, Front-Right, Surge-Left, Surge-Right, 
                 Back-Right, Front-Left, Sway-Left, Sway-Right]

NEUTRAL VALUES: [1500, 1500, 1530, 1500, 1500, 1480, 1500, 1500]

NOTE: Sway thrusters are INACTIVE - send neutral values only
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16MultiArray
import numpy as np


class HardwarePWMMapper(Node):
    def __init__(self):
        super().__init__('hardware_pwm_mapper')
        
        # PWM Configuration
        self.PWM_MIN = 1300
        self.PWM_MAX = 1700
        self.PWM_NEUTRAL = np.array([1500, 1500, 1530, 1500, 1500, 1480, 1500, 1500])
        
        # Thruster indices
        self.BACK_LEFT = 0
        self.FRONT_RIGHT = 1
        self.SURGE_LEFT = 2
        self.SURGE_RIGHT = 3
        self.BACK_RIGHT = 4
        self.FRONT_LEFT = 5
        self.SWAY_LEFT = 6    # INACTIVE
        self.SWAY_RIGHT = 7   # INACTIVE
        
        # Control gains (tune these for your robot)
        self.HEAVE_GAIN = 150.0
        self.SURGE_GAIN = 150.0
        self.YAW_GAIN = 100.0
        self.PITCH_GAIN = 80.0
        
        # Subscriptions
        self.twist_sub = self.create_subscription(
            Twist, '/cmd_vel', self.twist_callback, 10)
        
        # Publishers - PWM8 topic for hardware
        self.pwm_pub = self.create_publisher(UInt16MultiArray, '/PWM8', 10)
        
        self.get_logger().info('='*70)
        self.get_logger().info('🚀 Hardware PWM Mapper Initialized')
        self.get_logger().info('='*70)
        self.get_logger().info('  Configuration: 4 Heave + 2 Surge + 2 Sway (INACTIVE)')
        self.get_logger().info('  Subscribing to: /cmd_vel')
        self.get_logger().info('  Publishing to: /PWM8')
        self.get_logger().info('='*70)
    
    def twist_callback(self, msg: Twist):
        """
        Convert Twist command to PWM values
        
        Twist inputs:
        - linear.x: Surge (forward/backward)
        - linear.y: Sway (left/right) - NOT USED (sway inactive)
        - linear.z: Heave (up/down)
        - angular.x: Roll - NOT USED
        - angular.y: Pitch (nose up/down)
        - angular.z: Yaw (turn left/right)
        """
        
        # Extract commands
        surge = msg.linear.x
        heave = msg.linear.z
        pitch = msg.angular.y
        yaw = msg.angular.z
        
        # Initialize PWM array with neutral values
        pwm = self.PWM_NEUTRAL.copy().astype(float)
        
        # ========================================
        # HEAVE CONTROL (4 vertical thrusters)
        # ========================================
        # Range: 1300-1499 = backward/down, 1500 = neutral, 1501-1700 = forward/up
        # For heave DOWN (negative z), we want 1300-1499
        # For heave UP (positive z), we want 1501-1700
        
        heave_pwm = -heave * self.HEAVE_GAIN  # Negative because down is negative
        
        pwm[self.BACK_LEFT] += heave_pwm
        pwm[self.FRONT_RIGHT] += heave_pwm
        pwm[self.BACK_RIGHT] += heave_pwm
        pwm[self.FRONT_LEFT] += heave_pwm
        
        # ========================================
        # PITCH CONTROL (differential heave)
        # ========================================
        # Positive pitch = nose up
        pitch_pwm = pitch * self.PITCH_GAIN
        
        # Front thrusters push down (lower PWM), back push up (higher PWM)
        pwm[self.FRONT_RIGHT] -= pitch_pwm
        pwm[self.FRONT_LEFT] -= pitch_pwm
        pwm[self.BACK_RIGHT] += pitch_pwm
        pwm[self.BACK_LEFT] += pitch_pwm
        
        # ========================================
        # SURGE CONTROL (2 surge thrusters)
        # ========================================
        # Surge-Left neutral: 1530
        # Surge-Right neutral: 1500
        # For forward: higher PWM
        # For backward: lower PWM
        
        surge_pwm = surge * self.SURGE_GAIN
        
        pwm[self.SURGE_LEFT] += surge_pwm
        pwm[self.SURGE_RIGHT] += surge_pwm
        
        # ========================================
        # YAW CONTROL (differential surge)
        # ========================================
        # Positive yaw = turn left (CCW from above)
        yaw_pwm = yaw * self.YAW_GAIN
        
        # Left thruster backward, right forward for left turn
        pwm[self.SURGE_LEFT] -= yaw_pwm
        pwm[self.SURGE_RIGHT] += yaw_pwm
        
        # ========================================
        # SWAY THRUSTERS - KEEP NEUTRAL (INACTIVE)
        # ========================================
        # Do not modify sway thruster PWM values
        
        # ========================================
        # CLAMP AND PUBLISH
        # ========================================
        pwm = np.clip(pwm, self.PWM_MIN, self.PWM_MAX)
        pwm = pwm.astype(np.uint16)
        
        # Create message
        msg_out = UInt16MultiArray()
        msg_out.data = pwm.tolist()
        self.pwm_pub.publish(msg_out)
        
        # Debug logging (throttled)
        self.get_logger().info(
            f'PWM: [{pwm[0]}, {pwm[1]}, {pwm[2]}, {pwm[3]}, '
            f'{pwm[4]}, {pwm[5]}, {pwm[6]}, {pwm[7]}]',
            throttle_duration_sec=2.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = HardwarePWMMapper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Emergency stop - send all neutral
        neutral_msg = UInt16MultiArray()
        neutral_msg.data = node.PWM_NEUTRAL.tolist()
        node.pwm_pub.publish(neutral_msg)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
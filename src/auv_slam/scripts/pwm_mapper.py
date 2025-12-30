#!/usr/bin/env python3
"""
PWM Mapper - Maps Twist to 6 ESC channels
6-Thruster Configuration (NO SWAY):
  [0] Back-Left (vertical)     - Heave/Pitch/Roll
  [1] Front-Right (vertical)   - Heave/Pitch/Roll
  [2] Surge-Left              - Surge/Yaw (neutral: 1530)
  [3] Surge-Right             - Surge/Yaw
  [4] Back-Right (vertical)    - Heave/Pitch/Roll
  [5] Front-Left (vertical)    - Heave/Pitch/Roll (neutral: 1480)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16MultiArray
import numpy as np


class PWMMapper(Node):
    def __init__(self):
        super().__init__('pwm_mapper')
        
        # PWM Configuration - Updated to 1100-1900 range
        self.PWM_MIN = 1100
        self.PWM_MAX = 1900
        
        # Neutral values for each thruster (6 thrusters only)
        self.PWM_NEUTRAL = np.array([
            1500,  # [0] Back-Left (vertical)
            1500,  # [1] Front-Right (vertical)
            1530,  # [2] Surge-Left (has +30 offset)
            1500,  # [3] Surge-Right
            1500,  # [4] Back-Right (vertical)
            1480,  # [5] Front-Left (vertical, has -20 offset)
        ], dtype=np.float32)
        
        # Thruster indices for clarity
        self.BACK_LEFT = 0      # Vertical
        self.FRONT_RIGHT = 1    # Vertical
        self.SURGE_LEFT = 2     # Horizontal forward
        self.SURGE_RIGHT = 3    # Horizontal forward
        self.BACK_RIGHT = 4     # Vertical
        self.FRONT_LEFT = 5     # Vertical
        
        # Control gains - adjusted for 1100-1900 range (400 range instead of 400)
        # These gains map normalized velocity (-1 to +1) to PWM offset
        self.HEAVE_GAIN = 200.0    # Vertical movement
        self.SURGE_GAIN = 200.0    # Forward/backward
        self.YAW_GAIN = 150.0      # Rotation
        self.PITCH_GAIN = 100.0    # Nose up/down
        self.ROLL_GAIN = 100.0     # Side tilt
        
        # Thruster geometry (for proper mixing)
        # Vertical thrusters positions (for pitch/roll moments)
        self.VERT_FRONT_X = 0.23   # Front thrusters X position
        self.VERT_BACK_X = -0.23   # Back thrusters X position
        self.VERT_LEFT_Y = 0.16    # Left thrusters Y position
        self.VERT_RIGHT_Y = -0.16  # Right thrusters Y position
        
        # Surge thrusters lateral spacing (for yaw)
        self.SURGE_LATERAL = 0.15  # Distance from centerline
        
        # Subscribers & Publishers
        self.twist_sub = self.create_subscription(
            Twist, '/cmd_vel', self.twist_callback, 10)
        self.pwm_pub = self.create_publisher(UInt16MultiArray, '/PWM6', 10)
        
        # Statistics
        self.cmd_count = 0
        
        self.get_logger().info('='*70)
        self.get_logger().info('✅ PWM Mapper: 6-Thruster Configuration (NO SWAY)')
        self.get_logger().info('='*70)
        self.get_logger().info(f'  PWM Range: {self.PWM_MIN}-{self.PWM_MAX} µs')
        self.get_logger().info(f'  Neutral: {self.PWM_NEUTRAL.tolist()}')
        self.get_logger().info('  Thruster Layout:')
        self.get_logger().info('    [0-1,4-5] = Vertical (Heave/Pitch/Roll)')
        self.get_logger().info('    [2-3] = Surge (Forward/Yaw)')
        self.get_logger().info('  ⚠️  NO SWAY capability in this configuration')
        self.get_logger().info('='*70)
    
    def twist_callback(self, msg: Twist):
        """
        Convert Twist command to 6-channel PWM values
        
        Twist components:
          linear.x  = Surge (forward/backward)
          linear.y  = Sway (NOT SUPPORTED - will be ignored)
          linear.z  = Heave (up/down)
          angular.x = Roll (tilt left/right)
          angular.y = Pitch (tilt forward/back)
          angular.z = Yaw (rotate left/right)
        """
        surge = msg.linear.x
        sway = msg.linear.y  # Will be ignored
        heave = msg.linear.z
        roll = msg.angular.x
        pitch = msg.angular.y
        yaw = msg.angular.z
        
        # Warn if sway is commanded
        if abs(sway) > 0.01 and self.cmd_count % 50 == 0:
            self.get_logger().warn(
                f'⚠️ Sway command ({sway:.2f}) ignored - no sway thrusters!',
                throttle_duration_sec=5.0
            )
        
        # Start with neutral PWM values
        pwm = self.PWM_NEUTRAL.copy()
        
        # ============================================================
        # VERTICAL THRUSTERS (4x): Heave + Pitch + Roll
        # ============================================================
        # Base heave contribution (all vertical thrusters)
        heave_pwm = -heave * self.HEAVE_GAIN  # Negative: down is positive heave
        
        # Pitch contribution (front vs back differential)
        # Positive pitch = nose up -> increase back, decrease front
        pitch_pwm_back = pitch * self.PITCH_GAIN
        pitch_pwm_front = -pitch * self.PITCH_GAIN
        
        # Roll contribution (left vs right differential)
        # Positive roll = right side down -> increase right, decrease left
        roll_pwm_right = roll * self.ROLL_GAIN
        roll_pwm_left = -roll * self.ROLL_GAIN
        
        # Apply to each vertical thruster
        pwm[self.BACK_LEFT] += heave_pwm + pitch_pwm_back + roll_pwm_left
        pwm[self.BACK_RIGHT] += heave_pwm + pitch_pwm_back + roll_pwm_right
        pwm[self.FRONT_LEFT] += heave_pwm + pitch_pwm_front + roll_pwm_left
        pwm[self.FRONT_RIGHT] += heave_pwm + pitch_pwm_front + roll_pwm_right
        
        # ============================================================
        # SURGE THRUSTERS (2x): Forward + Yaw
        # ============================================================
        surge_pwm = surge * self.SURGE_GAIN
        
        # Yaw contribution (differential thrust)
        # Positive yaw = turn right -> left forward, right backward
        yaw_surge_pwm = yaw * self.YAW_GAIN
        
        pwm[self.SURGE_LEFT] += surge_pwm + yaw_surge_pwm
        pwm[self.SURGE_RIGHT] += surge_pwm - yaw_surge_pwm
        
        # ============================================================
        # CLAMP AND PUBLISH
        # ============================================================
        pwm = np.clip(pwm, self.PWM_MIN, self.PWM_MAX).astype(np.uint16)
        
        msg_out = UInt16MultiArray()
        msg_out.data = pwm.tolist()
        self.pwm_pub.publish(msg_out)
        
        # Periodic logging
        self.cmd_count += 1
        if self.cmd_count % 50 == 0:
            self.get_logger().info(
                f'CMD: Surge={surge:+.2f} Heave={heave:+.2f} Yaw={yaw:+.2f} '
                f'| PWM: {pwm.tolist()}',
                throttle_duration_sec=2.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = PWMMapper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send neutral PWM on exit
        neutral_msg = UInt16MultiArray()
        neutral_msg.data = node.PWM_NEUTRAL.astype(np.uint16).tolist()
        node.pwm_pub.publish(neutral_msg)
        node.get_logger().info('Sent neutral PWM on shutdown')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
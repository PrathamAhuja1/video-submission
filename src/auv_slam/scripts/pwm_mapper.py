#!/usr/bin/env python3
"""
PWM Mapper - Maps Twist to 8 ESC channels
8-Thruster Configuration (WITH SWAY):
  [0] Back-Left (vertical)        - Heave/Pitch/Roll
  [1] Front-Right (vertical)      - Heave/Pitch/Roll
  [2] Surge-Left                  - Surge/Yaw
  [3] Surge-Right                 - Surge/Yaw
  [4] Back-Right (vertical)       - Heave/Pitch/Roll
  [5] Front-Left (vertical)       - Heave/Pitch/Roll
  [6] Sway-Port (left)            - Sway/Yaw
  [7] Sway-Starboard (right)      - Sway/Yaw
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16MultiArray
import numpy as np


class PWMMapper(Node):
    def __init__(self):
        super().__init__('pwm_mapper')
        
        # PWM Configuration - 1300-1700 range
        self.PWM_MIN = 1300
        self.PWM_MAX = 1700
        self.PWM_RANGE = self.PWM_MAX - self.PWM_MIN  # 400
        
        # Neutral values for 8 thrusters
        self.PWM_NEUTRAL = np.array([
            1500,  # [0] Back-Left (vertical)
            1500,  # [1] Front-Right (vertical)
            1530,  # [2] Surge-Left (has +30 offset)
            1500,  # [3] Surge-Right
            1500,  # [4] Back-Right (vertical)
            1480,  # [5] Front-Left (vertical, has -20 offset)
            1500,  # [6] Sway-Port (left)
            1500,  # [7] Sway-Starboard (right)
        ], dtype=np.float32)
        
        # Thruster indices for clarity
        self.BACK_LEFT = 0      # Vertical
        self.FRONT_RIGHT = 1    # Vertical
        self.SURGE_LEFT = 2     # Horizontal forward (left)
        self.SURGE_RIGHT = 3    # Horizontal forward (right)
        self.BACK_RIGHT = 4     # Vertical
        self.FRONT_LEFT = 5     # Vertical
        self.SWAY_PORT = 6      # Sway (left)
        self.SWAY_STARBOARD = 7 # Sway (right)
        
        # Control gains - adjusted for 1300-1700 range
        self.HEAVE_GAIN = 200.0    # Vertical movement (up/down)
        self.SURGE_GAIN = 200.0    # Forward/backward movement
        self.SWAY_GAIN = 200.0     # Left/right movement (NEW)
        self.YAW_GAIN = 150.0      # Rotation (left/right turn)
        self.PITCH_GAIN = 100.0    # Nose up/down
        self.ROLL_GAIN = 100.0     # Side tilt (left/right lean)
        
        # Thruster geometry (for proper mixing)
        # Vertical thrusters positions
        self.VERT_FRONT_X = 0.23
        self.VERT_BACK_X = -0.23
        self.VERT_LEFT_Y = 0.16
        self.VERT_RIGHT_Y = -0.16
        
        # Surge thrusters lateral spacing
        self.SURGE_LATERAL = 0.15
        
        # Sway thrusters longitudinal spacing
        self.SWAY_LONGITUDINAL = 0.20
        
        # Subscribers & Publishers
        self.twist_sub = self.create_subscription(
            Twist, '/cmd_vel', self.twist_callback, 10)
        self.pwm_pub = self.create_publisher(UInt16MultiArray, '/PWM6', 10)
        
        # Statistics
        self.cmd_count = 0
        
        self.get_logger().info('='*70)
        self.get_logger().info('✅ PWM Mapper: 8-Thruster Configuration (WITH SWAY)')
        self.get_logger().info('='*70)
        self.get_logger().info(f'  PWM Range: {self.PWM_MIN}-{self.PWM_MAX} µs')
        self.get_logger().info(f'  Neutral: {self.PWM_NEUTRAL.tolist()}')
        self.get_logger().info('')
        self.get_logger().info('  Thruster Layout (8):')
        self.get_logger().info('    [0-1,4-5] = Vertical Thrusters (Heave/Pitch/Roll)')
        self.get_logger().info('    [2-3] = Surge Thrusters (Forward)')
        self.get_logger().info('    [6-7] = Sway Thrusters (Lateral) - NEW!')
        self.get_logger().info('')
        self.get_logger().info('  Twist Command Components:')
        self.get_logger().info('    linear.x  = Surge (forward/backward)')
        self.get_logger().info('    linear.y  = Sway (left/right) - NOW SUPPORTED!')
        self.get_logger().info('    linear.z  = Heave (up/down)')
        self.get_logger().info('    angular.x = Roll (tilt left/right)')
        self.get_logger().info('    angular.y = Pitch (tilt forward/back)')
        self.get_logger().info('    angular.z = Yaw (rotate left/right)')
        self.get_logger().info('='*70)
    
    def twist_callback(self, msg: Twist):
        """
        Convert Twist command to 8-channel PWM values
        
        Twist components:
          linear.x  = Surge (forward/backward)
          linear.y  = Sway (left/right) - NOW SUPPORTED!
          linear.z  = Heave (up/down)
          angular.x = Roll (tilt left/right)
          angular.y = Pitch (tilt forward/back)
          angular.z = Yaw (rotate left/right)
        """
        surge = msg.linear.x    # Forward/backward
        sway = msg.linear.y     # Left/right (NEW - NOW SUPPORTED)
        heave = msg.linear.z    # Up/down
        roll = msg.angular.x    # Tilt left/right
        pitch = msg.angular.y   # Tilt forward/back
        yaw = msg.angular.z     # Rotate left/right
        
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
        
        # Yaw contribution from surge (differential thrust)
        # Positive yaw = turn right -> left forward, right backward
        yaw_surge_pwm = yaw * self.YAW_GAIN
        
        pwm[self.SURGE_LEFT] += surge_pwm + yaw_surge_pwm
        pwm[self.SURGE_RIGHT] += surge_pwm - yaw_surge_pwm
        
        # ============================================================
        # SWAY THRUSTERS (2x): Lateral + Yaw (NEW!)
        # ============================================================
        # Sway contribution (left/right movement)
        # Positive sway = move right -> port back, starboard forward
        sway_pwm = sway * self.SWAY_GAIN
        
        # Yaw contribution from sway (differential thrust)
        # Positive yaw = turn right -> port forward, starboard back
        yaw_sway_pwm = yaw * self.YAW_GAIN * 0.5  # Reduced for sway thrusters
        
        pwm[self.SWAY_PORT] += -sway_pwm + yaw_sway_pwm      # Port (left)
        pwm[self.SWAY_STARBOARD] += sway_pwm - yaw_sway_pwm  # Starboard (right)
        
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
                f'CMD: Surge={surge:+.2f} Sway={sway:+.2f} Heave={heave:+.2f} '
                f'Yaw={yaw:+.2f} | PWM[0-7]: {pwm.tolist()}',
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
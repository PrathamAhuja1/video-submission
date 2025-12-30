#!/usr/bin/env python3
"""
Command Mixer - Combines navigation and heave commands
Subscribes to /cmd_vel (navigation) and /cmd_vel_heave (depth control)
Publishes combined command to /cmd_vel_combined
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class CommandMixer(Node):
    def __init__(self):
        super().__init__('command_mixer')
        
        # Command timeout (stop if no commands received)
        self.declare_parameter('command_timeout', 1.0)
        self.command_timeout = self.get_parameter('command_timeout').value
        
        # Latest commands
        self.nav_cmd = Twist()
        self.heave_cmd = Twist()
        
        # Timestamps
        self.nav_time = None
        self.heave_time = None
        
        # Subscriptions
        self.nav_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.nav_callback,
            10
        )
        
        self.heave_sub = self.create_subscription(
            Twist,
            '/cmd_vel_heave',
            self.heave_callback,
            10
        )
        
        # Publisher
        self.combined_pub = self.create_publisher(Twist, '/cmd_vel_combined', 10)
        
        # Mixing timer at 50Hz
        self.mix_timer = self.create_timer(0.02, self.mix_commands)
        
        self.get_logger().info('='*70)
        self.get_logger().info('🎛️ Command Mixer Started')
        self.get_logger().info('   Mixing: /cmd_vel + /cmd_vel_heave → /cmd_vel_combined')
        self.get_logger().info('='*70)
    
    def nav_callback(self, msg: Twist):
        """Receive navigation commands (surge, yaw, pitch)"""
        self.nav_cmd = msg
        self.nav_time = time.time()
    
    def heave_callback(self, msg: Twist):
        """Receive heave (depth) commands"""
        self.heave_cmd = msg
        self.heave_time = time.time()
    
    def mix_commands(self):
        """Combine navigation and heave commands"""
        now = time.time()
        
        # Create combined command
        combined = Twist()
        
        # Check navigation command timeout
        if self.nav_time is not None:
            if (now - self.nav_time) < self.command_timeout:
                # Navigation is active
                combined.linear.x = self.nav_cmd.linear.x    # Surge
                combined.linear.y = self.nav_cmd.linear.y    # Sway
                combined.angular.x = self.nav_cmd.angular.x  # Roll
                combined.angular.y = self.nav_cmd.angular.y  # Pitch
                combined.angular.z = self.nav_cmd.angular.z  # Yaw
        
        # Check heave command timeout
        if self.heave_time is not None:
            if (now - self.heave_time) < self.command_timeout:
                # Heave control is active
                combined.linear.z = self.heave_cmd.linear.z  # Heave
        
        # Publish combined command
        self.combined_pub.publish(combined)


def main(args=None):
    rclpy.init(args=args)
    node = CommandMixer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
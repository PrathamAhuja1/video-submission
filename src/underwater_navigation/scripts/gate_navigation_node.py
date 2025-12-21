#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class GateNavigationNode(Node):
    def __init__(self):
        super().__init__('gate_navigation')
        
        # Subscribers
        self.gate_detection_sub = self.create_subscription(
            Float32MultiArray,
            '/gate_detection',
            self.gate_detection_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/orca4/odom',
            self.odom_callback,
            10
        )
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # State variables
        self.gate_detected = False
        self.gate_center_x = 0
        self.gate_center_y = 0
        self.gate_area = 0
        self.image_width = 1280  # From camera specs
        self.image_height = 720
        
        self.current_position = None
        self.start_position = None
        self.mission_complete = False
        
        # Control parameters
        self.forward_speed = 0.3  # m/s
        self.alignment_gain = 0.002  # Proportional gain for alignment
        self.target_area = 50000  # Target area before passing through
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Gate Navigation Node Started')
        
    def gate_detection_callback(self, msg):
        """Receive gate detection data"""
        if len(msg.data) >= 3:
            self.gate_center_x = msg.data[0]
            self.gate_center_y = msg.data[1]
            self.gate_area = msg.data[2]
            self.gate_detected = True
        else:
            self.gate_detected = False
    
    def odom_callback(self, msg):
        """Track robot position"""
        self.current_position = msg.pose.pose.position
        
        if self.start_position is None:
            self.start_position = self.current_position
            self.get_logger().info(f'Starting position: x={self.current_position.x:.2f}')
    
    def calculate_distance_traveled(self):
        """Calculate distance from start"""
        if self.start_position is None or self.current_position is None:
            return 0.0
        
        dx = self.current_position.x - self.start_position.x
        dy = self.current_position.y - self.start_position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def control_loop(self):
        """Main control loop for navigation"""
        if self.mission_complete:
            return
        
        twist = Twist()
        
        if not self.gate_detected:
            # No gate detected - move forward slowly
            twist.linear.x = 0.2
            self.get_logger().warn('No gate detected, moving forward slowly', throttle_duration_sec=2.0)
        else:
            # Calculate alignment error
            image_center_x = self.image_width / 2
            error_x = self.gate_center_x - image_center_x
            
            # Normalize error (-1 to 1)
            normalized_error = error_x / image_center_x
            
            # Calculate yaw correction
            yaw_correction = -normalized_error * self.alignment_gain
            
            # Move forward while aligning
            twist.linear.x = self.forward_speed
            twist.angular.z = yaw_correction
            
            self.get_logger().info(
                f'Gate area: {self.gate_area:.0f}, Error: {error_x:.0f}, Yaw: {yaw_correction:.3f}',
                throttle_duration_sec=1.0
            )
            
            # Check if we've passed through the gate
            distance = self.calculate_distance_traveled()
            if distance > 2.0:  # Traveled more than 2 meters
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.mission_complete = True
                self.get_logger().info('Mission Complete! Passed through the gate.')
        
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = GateNavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
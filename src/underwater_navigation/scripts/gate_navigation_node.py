#!/usr/bin/env python3
"""
Gate Navigation Node
Autonomously navigates through white gate using visual feedback
Implements simple proportional control for alignment
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
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
        
        self.gate_detected_sub = self.create_subscription(
            Bool,
            '/gate_detected',
            self.gate_detected_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/orca4_ign/odom',
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
        
        # Image dimensions (from camera specs)
        self.image_width = 1280
        self.image_height = 720
        
        # Position tracking
        self.current_position = None
        self.start_position = None
        self.mission_complete = False
        
        # Control parameters
        self.forward_speed = 0.4          # m/s - moderate forward speed
        self.alignment_gain = 0.0015      # Proportional gain for yaw correction
        self.depth_target = -0.5          # Target depth (0.5m below surface)
        self.depth_gain = 0.8             # Proportional gain for depth control
        
        # Mission parameters
        self.target_area_threshold = 80000  # Area when close to gate
        self.distance_threshold = 2.5       # Distance to consider mission complete
        
        # Control loop timer (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('='*60)
        self.get_logger().info('Gate Navigation Node Started')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Forward speed: {self.forward_speed} m/s')
        self.get_logger().info(f'Target depth: {self.depth_target} m')
        self.get_logger().info(f'Alignment gain: {self.alignment_gain}')
        self.get_logger().info('='*60)
    
    def gate_detection_callback(self, msg):
        """Receive gate detection data (center position and area)"""
        if len(msg.data) >= 3:
            self.gate_center_x = msg.data[0]
            self.gate_center_y = msg.data[1]
            self.gate_area = msg.data[2]
    
    def gate_detected_callback(self, msg):
        """Receive gate detection flag"""
        self.gate_detected = msg.data
    
    def odom_callback(self, msg):
        """Track robot position for mission completion"""
        self.current_position = msg.pose.pose.position
        
        if self.start_position is None:
            self.start_position = self.current_position
            self.get_logger().info(
                f'Starting position: X={self.current_position.x:.2f}, '
                f'Y={self.current_position.y:.2f}, '
                f'Z={self.current_position.z:.2f}'
            )
    
    def calculate_distance_traveled(self):
        """Calculate 2D distance from start position"""
        if self.start_position is None or self.current_position is None:
            return 0.0
        
        dx = self.current_position.x - self.start_position.x
        dy = self.current_position.y - self.start_position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def control_loop(self):
        """Main control loop for autonomous navigation"""
        
        # Check if mission is complete
        if self.mission_complete:
            twist = Twist()  # Stop
            self.cmd_vel_pub.publish(twist)
            return
        
        twist = Twist()
        
        # Depth control (always maintain target depth)
        if self.current_position is not None:
            depth_error = self.depth_target - self.current_position.z
            twist.linear.z = depth_error * self.depth_gain
            
            # Clamp vertical velocity
            twist.linear.z = max(-0.5, min(twist.linear.z, 0.5))
        
        if not self.gate_detected:
            # ============================================================
            # NO GATE DETECTED - Search mode
            # ============================================================
            twist.linear.x = 0.2  # Slow forward movement
            twist.angular.z = 0.1  # Gentle rotation to search
            
            self.get_logger().warn(
                'Gate not detected - Searching...',
                throttle_duration_sec=2.0
            )
        
        else:
            # ============================================================
            # GATE DETECTED - Approach and align
            # ============================================================
            
            # Calculate alignment error (horizontal)
            image_center_x = self.image_width / 2
            error_x = self.gate_center_x - image_center_x
            
            # Normalize error to range [-1, 1]
            normalized_error = error_x / image_center_x
            
            # Calculate yaw correction (proportional control)
            yaw_correction = -normalized_error * self.alignment_gain
            
            # Clamp yaw rate
            yaw_correction = max(-0.5, min(yaw_correction, 0.5))
            
            # Forward speed - reduce as we get closer to gate
            if self.gate_area < 20000:
                # Far from gate - full speed
                twist.linear.x = self.forward_speed
            elif self.gate_area < 50000:
                # Medium distance - moderate speed
                twist.linear.x = self.forward_speed * 0.7
            else:
                # Close to gate - slow speed
                twist.linear.x = self.forward_speed * 0.5
            
            # Apply yaw correction
            twist.angular.z = yaw_correction
            
            # Log status
            self.get_logger().info(
                f'Gate area: {int(self.gate_area)} px | '
                f'Error: {int(error_x)} px | '
                f'Yaw: {yaw_correction:+.3f} rad/s | '
                f'Speed: {twist.linear.x:.2f} m/s',
                throttle_duration_sec=0.5
            )
            
            # Check if close enough to gate (large area = close)
            if self.gate_area > self.target_area_threshold:
                self.get_logger().info(
                    '🎯 VERY CLOSE TO GATE - Committing to passage!',
                    throttle_duration_sec=1.0
                )
                # Full speed through gate
                twist.linear.x = self.forward_speed * 1.2
                twist.angular.z = 0.0  # No more corrections
        
        # Publish velocity command
        self.cmd_vel_pub.publish(twist)
        
        # Check mission completion
        distance = self.calculate_distance_traveled()
        if distance > self.distance_threshold:
            self.mission_complete = True
            self.get_logger().info('='*60)
            self.get_logger().info('🎉 MISSION COMPLETE!')
            self.get_logger().info(f'Distance traveled: {distance:.2f} m')
            self.get_logger().info('Successfully passed through the gate!')
            self.get_logger().info('='*60)


def main(args=None):
    rclpy.init(args=args)
    node = GateNavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down gracefully...')
    finally:
        # Send stop command
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
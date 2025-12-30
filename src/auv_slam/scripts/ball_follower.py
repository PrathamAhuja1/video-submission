#!/usr/bin/env python3
"""
Ball Follower - Autonomous ball tracking and following
Moves toward and follows the orange ball underwater
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist, Vector3
import time
import math


class BallFollower(Node):
    def __init__(self):
        super().__init__('ball_follower')
        
        # State machine
        self.STABILIZING = 0
        self.SEARCHING = 1
        self.TRACKING = 2
        self.APPROACHING = 3
        self.CLOSE_FOLLOW = 4
        self.REACHED = 5
        
        self.state = self.STABILIZING
        
        # Navigation parameters
        self.TARGET_DEPTH = 1.0
        self.DEPTH_TOLERANCE = 0.1
        
        # Distance thresholds
        self.TRACKING_DISTANCE = 3.0    # Start tracking
        self.APPROACH_DISTANCE = 1.5    # Start approaching
        self.CLOSE_DISTANCE = 0.5       # Close following
        self.REACHED_DISTANCE = 0.1     # Ball reached
        
        # Speed parameters
        self.SEARCH_SPEED = 0.25
        self.TRACKING_SPEED = 0.4
        self.APPROACH_SPEED = 0.5
        self.CLOSE_SPEED = 0.2
        
        # Control gains
        self.SEARCH_YAW_RATE = 0.3
        self.TRACKING_YAW_GAIN = 2.5    # Aggressive turning for tracking
        self.APPROACH_YAW_GAIN = 2.0
        self.CLOSE_YAW_GAIN = 1.5
        
        self.ALIGNMENT_THRESHOLD = 0.1  # Acceptable alignment error
        
        # State variables
        self.ball_detected = False
        self.alignment_error = 0.0
        self.estimated_distance = 999.0
        
        self.current_depth = 0.0
        self.current_yaw = 0.0
        
        self.state_start_time = time.time()
        self.last_detection_time = 0.0
        self.detection_timeout = 3.0  # Return to search after 3s without ball
        
        # Subscriptions - Ball detection
        self.create_subscription(Bool, '/ball/detected', 
                                self.ball_detected_callback, 10)
        self.create_subscription(Float32, '/ball/alignment_error',
                                self.alignment_callback, 10)
        self.create_subscription(Float32, '/ball/estimated_distance',
                                self.distance_callback, 10)
        
        # Subscriptions - Sensors
        self.create_subscription(Float32, '/depth',
                                self.depth_callback, 10)
        self.create_subscription(Vector3, '/vn100/ypr',
                                self.ypr_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control loop at 20Hz
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('='*70)
        self.get_logger().info('🏀 Ball Follower Started')
        self.get_logger().info('='*70)
        self.get_logger().info(f'  Target Depth: {self.TARGET_DEPTH}m')
        self.get_logger().info(f'  Approach: {self.APPROACH_DISTANCE}m')
        self.get_logger().info(f'  Close: {self.CLOSE_DISTANCE}m')
        self.get_logger().info('='*70)
    
    # ========================================
    # CALLBACKS
    # ========================================
    
    def ball_detected_callback(self, msg: Bool):
        was_detected = self.ball_detected
        self.ball_detected = msg.data
        
        if self.ball_detected:
            self.last_detection_time = time.time()
            
            if not was_detected and self.state == self.SEARCHING:
                self.get_logger().info(f'🎯 BALL DETECTED at {self.estimated_distance:.2f}m')
    
    def alignment_callback(self, msg: Float32):
        self.alignment_error = msg.data
    
    def distance_callback(self, msg: Float32):
        self.estimated_distance = msg.data
    
    def depth_callback(self, msg: Float32):
        self.current_depth = msg.data
    
    def ypr_callback(self, msg: Vector3):
        self.current_yaw = math.radians(msg.x)
    
    # ========================================
    # CONTROL LOOP
    # ========================================
    
    def control_loop(self):
        cmd = Twist()
        
        # Depth control (always active)
        cmd.linear.z = self.compute_depth_control()
        
        # State machine
        if self.state == self.STABILIZING:
            cmd = self.stabilizing(cmd)
        elif self.state == self.SEARCHING:
            cmd = self.searching(cmd)
        elif self.state == self.TRACKING:
            cmd = self.tracking(cmd)
        elif self.state == self.APPROACHING:
            cmd = self.approaching(cmd)
        elif self.state == self.CLOSE_FOLLOW:
            cmd = self.close_follow(cmd)
        elif self.state == self.REACHED:
            cmd = self.reached(cmd)
        
        self.cmd_vel_pub.publish(cmd)
    
    # ========================================
    # DEPTH CONTROL
    # ========================================
    
    def compute_depth_control(self) -> float:
        depth_error = self.TARGET_DEPTH - self.current_depth
        
        if abs(depth_error) < self.DEPTH_TOLERANCE:
            return 0.0
        
        z_cmd = depth_error * 0.8
        return max(-0.6, min(z_cmd, 0.6))
    
    # ========================================
    # STATE HANDLERS
    # ========================================
    
    def stabilizing(self, cmd: Twist) -> Twist:
        """Wait for depth stabilization"""
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        
        depth_stable = abs(self.TARGET_DEPTH - self.current_depth) < 0.15
        elapsed = time.time() - self.state_start_time
        
        if depth_stable and elapsed > 3.0:
            self.get_logger().info('✅ Stabilized - Starting ball search')
            self.transition_to(self.SEARCHING)
        elif int(elapsed) % 2 == 0:
            self.get_logger().info(
                f'⏳ Stabilizing... Depth: {self.current_depth:.2f}m',
                throttle_duration_sec=1.9
            )
        
        return cmd
    
    def searching(self, cmd: Twist) -> Twist:
        """Search for ball with rotation"""
        
        if self.ball_detected and self.estimated_distance < 999:
            self.get_logger().info('🎯 Ball found - Starting tracking')
            self.transition_to(self.TRACKING)
            return cmd
        
        # Slow forward movement + rotation
        cmd.linear.x = self.SEARCH_SPEED
        
        elapsed = time.time() - self.state_start_time
        sweep_phase = (elapsed % 10.0) / 10.0
        
        if sweep_phase < 0.5:
            cmd.angular.z = self.SEARCH_YAW_RATE
        else:
            cmd.angular.z = -self.SEARCH_YAW_RATE
        
        if int(elapsed) % 3 == 0:
            self.get_logger().info(
                f'🔍 Searching for ball... ({elapsed:.0f}s)',
                throttle_duration_sec=2.9
            )
        
        return cmd
    
    def tracking(self, cmd: Twist) -> Twist:
        """Track ball from distance - aggressive turning"""
        
        # Check if ball lost
        if not self.ball_detected or (time.time() - self.last_detection_time) > self.detection_timeout:
            self.get_logger().warn('⚠️ Ball lost - returning to search')
            self.transition_to(self.SEARCHING)
            return cmd
        
        # Transition based on distance
        if self.estimated_distance <= self.APPROACH_DISTANCE:
            self.get_logger().info('➡️ Close enough - Starting approach')
            self.transition_to(self.APPROACHING)
            return cmd
        
        # Track with aggressive turning
        cmd.linear.x = self.TRACKING_SPEED
        cmd.angular.z = -self.alignment_error * self.TRACKING_YAW_GAIN
        
        if int((time.time() - self.state_start_time) * 2) % 3 == 0:
            self.get_logger().info(
                f'🎯 Tracking: {self.estimated_distance:.2f}m | Align: {self.alignment_error:+.3f}',
                throttle_duration_sec=1.4
            )
        
        return cmd
    
    def approaching(self, cmd: Twist) -> Twist:
        """Approach ball with precise alignment"""
        
        if not self.ball_detected or (time.time() - self.last_detection_time) > self.detection_timeout:
            self.get_logger().warn('⚠️ Ball lost during approach')
            self.transition_to(self.SEARCHING)
            return cmd
        
        # Transition based on distance
        if self.estimated_distance <= self.CLOSE_DISTANCE:
            self.get_logger().info('🔥 Very close - Close following mode')
            self.transition_to(self.CLOSE_FOLLOW)
            return cmd
        
        # Approach with moderate turning
        cmd.linear.x = self.APPROACH_SPEED
        cmd.angular.z = -self.alignment_error * self.APPROACH_YAW_GAIN
        
        if int((time.time() - self.state_start_time) * 2) % 3 == 0:
            self.get_logger().info(
                f'➡️ Approaching: {self.estimated_distance:.2f}m | Align: {self.alignment_error:+.3f}',
                throttle_duration_sec=1.4
            )
        
        return cmd
    
    def close_follow(self, cmd: Twist) -> Twist:
        """Close range following"""
        
        if not self.ball_detected or (time.time() - self.last_detection_time) > self.detection_timeout:
            self.get_logger().warn('⚠️ Ball lost at close range')
            self.transition_to(self.SEARCHING)
            return cmd
        
        # Check if reached
        if self.estimated_distance <= self.REACHED_DISTANCE:
            self.get_logger().info('🎉 BALL REACHED!')
            self.transition_to(self.REACHED)
            return cmd
        
        # Slow precise following
        cmd.linear.x = self.CLOSE_SPEED
        cmd.angular.z = -self.alignment_error * self.CLOSE_YAW_GAIN
        
        if int((time.time() - self.state_start_time) * 3) % 4 == 0:
            self.get_logger().info(
                f'🔥 Close follow: {self.estimated_distance:.2f}m | Align: {self.alignment_error:+.3f}',
                throttle_duration_sec=1.2
            )
        
        return cmd
    
    def reached(self, cmd: Twist) -> Twist:
        """Ball reached - maintain position"""
        
        if not self.ball_detected or (time.time() - self.last_detection_time) > self.detection_timeout:
            self.get_logger().warn('⚠️ Ball moved away')
            self.transition_to(self.SEARCHING)
            return cmd
        
        # If ball moves away, follow again
        if self.estimated_distance > self.CLOSE_DISTANCE:
            self.get_logger().info('🏀 Ball moving - resuming follow')
            self.transition_to(self.CLOSE_FOLLOW)
            return cmd
        
        # Maintain position with minimal movement
        if abs(self.alignment_error) > self.ALIGNMENT_THRESHOLD:
            cmd.angular.z = -self.alignment_error * 1.0
        
        if abs(self.estimated_distance - self.REACHED_DISTANCE) > 0.1:
            cmd.linear.x = (self.estimated_distance - self.REACHED_DISTANCE) * 0.3
        
        if int((time.time() - self.state_start_time) * 2) % 5 == 0:
            self.get_logger().info(
                f'🎯 Holding position: {self.estimated_distance:.2f}m',
                throttle_duration_sec=2.4
            )
        
        return cmd
    
    # ========================================
    # UTILITIES
    # ========================================
    
    def transition_to(self, new_state: int):
        state_names = {
            self.STABILIZING: 'STABILIZING',
            self.SEARCHING: 'SEARCHING',
            self.TRACKING: 'TRACKING',
            self.APPROACHING: 'APPROACHING',
            self.CLOSE_FOLLOW: 'CLOSE_FOLLOW',
            self.REACHED: 'REACHED'
        }
        
        old_name = state_names.get(self.state, 'UNKNOWN')
        self.state = new_state
        self.state_start_time = time.time()
        new_name = state_names.get(new_state, 'UNKNOWN')
        
        self.get_logger().info(f'🔄 STATE: {old_name} → {new_name}')


def main(args=None):
    rclpy.init(args=args)
    node = BallFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Emergency stop
        stop_cmd = Twist()
        node.cmd_vel_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
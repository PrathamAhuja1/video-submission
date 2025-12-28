#!/usr/bin/env python3
"""
Hardware Gate Navigator - Real Robot Operation
Uses actual depth sensor and VN100 IMU data
Pool: 6.1m x 6.1m, depth 1.37m
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist, Vector3
import time
import math


class HardwareGateNavigator(Node):
    def __init__(self):
        super().__init__('hardware_gate_navigator')
        
        # State machine
        self.STABILIZING = 0
        self.SEARCHING = 1
        self.APPROACHING = 2
        self.ALIGNING = 3
        self.PASSING = 4
        self.COMPLETED = 5
        
        self.state = self.STABILIZING
        
        # Navigation parameters (tuned for 6.1m x 6.1m pool)
        self.TARGET_DEPTH = 0.6  # 60cm below surface (pool is 1.37m deep)
        self.DEPTH_TOLERANCE = 0.1
        
        self.APPROACH_DISTANCE = 2.0  # When to start approaching
        self.ALIGNMENT_DISTANCE = 1.0  # When to align precisely
        self.PASSING_DISTANCE = 0.5    # When to commit to passage
        
        self.SEARCH_SPEED = 0.3
        self.APPROACH_SPEED = 0.4
        self.PASSING_SPEED = 0.6
        self.PASSING_DURATION = 5.0  # Time to drive through gate
        
        # State variables
        self.gate_detected = False
        self.alignment_error = 0.0
        self.estimated_distance = 999.0
        
        self.current_depth = 0.0
        self.current_yaw = 0.0
        self.current_yaw_rate = 0.0
        
        self.state_start_time = time.time()
        self.passing_start_time = 0.0
        
        # Subscriptions - Gate detection
        self.create_subscription(Bool, '/gate/detected', 
                                self.gate_detected_callback, 10)
        self.create_subscription(Float32, '/gate/alignment_error',
                                self.alignment_callback, 10)
        self.create_subscription(Float32, '/gate/estimated_distance',
                                self.distance_callback, 10)
        
        # Subscriptions - Sensors
        self.create_subscription(Float32, '/depth',
                                self.depth_callback, 10)
        self.create_subscription(Vector3, '/vn100/ypr',
                                self.ypr_callback, 10)
        self.create_subscription(Vector3, '/vn100/ypr_rate',
                                self.ypr_rate_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control loop at 20Hz
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('='*70)
        self.get_logger().info('🤖 Hardware Gate Navigator Started')
        self.get_logger().info('='*70)
        self.get_logger().info(f'  Pool: 6.1m x 6.1m, Depth: 1.37m')
        self.get_logger().info(f'  Target Depth: {self.TARGET_DEPTH}m')
        self.get_logger().info('='*70)
    
    # ========================================
    # CALLBACKS
    # ========================================
    
    def gate_detected_callback(self, msg: Bool):
        was_detected = self.gate_detected
        self.gate_detected = msg.data
        
        if not was_detected and self.gate_detected and self.state == self.SEARCHING:
            self.get_logger().info(f'🎯 GATE DETECTED at {self.estimated_distance:.2f}m')
    
    def alignment_callback(self, msg: Float32):
        self.alignment_error = msg.data
    
    def distance_callback(self, msg: Float32):
        self.estimated_distance = msg.data
    
    def depth_callback(self, msg: Float32):
        """Real depth sensor data (in meters)"""
        self.current_depth = msg.data
    
    def ypr_callback(self, msg: Vector3):
        """VN100 Yaw, Pitch, Roll (in degrees)"""
        self.current_yaw = math.radians(msg.x)  # Convert to radians
    
    def ypr_rate_callback(self, msg: Vector3):
        """VN100 YPR rates (in degrees/sec)"""
        self.current_yaw_rate = math.radians(msg.x)
    
    # ========================================
    # CONTROL LOOP
    # ========================================
    
    def control_loop(self):
        """Main control loop at 20Hz"""
        
        cmd = Twist()
        
        # Depth control (always active)
        cmd.linear.z = self.compute_depth_control()
        
        # State machine
        if self.state == self.STABILIZING:
            cmd = self.stabilizing(cmd)
        elif self.state == self.SEARCHING:
            cmd = self.searching(cmd)
        elif self.state == self.APPROACHING:
            cmd = self.approaching(cmd)
        elif self.state == self.ALIGNING:
            cmd = self.aligning(cmd)
        elif self.state == self.PASSING:
            cmd = self.passing(cmd)
        elif self.state == self.COMPLETED:
            cmd = self.completed(cmd)
        
        self.cmd_vel_pub.publish(cmd)
    
    # ========================================
    # DEPTH CONTROL
    # ========================================
    
    def compute_depth_control(self) -> float:
        """
        Depth control using real sensor
        Positive depth_error = need to go deeper (down)
        Returns: linear.z command
        """
        depth_error = self.TARGET_DEPTH - self.current_depth
        
        if abs(depth_error) < self.DEPTH_TOLERANCE:
            return 0.0
        
        # Proportional control
        z_cmd = depth_error * 0.8
        return max(-0.6, min(z_cmd, 0.6))
    
    # ========================================
    # STATE HANDLERS
    # ========================================
    
    def stabilizing(self, cmd: Twist) -> Twist:
        """Wait for depth to stabilize"""
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        
        depth_stable = abs(self.TARGET_DEPTH - self.current_depth) < 0.15
        elapsed = time.time() - self.state_start_time
        
        if depth_stable and elapsed > 3.0:
            self.get_logger().info('='*70)
            self.get_logger().info('✅ Depth Stabilized - Starting Mission')
            self.get_logger().info(f'   Current Depth: {self.current_depth:.2f}m')
            self.get_logger().info('='*70)
            self.transition_to(self.SEARCHING)
        elif int(elapsed) % 2 == 0:
            self.get_logger().info(
                f'⏳ Stabilizing... Depth: {self.current_depth:.2f}m / {self.TARGET_DEPTH:.2f}m',
                throttle_duration_sec=1.9
            )
        
        return cmd
    
    def searching(self, cmd: Twist) -> Twist:
        """Search pattern for gate"""
        
        if self.gate_detected and self.estimated_distance < 999:
            self.get_logger().info('🎯 Gate found - Starting approach')
            self.transition_to(self.APPROACHING)
            return cmd
        
        # Search pattern: forward + slow rotation
        cmd.linear.x = self.SEARCH_SPEED
        
        elapsed = time.time() - self.state_start_time
        sweep_phase = (elapsed % 8.0) / 8.0
        
        if sweep_phase < 0.5:
            cmd.angular.z = 0.2
        else:
            cmd.angular.z = -0.2
        
        if int(elapsed) % 3 == 0:
            self.get_logger().info(
                f'🔍 Searching... ({elapsed:.0f}s)',
                throttle_duration_sec=2.9
            )
        
        return cmd
    
    def approaching(self, cmd: Twist) -> Twist:
        """Approach gate"""
        
        if not self.gate_detected:
            self.get_logger().warn('⚠️ Gate lost - returning to search')
            self.transition_to(self.SEARCHING)
            return cmd
        
        # Check distance thresholds
        if self.estimated_distance <= self.ALIGNMENT_DISTANCE:
            self.get_logger().info('📍 Close enough - Starting alignment')
            self.transition_to(self.ALIGNING)
            return cmd
        
        # Approach with light correction
        cmd.linear.x = self.APPROACH_SPEED
        cmd.angular.z = -self.alignment_error * 1.0
        
        if int((time.time() - self.state_start_time) * 2) % 3 == 0:
            self.get_logger().info(
                f'➡️ Approaching: {self.estimated_distance:.2f}m',
                throttle_duration_sec=1.4
            )
        
        return cmd
    
    def aligning(self, cmd: Twist) -> Twist:
        """Precise alignment before passing"""
        
        if not self.gate_detected:
            self.get_logger().warn('⚠️ Gate lost during alignment')
            self.transition_to(self.SEARCHING)
            return cmd
        
        # Check if ready to pass
        if self.estimated_distance <= self.PASSING_DISTANCE:
            if abs(self.alignment_error) < 0.15:
                self.get_logger().info('🚀 ALIGNED - PASSING THROUGH GATE!')
                self.passing_start_time = time.time()
                self.transition_to(self.PASSING)
                return cmd
        
        # Alignment control
        if abs(self.alignment_error) > 0.2:
            cmd.linear.x = 0.1
            cmd.angular.z = -self.alignment_error * 3.0
        else:
            cmd.linear.x = 0.2
            cmd.angular.z = -self.alignment_error * 2.0
        
        return cmd
    
    def passing(self, cmd: Twist) -> Twist:
        """Full speed passage through gate"""
        
        elapsed = time.time() - self.passing_start_time
        
        if elapsed >= self.PASSING_DURATION:
            self.get_logger().info('='*70)
            self.get_logger().info('🎉 GATE PASSAGE COMPLETE!')
            self.get_logger().info('='*70)
            self.transition_to(self.COMPLETED)
            return cmd
        
        # Full speed forward
        cmd.linear.x = self.PASSING_SPEED
        cmd.angular.z = 0.0
        
        remaining = self.PASSING_DURATION - elapsed
        self.get_logger().info(
            f'🚀 PASSING: {elapsed:.1f}s / {self.PASSING_DURATION}s ({remaining:.1f}s left)',
            throttle_duration_sec=0.5
        )
        
        return cmd
    
    def completed(self, cmd: Twist) -> Twist:
        """Mission complete - stop"""
        
        if not hasattr(self, '_completion_logged'):
            total_time = time.time() - self.state_start_time
            self.get_logger().info('='*70)
            self.get_logger().info('🏆 MISSION COMPLETE!')
            self.get_logger().info(f'   Total Time: {total_time:.1f}s')
            self.get_logger().info('='*70)
            self._completion_logged = True
        
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        
        return cmd
    
    # ========================================
    # UTILITIES
    # ========================================
    
    def transition_to(self, new_state: int):
        """State transition"""
        state_names = {
            self.STABILIZING: 'STABILIZING',
            self.SEARCHING: 'SEARCHING',
            self.APPROACHING: 'APPROACHING',
            self.ALIGNING: 'ALIGNING',
            self.PASSING: 'PASSING',
            self.COMPLETED: 'COMPLETED'
        }
        
        old_name = state_names.get(self.state, 'UNKNOWN')
        self.state = new_state
        self.state_start_time = time.time()
        new_name = state_names.get(new_state, 'UNKNOWN')
        
        self.get_logger().info(f'🔄 STATE: {old_name} → {new_name}')


def main(args=None):
    rclpy.init(args=args)
    node = HardwareGateNavigator()
    
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
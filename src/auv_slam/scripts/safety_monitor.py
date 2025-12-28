#!/usr/bin/env python3
"""
Hardware Safety Monitor
Monitors depth, orientation, and pool boundaries
Emergency stop if safety limits exceeded
Pool: 6.1m x 6.1m, depth 1.37m
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, UInt16MultiArray
from geometry_msgs.msg import Vector3
import time
import math


class HardwareSafetyMonitor(Node):
    def __init__(self):
        super().__init__('hardware_safety_monitor')
        
        # Safety parameters
        self.declare_parameter('max_depth', 1.2)  # Don't go below 1.2m
        self.declare_parameter('min_depth', 0.1)  # Stay at least 10cm below surface
        self.declare_parameter('max_roll', 30.0)  # degrees
        self.declare_parameter('max_pitch', 30.0)  # degrees
        self.declare_parameter('max_mission_time', 300.0)  # 5 minutes
        
        self.max_depth = self.get_parameter('max_depth').value
        self.min_depth = self.get_parameter('min_depth').value
        self.max_roll = self.get_parameter('max_roll').value
        self.max_pitch = self.get_parameter('max_pitch').value
        self.max_mission_time = self.get_parameter('max_mission_time').value
        
        # State tracking
        self.current_depth = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        
        self.last_depth_time = None
        self.last_imu_time = None
        
        self.emergency_triggered = False
        self.mission_start_time = time.time()
        
        self.depth_violations = 0
        self.tilt_violations = 0
        
        # Subscriptions
        self.create_subscription(Float32, '/depth', self.depth_callback, 10)
        self.create_subscription(Vector3, '/vn100/ypr', self.ypr_callback, 10)
        
        # Publishers
        self.emergency_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.emergency_pwm_pub = self.create_publisher(UInt16MultiArray, '/PWM8', 10)
        
        # Safety check timer (10Hz)
        self.create_timer(0.1, self.safety_check)
        
        self.get_logger().info('='*70)
        self.get_logger().info('🛡️ Hardware Safety Monitor Active')
        self.get_logger().info('='*70)
        self.get_logger().info(f'  Max Depth: {self.max_depth}m')
        self.get_logger().info(f'  Min Depth: {self.min_depth}m')
        self.get_logger().info(f'  Max Roll/Pitch: ±{self.max_roll}°')
        self.get_logger().info(f'  Max Mission Time: {self.max_mission_time}s')
        self.get_logger().info('='*70)
    
    def depth_callback(self, msg: Float32):
        self.current_depth = msg.data
        self.last_depth_time = time.time()
    
    def ypr_callback(self, msg: Vector3):
        """VN100 Yaw, Pitch, Roll in degrees"""
        self.current_yaw = msg.x
        self.current_pitch = msg.y
        self.current_roll = msg.z
        self.last_imu_time = time.time()
    
    def safety_check(self):
        """Main safety monitoring loop"""
        
        if self.emergency_triggered:
            self.execute_emergency_stop()
            return
        
        now = time.time()
        
        # Check sensor timeouts
        if self.last_depth_time:
            depth_age = now - self.last_depth_time
            if depth_age > 2.0:
                self.trigger_emergency(f"Depth sensor timeout ({depth_age:.1f}s)")
                return
        
        if self.last_imu_time:
            imu_age = now - self.last_imu_time
            if imu_age > 2.0:
                self.trigger_emergency(f"IMU sensor timeout ({imu_age:.1f}s)")
                return
        
        # Check mission timeout
        mission_time = now - self.mission_start_time
        if mission_time > self.max_mission_time:
            self.trigger_emergency(f"Mission timeout ({mission_time:.0f}s)")
            return
        
        # Check depth limits
        if self.current_depth > self.max_depth:
            self.depth_violations += 1
            if self.depth_violations > 3:
                self.trigger_emergency(f"Depth too deep: {self.current_depth:.2f}m > {self.max_depth}m")
                return
            self.get_logger().warn(
                f'⚠️ Depth warning: {self.current_depth:.2f}m (max: {self.max_depth}m)'
            )
        elif self.current_depth < self.min_depth:
            self.depth_violations += 1
            if self.depth_violations > 3:
                self.trigger_emergency(f"Depth too shallow: {self.current_depth:.2f}m < {self.min_depth}m")
                return
            self.get_logger().warn(
                f'⚠️ Depth warning: {self.current_depth:.2f}m (min: {self.min_depth}m)'
            )
        else:
            self.depth_violations = 0
        
        # Check orientation limits
        if abs(self.current_roll) > self.max_roll or abs(self.current_pitch) > self.max_pitch:
            self.tilt_violations += 1
            if self.tilt_violations > 5:
                self.trigger_emergency(
                    f"Excessive tilt: Roll={self.current_roll:.1f}°, Pitch={self.current_pitch:.1f}°"
                )
                return
            self.get_logger().warn(
                f'⚠️ Tilt warning: R={self.current_roll:.1f}°, P={self.current_pitch:.1f}°'
            )
        else:
            self.tilt_violations = 0
        
        # Periodic status (every 10 seconds)
        if int(mission_time) % 10 == 0:
            self.get_logger().info(
                f'✅ Safe | Depth: {self.current_depth:.2f}m | '
                f'R: {self.current_roll:.1f}° | P: {self.current_pitch:.1f}° | '
                f'Time: {mission_time:.0f}s',
                throttle_duration_sec=9.5
            )
    
    def trigger_emergency(self, reason: str):
        """Activate emergency protocols"""
        if self.emergency_triggered:
            return
        
        self.emergency_triggered = True
        
        self.get_logger().error('='*70)
        self.get_logger().error(f'🚨 EMERGENCY STOP: {reason}')
        self.get_logger().error('='*70)
        
        # Publish emergency flag
        self.emergency_pub.publish(Bool(data=True))
        
        # Execute emergency stop
        self.execute_emergency_stop()
    
    def execute_emergency_stop(self):
        """Send all-stop PWM command"""
        # Neutral PWM values for all thrusters
        neutral_pwm = [1500, 1500, 1530, 1500, 1500, 1480, 1500, 1500]
        
        emergency_msg = UInt16MultiArray()
        emergency_msg.data = neutral_pwm
        self.emergency_pwm_pub.publish(emergency_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HardwareSafetyMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send emergency stop on shutdown
        neutral_pwm = [1500, 1500, 1530, 1500, 1500, 1480, 1500, 1500]
        stop_msg = UInt16MultiArray()
        stop_msg.data = neutral_pwm
        node.emergency_pwm_pub.publish(stop_msg)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
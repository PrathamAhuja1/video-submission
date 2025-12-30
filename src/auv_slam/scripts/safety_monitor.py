#!/usr/bin/env python3
"""
Hardware Safety Monitor - 8 Thruster Configuration
Monitors depth, orientation, and operational limits
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
        self.declare_parameter('sensor_timeout', 3.0)  # seconds
        
        self.max_depth = self.get_parameter('max_depth').value
        self.min_depth = self.get_parameter('min_depth').value
        self.max_roll = self.get_parameter('max_roll').value
        self.max_pitch = self.get_parameter('max_pitch').value
        self.max_mission_time = self.get_parameter('max_mission_time').value
        self.sensor_timeout = self.get_parameter('sensor_timeout').value
        
        # Neutral PWM for 8 thrusters
        self.NEUTRAL_PWM = [1500, 1500, 1530, 1500, 1500, 1480, 1500, 1500]
        
        # State tracking
        self.current_depth = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        
        self.last_depth_time = None
        self.last_imu_time = None
        
        self.emergency_triggered = False
        self.mission_start_time = time.time()
        
        # Violation counters (to avoid false positives)
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
        
        # Emergency stop timer (50Hz when active)
        self.emergency_timer = self.create_timer(0.02, self.emergency_stop_loop)
        
        self.get_logger().info('='*70)
        self.get_logger().info('🛡️ Hardware Safety Monitor Active (8-Thruster)')
        self.get_logger().info('='*70)
        self.get_logger().info(f'  Depth Range: {self.min_depth}m - {self.max_depth}m')
        self.get_logger().info(f'  Max Roll/Pitch: ±{self.max_roll}°')
        self.get_logger().info(f'  Max Mission Time: {self.max_mission_time}s')
        self.get_logger().info(f'  Sensor Timeout: {self.sensor_timeout}s')
        self.get_logger().info('='*70)
    
    def depth_callback(self, msg: Float32):
        """Receive current depth measurement"""
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
        
        # If emergency already triggered, let emergency_stop_loop handle it
        if self.emergency_triggered:
            return
        
        now = time.time()
        
        # ========================================
        # CHECK 1: Sensor Timeouts
        # ========================================
        if self.last_depth_time:
            depth_age = now - self.last_depth_time
            if depth_age > self.sensor_timeout:
                self.trigger_emergency(
                    f"Depth sensor timeout ({depth_age:.1f}s > {self.sensor_timeout}s)"
                )
                return
        
        if self.last_imu_time:
            imu_age = now - self.last_imu_time
            if imu_age > self.sensor_timeout:
                self.trigger_emergency(
                    f"IMU sensor timeout ({imu_age:.1f}s > {self.sensor_timeout}s)"
                )
                return
        
        # ========================================
        # CHECK 2: Mission Timeout
        # ========================================
        mission_time = now - self.mission_start_time
        if mission_time > self.max_mission_time:
            self.trigger_emergency(
                f"Mission timeout ({mission_time:.0f}s > {self.max_mission_time}s)"
            )
            return
        
        # ========================================
        # CHECK 3: Depth Limits
        # ========================================
        if self.last_depth_time:  # Only check if we have depth data
            if self.current_depth > self.max_depth:
                self.depth_violations += 1
                if self.depth_violations > 5:  # 0.5s at 10Hz
                    self.trigger_emergency(
                        f"Depth too deep: {self.current_depth:.2f}m > {self.max_depth}m"
                    )
                    return
                self.get_logger().warn(
                    f'⚠️ Depth warning ({self.depth_violations}/5): '
                    f'{self.current_depth:.2f}m > {self.max_depth}m'
                )
            elif self.current_depth < self.min_depth:
                self.depth_violations += 1
                if self.depth_violations > 5:
                    self.trigger_emergency(
                        f"Depth too shallow: {self.current_depth:.2f}m < {self.min_depth}m"
                    )
                    return
                self.get_logger().warn(
                    f'⚠️ Depth warning ({self.depth_violations}/5): '
                    f'{self.current_depth:.2f}m < {self.min_depth}m'
                )
            else:
                # Depth is safe, reset counter
                self.depth_violations = max(0, self.depth_violations - 1)
        
        # ========================================
        # CHECK 4: Orientation Limits
        # ========================================
        if self.last_imu_time:  # Only check if we have IMU data
            roll_abs = abs(self.current_roll)
            pitch_abs = abs(self.current_pitch)
            
            if roll_abs > self.max_roll or pitch_abs > self.max_pitch:
                self.tilt_violations += 1
                if self.tilt_violations > 10:  # 1.0s at 10Hz
                    self.trigger_emergency(
                        f"Excessive tilt: Roll={self.current_roll:.1f}° "
                        f"Pitch={self.current_pitch:.1f}° (max: ±{self.max_roll}°)"
                    )
                    return
                self.get_logger().warn(
                    f'⚠️ Tilt warning ({self.tilt_violations}/10): '
                    f'R={self.current_roll:.1f}° P={self.current_pitch:.1f}°'
                )
            else:
                # Orientation is safe, reset counter
                self.tilt_violations = max(0, self.tilt_violations - 1)
        
        # ========================================
        # PERIODIC STATUS (every 10 seconds)
        # ========================================
        if int(mission_time) % 10 == 0:
            self.get_logger().info(
                f'✅ Safe | Depth: {self.current_depth:.2f}m | '
                f'R: {self.current_roll:.1f}° P: {self.current_pitch:.1f}° '
                f'Y: {self.current_yaw:.1f}° | Time: {mission_time:.0f}s',
                throttle_duration_sec=9.5
            )
    
    def trigger_emergency(self, reason: str):
        """Activate emergency protocols"""
        if self.emergency_triggered:
            return
        
        self.emergency_triggered = True
        
        self.get_logger().error('')
        self.get_logger().error('='*70)
        self.get_logger().error(f'🚨 EMERGENCY STOP TRIGGERED')
        self.get_logger().error(f'   Reason: {reason}')
        self.get_logger().error('='*70)
        self.get_logger().error('')
        
        # Publish emergency flag
        self.emergency_pub.publish(Bool(data=True))
        
        # Immediately send neutral PWM
        self.send_emergency_stop()
    
    def emergency_stop_loop(self):
        """Continuously send stop commands when in emergency state"""
        if self.emergency_triggered:
            self.send_emergency_stop()
    
    def send_emergency_stop(self):
        """Send neutral PWM to all 8 thrusters"""
        emergency_msg = UInt16MultiArray()
        emergency_msg.data = self.NEUTRAL_PWM
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
        node.get_logger().warn('Shutdown: sending emergency stop')
        for _ in range(5):  # Send multiple times for safety
            stop_msg = UInt16MultiArray()
            stop_msg.data = node.NEUTRAL_PWM
            node.emergency_pwm_pub.publish(stop_msg)
            time.sleep(0.05)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
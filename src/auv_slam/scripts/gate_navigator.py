#!/usr/bin/env python3
"""
Combined Gate Navigator with Flare Avoidance
HIGH THRUST VERSION - Aggressive speeds with proper alignment
Handles gate loss scenarios
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist, Vector3
import time
import math


class GateFlareNavigator(Node):
    def __init__(self):
        super().__init__('gate_flare_navigator')
        
        # State machine
        self.STABILIZING = 0
        self.SEARCHING_GATE = 1
        self.APPROACHING_GATE = 2
        self.ALIGNING_GATE = 3
        self.FLARE_EMERGENCY_AVOID = 4
        self.FLARE_CONTROLLED_AVOID = 5
        self.PASSING_GATE = 6
        self.GATE_LOST_RECOVERY = 7  # NEW: Handle gate loss
        self.COMPLETED = 8
        
        self.state = self.STABILIZING
        
        # Navigation parameters
        self.TARGET_DEPTH = 0.6
        self.DEPTH_TOLERANCE = 0.1
        
        # Gate parameters
        self.GATE_APPROACH_DISTANCE = 2.0
        self.GATE_ALIGNMENT_DISTANCE = 1.0
        self.GATE_PASSING_DISTANCE = 0.5
        self.GATE_PASSING_DURATION = 4.0  # Reduced from 5s
        
        # Flare parameters
        self.FLARE_DANGER_DISTANCE = 0.5
        self.FLARE_AVOID_DISTANCE = 1.0
        self.FLARE_SAFE_DISTANCE = 1.3
        
        self.SEARCH_SPEED = 0.7          # 1640 PWM
        self.APPROACH_SPEED_MAX = 0.95   # 1690 PWM
        self.APPROACH_SPEED_MIN = 0.5    # 1600 PWM
        self.ALIGNMENT_SPEED_MAX = 0.45  # 1590 PWM
        self.ALIGNMENT_SPEED_MIN = 0.2   # 1540 PWM
        self.PASSING_SPEED = 1.0         # 1700 PWM
        self.AVOID_SPEED = 0.6           # 1620 PWM
        self.RECOVERY_SPEED = 0.4        # 1580 PWM
        
        self.SEARCH_YAW_RATE = 0.4
        self.APPROACH_YAW_GAIN = 3.0
        self.ALIGNMENT_YAW_GAIN = 4.0   # Very aggressive for precise alignment
        
        # Alignment thresholds
        self.ROUGH_ALIGNMENT = 0.2      # Good enough to approach
        self.PRECISE_ALIGNMENT = 0.06   # Was 0.08
        
        # Gate loss handling
        self.GATE_LOST_TIMEOUT = 2.0    # Time before entering recovery
        self.RECOVERY_TIMEOUT = 5.0     # Max recovery time
        
        # State variables
        self.gate_detected = False
        self.gate_alignment_error = 0.0
        self.gate_distance = 999.0
        self.last_gate_detection_time = 0.0
        self.last_known_alignment = 0.0
        
        self.flare_detected = False
        self.flare_direction = 0.0
        self.flare_distance = 999.0
        self.flare_in_danger = False
        
        self.current_depth = 0.0
        self.current_yaw = 0.0
        self.pre_avoidance_state = None
        self.avoidance_target_yaw = 0.0
        
        self.state_start_time = time.time()
        self.passing_start_time = 0.0
        self.total_flare_avoidances = 0
        
        # Subscriptions - Gate
        self.create_subscription(Bool, '/gate/detected', 
                                self.gate_detected_callback, 10)
        self.create_subscription(Float32, '/gate/alignment_error',
                                self.gate_alignment_callback, 10)
        self.create_subscription(Float32, '/gate/estimated_distance',
                                self.gate_distance_callback, 10)
        
        # Subscriptions - Flare
        self.create_subscription(Bool, '/flare/detected', 
                                self.flare_detected_callback, 10)
        self.create_subscription(Float32, '/flare/direction_angle',
                                self.flare_direction_callback, 10)
        self.create_subscription(Float32, '/flare/estimated_distance',
                                self.flare_distance_callback, 10)
        self.create_subscription(Bool, '/flare/danger_zone',
                                self.flare_danger_callback, 10)
        
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
        self.get_logger().info('🎯 Gate Navigator - HIGH THRUST MODE')
        self.get_logger().info('='*70)
        self.get_logger().info(f'  Search Speed: {self.SEARCH_SPEED} (PWM ~{1500 + int(self.SEARCH_SPEED*200)})')
        self.get_logger().info(f'  Approach Speed: {self.APPROACH_SPEED_MIN}-{self.APPROACH_SPEED_MAX}')
        self.get_logger().info(f'  Passing Speed: {self.PASSING_SPEED} (PWM ~{1500 + int(self.PASSING_SPEED*200)})')
        self.get_logger().info('='*70)
    
    def gate_detected_callback(self, msg: Bool):
        was_detected = self.gate_detected
        self.gate_detected = msg.data
        
        if self.gate_detected:
            self.last_gate_detection_time = time.time()
            
            if not was_detected and self.state in [self.SEARCHING_GATE, self.GATE_LOST_RECOVERY]:
                self.get_logger().info(f'🎯 GATE DETECTED/REACQUIRED at {self.gate_distance:.2f}m')
    
    def gate_alignment_callback(self, msg: Float32):
        self.gate_alignment_error = msg.data
        if self.gate_detected:
            self.last_known_alignment = msg.data
    
    def gate_distance_callback(self, msg: Float32):
        self.gate_distance = msg.data
    
    def flare_detected_callback(self, msg: Bool):
        was_detected = self.flare_detected
        self.flare_detected = msg.data
        
        if not was_detected and self.flare_detected:
            self.get_logger().warn(f'🔴 FLARE DETECTED at {self.flare_distance:.2f}m!')
    
    def flare_direction_callback(self, msg: Float32):
        self.flare_direction = msg.data
    
    def flare_distance_callback(self, msg: Float32):
        self.flare_distance = msg.data
    
    def flare_danger_callback(self, msg: Bool):
        self.flare_in_danger = msg.data
    
    def depth_callback(self, msg: Float32):
        self.current_depth = msg.data
    
    def ypr_callback(self, msg: Vector3):
        self.current_yaw = math.radians(msg.x)
    
    def control_loop(self):
        cmd = Twist()
        
        # PRIORITY 1: Check for flare threats
        if self.state not in [self.COMPLETED, self.PASSING_GATE]:
            if self.check_flare_threat():
                return
        
        # Depth control
        cmd.linear.z = self.compute_depth_control()
        
        # PRIORITY 2: Execute current state
        if self.state == self.STABILIZING:
            cmd = self.stabilizing(cmd)
        elif self.state == self.SEARCHING_GATE:
            cmd = self.searching_gate(cmd)
        elif self.state == self.APPROACHING_GATE:
            cmd = self.approaching_gate(cmd)
        elif self.state == self.ALIGNING_GATE:
            cmd = self.aligning_gate(cmd)
        elif self.state == self.FLARE_EMERGENCY_AVOID:
            cmd = self.flare_emergency_avoid(cmd)
        elif self.state == self.FLARE_CONTROLLED_AVOID:
            cmd = self.flare_controlled_avoid(cmd)
        elif self.state == self.PASSING_GATE:
            cmd = self.passing_gate(cmd)
        elif self.state == self.GATE_LOST_RECOVERY:
            cmd = self.gate_lost_recovery(cmd)
        elif self.state == self.COMPLETED:
            cmd = self.completed(cmd)
        
        self.cmd_vel_pub.publish(cmd)
    
    def check_flare_threat(self) -> bool:
        """Check if flare requires immediate attention"""
        
        if self.state in [self.FLARE_EMERGENCY_AVOID, self.FLARE_CONTROLLED_AVOID]:
            return False
        
        if self.flare_detected:
            if self.flare_in_danger or self.flare_distance < self.FLARE_DANGER_DISTANCE:
                self.get_logger().error('🚨 FLARE EMERGENCY - IMMEDIATE AVOIDANCE!')
                self.pre_avoidance_state = self.state
                self.total_flare_avoidances += 1
                self.calculate_avoidance_heading()
                self.transition_to(self.FLARE_EMERGENCY_AVOID)
                return True
            
            elif self.flare_distance < self.FLARE_AVOID_DISTANCE:
                self.get_logger().warn('⚠️ Flare detected - controlled avoidance')
                self.pre_avoidance_state = self.state
                self.total_flare_avoidances += 1
                self.calculate_avoidance_heading()
                self.transition_to(self.FLARE_CONTROLLED_AVOID)
                return True
        
        return False
    
    def calculate_avoidance_heading(self):
        """Calculate which way to turn to avoid flare"""
        if self.flare_direction > 0:
            self.avoidance_target_yaw = self.current_yaw - math.radians(70)
            self.get_logger().warn('Avoiding: Turning LEFT')
        else:
            self.avoidance_target_yaw = self.current_yaw + math.radians(70)
            self.get_logger().warn('Avoiding: Turning RIGHT')
    
    def compute_depth_control(self) -> float:
        depth_error = self.TARGET_DEPTH - self.current_depth
        if abs(depth_error) < self.DEPTH_TOLERANCE:
            return 0.0
        return max(-0.6, min(depth_error * 0.8, 0.6))
    
    def stabilizing(self, cmd: Twist) -> Twist:
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        
        depth_stable = abs(self.TARGET_DEPTH - self.current_depth) < 0.15
        elapsed = time.time() - self.state_start_time
        
        if depth_stable and elapsed > 3.0:
            self.get_logger().info('✅ Stabilized - Searching for gate')
            self.transition_to(self.SEARCHING_GATE)
        
        return cmd
    
    def searching_gate(self, cmd: Twist) -> Twist:
        if self.gate_detected and self.gate_distance < 999:
            self.get_logger().info('🎯 Gate found - approaching')
            self.transition_to(self.APPROACHING_GATE)
            return cmd
        
        # FAST search pattern
        cmd.linear.x = self.SEARCH_SPEED  # 0.6 → 1620 PWM
        elapsed = time.time() - self.state_start_time
        sweep_phase = (elapsed % 8.0) / 8.0
        cmd.angular.z = 0.3 if sweep_phase < 0.5 else -0.3
        
        return cmd
    
    def approaching_gate(self, cmd: Twist) -> Twist:
        """Approach gate with DISTANCE-BASED SPEED RAMPING"""
        
        # Check if gate lost
        time_since_gate = time.time() - self.last_gate_detection_time
        if not self.gate_detected and time_since_gate > self.GATE_LOST_TIMEOUT:
            self.get_logger().warn('⚠️ Gate lost - entering recovery')
            self.transition_to(self.GATE_LOST_RECOVERY)
            return cmd
        
        # Use last known values if gate temporarily not visible
        alignment = self.gate_alignment_error if self.gate_detected else self.last_known_alignment
        distance = self.gate_distance if self.gate_detected else 1.5
        
        # Transition based on distance
        if distance <= self.GATE_ALIGNMENT_DISTANCE:
            self.get_logger().info('📍 Close - aligning with gate')
            self.transition_to(self.ALIGNING_GATE)
            return cmd
        
        # DISTANCE-BASED SPEED RAMPING
        # Far (2.0m): 0.8 speed → 1660 PWM
        # Medium (1.0m): 0.4 speed → 1580 PWM
        distance_ratio = (distance - self.GATE_ALIGNMENT_DISTANCE) / (self.GATE_APPROACH_DISTANCE - self.GATE_ALIGNMENT_DISTANCE)
        distance_ratio = max(0.0, min(1.0, distance_ratio))
        
        speed = self.APPROACH_SPEED_MIN + distance_ratio * (self.APPROACH_SPEED_MAX - self.APPROACH_SPEED_MIN)
        
        cmd.linear.x = speed
        cmd.angular.z = -alignment * self.APPROACH_YAW_GAIN
        
        return cmd
    
    def aligning_gate(self, cmd: Twist) -> Twist:
        """PRECISE ALIGNMENT before passing"""
        
        # Check if gate lost
        time_since_gate = time.time() - self.last_gate_detection_time
        if not self.gate_detected and time_since_gate > self.GATE_LOST_TIMEOUT:
            self.get_logger().warn('⚠️ Gate lost during alignment')
            self.transition_to(self.GATE_LOST_RECOVERY)
            return cmd
        
        # Use last known values if gate temporarily not visible
        alignment = self.gate_alignment_error if self.gate_detected else self.last_known_alignment
        distance = self.gate_distance if self.gate_detected else 0.8
        
        # Check if ready to pass
        if distance <= self.GATE_PASSING_DISTANCE and abs(alignment) < self.PRECISE_ALIGNMENT:
            self.get_logger().info('🚀 PERFECTLY ALIGNED - PASSING THROUGH GATE!')
            self.passing_start_time = time.time()
            self.transition_to(self.PASSING_GATE)
            return cmd
        
        # DISTANCE-BASED SPEED for alignment phase
        # Far edge (1.0m): 0.4 speed → 1580 PWM
        # Very close (0.5m): 0.15 speed → 1530 PWM
        distance_ratio = (distance - self.GATE_PASSING_DISTANCE) / (self.GATE_ALIGNMENT_DISTANCE - self.GATE_PASSING_DISTANCE)
        distance_ratio = max(0.0, min(1.0, distance_ratio))
        
        speed = self.ALIGNMENT_SPEED_MIN + distance_ratio * (self.ALIGNMENT_SPEED_MAX - self.ALIGNMENT_SPEED_MIN)
        
        # AGGRESSIVE TURNING for alignment
        cmd.linear.x = speed if abs(alignment) < self.ROUGH_ALIGNMENT else speed * 0.5
        cmd.angular.z = -alignment * self.ALIGNMENT_YAW_GAIN
        
        return cmd
    
    def passing_gate(self, cmd: Twist) -> Twist:
        """Pass through gate with HIGH SPEED"""
        
        elapsed = time.time() - self.passing_start_time
        
        if elapsed >= self.GATE_PASSING_DURATION:
            self.get_logger().info('='*70)
            self.get_logger().info('🎉 GATE PASSAGE COMPLETE!')
            self.get_logger().info(f'   Flares Avoided: {self.total_flare_avoidances}')
            self.get_logger().info('='*70)
            self.transition_to(self.COMPLETED)
            return cmd
        
        # FAST passing speed with minimal correction
        cmd.linear.x = self.PASSING_SPEED  # 0.8 → 1660 PWM
        
        # Only minor corrections if gate still visible
        if self.gate_detected and abs(self.gate_alignment_error) > 0.15:
            cmd.angular.z = -self.gate_alignment_error * 1.0
        else:
            cmd.angular.z = 0.0
        
        remaining = self.GATE_PASSING_DURATION - elapsed
        self.get_logger().info(
            f'🚀 PASSING: {remaining:.1f}s remaining',
            throttle_duration_sec=0.5
        )
        return cmd
    
    def gate_lost_recovery(self, cmd: Twist) -> Twist:
        """Recover from gate loss"""
        
        elapsed = time.time() - self.state_start_time
        
        # Check if gate reacquired
        if self.gate_detected and self.gate_distance < 999:
            self.get_logger().info('✅ Gate reacquired - resuming approach')
            self.transition_to(self.APPROACHING_GATE)
            return cmd
        
        # Recovery timeout
        if elapsed > self.RECOVERY_TIMEOUT:
            self.get_logger().warn('Recovery timeout - returning to search')
            self.transition_to(self.SEARCHING_GATE)
            return cmd
        
        # SLOW FORWARD while turning toward last known position
        cmd.linear.x = self.RECOVERY_SPEED  # 0.3 → 1560 PWM
        
        # Turn toward last known alignment
        cmd.angular.z = -self.last_known_alignment * 1.5
        
        self.get_logger().warn(
            f'🔍 Recovering gate... ({elapsed:.1f}s)',
            throttle_duration_sec=0.5
        )
        
        return cmd
    
    def flare_emergency_avoid(self, cmd: Twist) -> Twist:
        """Emergency flare avoidance"""
        
        if not self.flare_detected or self.flare_distance > self.FLARE_SAFE_DISTANCE:
            self.get_logger().info('✅ Emergency cleared - resuming mission')
            self.transition_to(self.pre_avoidance_state)
            return cmd
        
        yaw_error = self.normalize_angle(self.avoidance_target_yaw - self.current_yaw)
        
        if abs(yaw_error) > math.radians(15):
            cmd.linear.x = 0.0
            cmd.angular.z = yaw_error * 3.0
        else:
            cmd.linear.x = self.AVOID_SPEED  # 0.5 → 1600 PWM
            cmd.angular.z = 0.0
        
        return cmd
    
    def flare_controlled_avoid(self, cmd: Twist) -> Twist:
        """Controlled flare avoidance"""
        
        if not self.flare_detected or self.flare_distance > self.FLARE_SAFE_DISTANCE:
            self.get_logger().info('✅ Flare cleared - resuming mission')
            self.transition_to(self.pre_avoidance_state)
            return cmd
        
        if self.flare_distance < self.FLARE_DANGER_DISTANCE:
            self.get_logger().error('⚠️ Flare too close - EMERGENCY!')
            self.transition_to(self.FLARE_EMERGENCY_AVOID)
            return cmd
        
        yaw_error = self.normalize_angle(self.avoidance_target_yaw - self.current_yaw)
        
        if abs(yaw_error) > math.radians(10):
            cmd.linear.x = 0.2
            cmd.angular.z = yaw_error * 2.0
        else:
            cmd.linear.x = self.AVOID_SPEED  # 0.5 → 1600 PWM
            cmd.angular.z = 0.0
        
        return cmd
    
    def completed(self, cmd: Twist) -> Twist:
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        return cmd
    
    def normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def transition_to(self, new_state: int):
        state_names = {
            self.STABILIZING: 'STABILIZING',
            self.SEARCHING_GATE: 'SEARCHING_GATE',
            self.APPROACHING_GATE: 'APPROACHING_GATE',
            self.ALIGNING_GATE: 'ALIGNING_GATE',
            self.FLARE_EMERGENCY_AVOID: 'FLARE_EMERGENCY_AVOID',
            self.FLARE_CONTROLLED_AVOID: 'FLARE_CONTROLLED_AVOID',
            self.PASSING_GATE: 'PASSING_GATE',
            self.GATE_LOST_RECOVERY: 'GATE_LOST_RECOVERY',
            self.COMPLETED: 'COMPLETED'
        }
        
        old_name = state_names.get(self.state, 'UNKNOWN')
        self.state = new_state
        self.state_start_time = time.time()
        new_name = state_names.get(new_state, 'UNKNOWN')
        
        self.get_logger().info(f'🔄 STATE: {old_name} → {new_name}')


def main(args=None):
    rclpy.init(args=args)
    node = GateFlareNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_cmd = Twist()
        node.cmd_vel_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
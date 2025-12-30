#!/usr/bin/env python3
"""
Combined Gate Navigator with Flare Avoidance
Primary mission: Pass through gate
Secondary: Avoid red flares while navigating
Priority: Safety (flare avoidance) > Mission (gate passing)
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
        
        # State machine - Priority based
        self.STABILIZING = 0
        self.SEARCHING_GATE = 1
        self.APPROACHING_GATE = 2
        self.ALIGNING_GATE = 3
        self.FLARE_EMERGENCY_AVOID = 4  # Highest priority
        self.FLARE_CONTROLLED_AVOID = 5
        self.PASSING_GATE = 6
        self.COMPLETED = 7
        
        self.state = self.STABILIZING
        
        # Navigation parameters
        self.TARGET_DEPTH = 1.0
        self.DEPTH_TOLERANCE = 0.1
        
        # Gate parameters
        self.GATE_APPROACH_DISTANCE = 2.0
        self.GATE_ALIGNMENT_DISTANCE = 1.0
        self.GATE_PASSING_DISTANCE = 0.5
        self.GATE_PASSING_DURATION = 5.0
        
        # Flare parameters
        self.FLARE_DANGER_DISTANCE = 0.5   # Emergency
        self.FLARE_AVOID_DISTANCE = 1.0    # Controlled avoid
        self.FLARE_SAFE_DISTANCE = 1.3     # Resume mission
        
        # Speed parameters
        self.SEARCH_SPEED = 0.3
        self.APPROACH_SPEED = 0.4
        self.PASSING_SPEED = 0.6
        self.AVOID_SPEED = 0.3
        
        # Gate state variables
        self.gate_detected = False
        self.gate_alignment_error = 0.0
        self.gate_distance = 999.0
        
        # Flare state variables
        self.flare_detected = False
        self.flare_direction = 0.0
        self.flare_distance = 999.0
        self.flare_in_danger = False
        
        # Navigation state
        self.current_depth = 0.0
        self.current_yaw = 0.0
        self.pre_avoidance_state = None
        self.avoidance_target_yaw = 0.0
        
        self.state_start_time = time.time()
        self.passing_start_time = 0.0
        
        # Statistics
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
        self.get_logger().info('🎯 Combined Gate + Flare Navigator Started')
        self.get_logger().info('='*70)
        self.get_logger().info('  Priority: Flare Avoidance > Gate Mission')
        self.get_logger().info('='*70)
    
    # ========================================
    # CALLBACKS - Gate
    # ========================================
    
    def gate_detected_callback(self, msg: Bool):
        was_detected = self.gate_detected
        self.gate_detected = msg.data
        
        if not was_detected and self.gate_detected:
            if self.state == self.SEARCHING_GATE:
                self.get_logger().info(f'🎯 GATE DETECTED at {self.gate_distance:.2f}m')
    
    def gate_alignment_callback(self, msg: Float32):
        self.gate_alignment_error = msg.data
    
    def gate_distance_callback(self, msg: Float32):
        self.gate_distance = msg.data
    
    # ========================================
    # CALLBACKS - Flare
    # ========================================
    
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
    
    # ========================================
    # CALLBACKS - Sensors
    # ========================================
    
    def depth_callback(self, msg: Float32):
        self.current_depth = msg.data
    
    def ypr_callback(self, msg: Vector3):
        self.current_yaw = math.radians(msg.x)
    
    # ========================================
    # CONTROL LOOP
    # ========================================
    
    def control_loop(self):
        cmd = Twist()
        
        # PRIORITY 1: Check for flare threats (overrides everything except COMPLETED)
        if self.state != self.COMPLETED:
            if self.check_flare_threat():
                # Flare threat detected - save current state and avoid
                return
        
        # Depth control (always active)
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
        elif self.state == self.COMPLETED:
            cmd = self.completed(cmd)
        
        self.cmd_vel_pub.publish(cmd)
    
    # ========================================
    # FLARE THREAT DETECTION
    # ========================================
    
    def check_flare_threat(self) -> bool:
        """Check if flare requires immediate attention"""
        
        # Already in avoidance state
        if self.state in [self.FLARE_EMERGENCY_AVOID, self.FLARE_CONTROLLED_AVOID]:
            return False
        
        # Check for flare threats
        if self.flare_detected:
            # EMERGENCY: Flare too close
            if self.flare_in_danger or self.flare_distance < self.FLARE_DANGER_DISTANCE:
                self.get_logger().error('🚨 FLARE EMERGENCY - IMMEDIATE AVOIDANCE!')
                self.pre_avoidance_state = self.state
                self.total_flare_avoidances += 1
                self.calculate_avoidance_heading()
                self.transition_to(self.FLARE_EMERGENCY_AVOID)
                return True
            
            # CONTROLLED: Flare at warning distance
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
            # Flare on RIGHT - turn LEFT
            self.avoidance_target_yaw = self.current_yaw - math.radians(70)
            self.get_logger().warn('Avoiding: Turning LEFT')
        else:
            # Flare on LEFT - turn RIGHT
            self.avoidance_target_yaw = self.current_yaw + math.radians(70)
            self.get_logger().warn('Avoiding: Turning RIGHT')
    
    # ========================================
    # DEPTH CONTROL
    # ========================================
    
    def compute_depth_control(self) -> float:
        depth_error = self.TARGET_DEPTH - self.current_depth
        if abs(depth_error) < self.DEPTH_TOLERANCE:
            return 0.0
        return max(-0.6, min(depth_error * 0.8, 0.6))
    
    # ========================================
    # STATE HANDLERS - Gate Mission
    # ========================================
    
    def stabilizing(self, cmd: Twist) -> Twist:
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        
        depth_stable = abs(self.TARGET_DEPTH - self.current_depth) < 0.15
        elapsed = time.time() - self.state_start_time
        
        if depth_stable and elapsed > 3.0:
            self.get_logger().info('✅ Stabilized - Searching for gate')
            self.transition_to(self.SEARCHING_GATE)
        elif int(elapsed) % 2 == 0:
            self.get_logger().info(
                f'⏳ Stabilizing... {self.current_depth:.2f}m',
                throttle_duration_sec=1.9
            )
        return cmd
    
    def searching_gate(self, cmd: Twist) -> Twist:
        if self.gate_detected and self.gate_distance < 999:
            self.get_logger().info('🎯 Gate found - approaching')
            self.transition_to(self.APPROACHING_GATE)
            return cmd
        
        # Search pattern
        cmd.linear.x = self.SEARCH_SPEED
        elapsed = time.time() - self.state_start_time
        sweep_phase = (elapsed % 8.0) / 8.0
        cmd.angular.z = 0.2 if sweep_phase < 0.5 else -0.2
        
        if int(elapsed) % 3 == 0:
            self.get_logger().info(
                f'🔍 Searching gate... ({elapsed:.0f}s)',
                throttle_duration_sec=2.9
            )
        return cmd
    
    def approaching_gate(self, cmd: Twist) -> Twist:
        if not self.gate_detected:
            self.get_logger().warn('Gate lost - returning to search')
            self.transition_to(self.SEARCHING_GATE)
            return cmd
        
        if self.gate_distance <= self.GATE_ALIGNMENT_DISTANCE:
            self.get_logger().info('📍 Close - aligning with gate')
            self.transition_to(self.ALIGNING_GATE)
            return cmd
        
        cmd.linear.x = self.APPROACH_SPEED
        cmd.angular.z = -self.gate_alignment_error * 1.0
        
        if int((time.time() - self.state_start_time) * 2) % 3 == 0:
            self.get_logger().info(
                f'➡️ Approaching gate: {self.gate_distance:.2f}m',
                throttle_duration_sec=1.4
            )
        return cmd
    
    def aligning_gate(self, cmd: Twist) -> Twist:
        if not self.gate_detected:
            self.get_logger().warn('Gate lost during alignment')
            self.transition_to(self.SEARCHING_GATE)
            return cmd
        
        if self.gate_distance <= self.GATE_PASSING_DISTANCE:
            if abs(self.gate_alignment_error) < 0.15:
                self.get_logger().info('🚀 ALIGNED - PASSING THROUGH GATE!')
                self.passing_start_time = time.time()
                self.transition_to(self.PASSING_GATE)
                return cmd
        
        if abs(self.gate_alignment_error) > 0.2:
            cmd.linear.x = 0.1
            cmd.angular.z = -self.gate_alignment_error * 3.0
        else:
            cmd.linear.x = 0.2
            cmd.angular.z = -self.gate_alignment_error * 2.0
        
        return cmd
    
    def passing_gate(self, cmd: Twist) -> Twist:
        elapsed = time.time() - self.passing_start_time
        
        if elapsed >= self.GATE_PASSING_DURATION:
            self.get_logger().info('='*70)
            self.get_logger().info('🎉 GATE PASSAGE COMPLETE!')
            self.get_logger().info(f'   Flares Avoided: {self.total_flare_avoidances}')
            self.get_logger().info('='*70)
            self.transition_to(self.COMPLETED)
            return cmd
        
        cmd.linear.x = self.PASSING_SPEED
        cmd.angular.z = 0.0
        
        remaining = self.GATE_PASSING_DURATION - elapsed
        self.get_logger().info(
            f'🚀 PASSING: {remaining:.1f}s remaining',
            throttle_duration_sec=0.5
        )
        return cmd
    
    # ========================================
    # STATE HANDLERS - Flare Avoidance
    # ========================================
    
    def flare_emergency_avoid(self, cmd: Twist) -> Twist:
        """Emergency flare avoidance with aggressive maneuver"""
        
        # Check if flare cleared
        if not self.flare_detected or self.flare_distance > self.FLARE_SAFE_DISTANCE:
            self.get_logger().info('✅ Emergency cleared - resuming mission')
            self.transition_to(self.pre_avoidance_state)
            return cmd
        
        # Calculate yaw to target
        yaw_error = self.normalize_angle(self.avoidance_target_yaw - self.current_yaw)
        
        if abs(yaw_error) > math.radians(15):
            # Still turning
            cmd.linear.x = 0.0  # Stop while turning in emergency
            cmd.angular.z = yaw_error * 3.0  # Aggressive turn
            
            self.get_logger().error(
                f'🚨 EMERGENCY AVOID: Turning {math.degrees(yaw_error):.1f}°',
                throttle_duration_sec=0.5
            )
        else:
            # Turned - move away quickly
            cmd.linear.x = self.AVOID_SPEED
            cmd.angular.z = 0.0
            
            self.get_logger().error(
                f'🚨 EMERGENCY AVOID: Escaping | Dist: {self.flare_distance:.2f}m',
                throttle_duration_sec=0.5
            )
        
        return cmd
    
    def flare_controlled_avoid(self, cmd: Twist) -> Twist:
        """Controlled flare avoidance"""
        
        # Check if flare cleared
        if not self.flare_detected or self.flare_distance > self.FLARE_SAFE_DISTANCE:
            self.get_logger().info('✅ Flare cleared - resuming mission')
            self.transition_to(self.pre_avoidance_state)
            return cmd
        
        # If flare got closer - escalate to emergency
        if self.flare_distance < self.FLARE_DANGER_DISTANCE:
            self.get_logger().error('⚠️ Flare too close - EMERGENCY!')
            self.transition_to(self.FLARE_EMERGENCY_AVOID)
            return cmd
        
        # Calculate yaw to target
        yaw_error = self.normalize_angle(self.avoidance_target_yaw - self.current_yaw)
        
        if abs(yaw_error) > math.radians(10):
            cmd.linear.x = 0.15
            cmd.angular.z = yaw_error * 2.0
            
            self.get_logger().warn(
                f'⚠️ Avoiding: Turning {math.degrees(yaw_error):.1f}°',
                throttle_duration_sec=0.7
            )
        else:
            cmd.linear.x = self.AVOID_SPEED
            cmd.angular.z = 0.0
            
            self.get_logger().warn(
                f'⚠️ Avoiding: Moving | Dist: {self.flare_distance:.2f}m',
                throttle_duration_sec=0.7
            )
        
        return cmd
    
    # ========================================
    # STATE HANDLERS - Mission End
    # ========================================
    
    def completed(self, cmd: Twist) -> Twist:
        if not hasattr(self, '_completion_logged'):
            self.get_logger().info('='*70)
            self.get_logger().info('🏆 MISSION COMPLETE!')
            self.get_logger().info('='*70)
            self._completion_logged = True
        
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        return cmd
    
    # ========================================
    # UTILITIES
    # ========================================
    
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
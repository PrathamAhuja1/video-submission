#!/usr/bin/env python3
"""
Gate Navigator - ALIGNMENT ONLY (NO SURGE)
HIGH THRUST VERSION - Aggressive speeds with proper alignment
Aligns with gate both horizontally and vertically using depth sensor
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist, Vector3, Point
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
        self.GATE_LOST_RECOVERY = 6
        self.ALIGNED = 7
        
        self.state = self.STABILIZING
        
        # Navigation parameters
        self.TARGET_DEPTH = 0.6
        self.DEPTH_TOLERANCE = 0.05  # ±5cm tolerance
        
        # Gate parameters
        self.GATE_APPROACH_DISTANCE = 2.0
        self.GATE_ALIGNMENT_DISTANCE = 1.0
        self.GATE_HEIGHT_METERS = 1.5
        
        # Flare parameters
        self.FLARE_DANGER_DISTANCE = 0.5
        self.FLARE_AVOID_DISTANCE = 1.0
        self.FLARE_SAFE_DISTANCE = 1.3
        
        self.SEARCH_SPEED = 0.7          # Forward speed while searching
        self.APPROACH_SPEED_MAX = 0.95   # Max forward speed when approaching
        self.APPROACH_SPEED_MIN = 0.5    # Min forward speed
        self.ALIGNMENT_SPEED_MAX = 0.45
        self.ALIGNMENT_SPEED_MIN = 0.2
        self.AVOID_SPEED = 0.6
        self.RECOVERY_SPEED = 0.4
        
        self.SEARCH_YAW_RATE = 0.4
        self.APPROACH_YAW_GAIN = 3.0
        self.ALIGNMENT_YAW_GAIN = 4.0
        
        # Vertical/Heave alignment parameters
        self.HEAVE_GAIN = 0.8  # Gain for vertical error control
        self.MAX_HEAVE_SPEED = 0.4  # Max vertical velocity
        
        # Sway parameters (new for 8-thruster setup)
        self.SWAY_GAIN = 0.6  # Gain for sway control
        self.MAX_SWAY_SPEED = 0.5  # Max sway velocity
        
        # Alignment thresholds
        self.ROUGH_ALIGNMENT = 0.2
        self.PRECISE_ALIGNMENT = 0.06   # Horizontal alignment tolerance
        self.PRECISE_VERTICAL = 0.08    # Vertical alignment tolerance
        
        # Gate loss handling
        self.GATE_LOST_TIMEOUT = 2.0
        self.RECOVERY_TIMEOUT = 5.0
        
        # ========================================================================
        # STATE VARIABLES - GATE
        # ========================================================================
        self.gate_detected = False
        self.gate_alignment_error = 0.0         # Horizontal (-1 to +1)
        self.gate_vertical_error = 0.0          # Vertical (-1 to +1)
        self.gate_distance = 999.0
        self.gate_center_x = 0.0                # Pixel coordinates
        self.gate_center_y = 0.0
        self.image_height = 480                 # Default, updated from camera
        self.last_gate_detection_time = 0.0
        self.last_known_alignment = 0.0
        self.last_known_vertical = 0.0
        
        # ========================================================================
        # STATE VARIABLES - DEPTH & POSITION
        # ========================================================================
        self.current_depth = 0.0
        self.current_yaw = 0.0
        self.depth_offset_from_target = 0.0
        self.depth_received = False
        
        # ========================================================================
        # STATE VARIABLES - FLARE
        # ========================================================================
        self.flare_detected = False
        self.flare_direction = 0.0
        self.flare_distance = 999.0
        self.flare_in_danger = False
        self.pre_avoidance_state = None
        self.avoidance_target_yaw = 0.0
        
        # Statistics
        self.state_start_time = time.time()
        self.total_flare_avoidances = 0
        
        # ========================================================================
        # SUBSCRIPTIONS - GATE & VISION
        # ========================================================================
        self.create_subscription(Bool, '/gate/detected', 
                                self.gate_detected_callback, 10)
        self.create_subscription(Float32, '/gate/alignment_error',
                                self.gate_alignment_callback, 10)
        self.create_subscription(Float32, '/gate/estimated_distance',
                                self.gate_distance_callback, 10)
        self.create_subscription(Float32, '/gate/vertical_error',
                                self.gate_vertical_callback, 10)
        self.create_subscription(Point, '/gate/center_point',
                                self.gate_center_callback, 10)
        
        # ========================================================================
        # SUBSCRIPTIONS - SENSORS (CRITICAL FOR DEPTH ALIGNMENT)
        # ========================================================================
        self.depth_sub = self.create_subscription(Float32, '/depth',
                                self.depth_callback, 10)
        self.ypr_sub = self.create_subscription(Vector3, '/vn100/ypr',
                                self.ypr_callback, 10)
        
        # ========================================================================
        # SUBSCRIPTIONS - FLARE
        # ========================================================================
        self.create_subscription(Bool, '/flare/detected', 
                                self.flare_detected_callback, 10)
        self.create_subscription(Float32, '/flare/direction_angle',
                                self.flare_direction_callback, 10)
        self.create_subscription(Float32, '/flare/estimated_distance',
                                self.flare_distance_callback, 10)
        self.create_subscription(Bool, '/flare/danger_zone',
                                self.flare_danger_callback, 10)
        
        # ========================================================================
        # PUBLISHER
        # ========================================================================
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control loop at 20Hz
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('='*70)
        self.get_logger().info('🎯 Gate Navigator - ALIGNMENT ONLY (8 THRUSTERS)')
        self.get_logger().info('='*70)
        self.get_logger().info('  Mission: Find gate and maintain perfect alignment')
        self.get_logger().info('  Horizontal + Vertical + Depth alignment')
        self.get_logger().info('  No surge/passing through gate')
        self.get_logger().info('')
        self.get_logger().info('  Thruster Configuration (8):')
        self.get_logger().info('    [0-1,4-5] = Vertical (Heave/Pitch/Roll)')
        self.get_logger().info('    [2-3] = Surge (Forward)')
        self.get_logger().info('    [6-7] = Sway (Lateral)')
        self.get_logger().info('='*70)
    
    # ============================================================================
    # CALLBACKS - GATE
    # ============================================================================
    
    def gate_detected_callback(self, msg: Bool):
        was_detected = self.gate_detected
        self.gate_detected = msg.data
        
        if self.gate_detected:
            self.last_gate_detection_time = time.time()
            if not was_detected and self.state in [self.SEARCHING_GATE, self.GATE_LOST_RECOVERY]:
                self.get_logger().info(f'🎯 GATE DETECTED at {self.gate_distance:.2f}m')
    
    def gate_alignment_callback(self, msg: Float32):
        self.gate_alignment_error = msg.data
        if self.gate_detected:
            self.last_known_alignment = msg.data
    
    def gate_vertical_callback(self, msg: Float32):
        """Vertical alignment error from detector"""
        self.gate_vertical_error = msg.data
        if self.gate_detected:
            self.last_known_vertical = msg.data
    
    def gate_distance_callback(self, msg: Float32):
        self.gate_distance = msg.data
    
    def gate_center_callback(self, msg: Point):
        """Gate center in image coordinates"""
        self.gate_center_x = msg.x
        self.gate_center_y = msg.y
    
    # ============================================================================
    # CALLBACKS - SENSORS (CRITICAL)
    # ============================================================================
    
    def depth_callback(self, msg: Float32):
        """Receive depth sensor data - CRITICAL FOR VERTICAL ALIGNMENT"""
        self.current_depth = msg.data
        self.depth_received = True
        self.depth_offset_from_target = self.TARGET_DEPTH - self.current_depth
    
    def ypr_callback(self, msg: Vector3):
        """Receive orientation data"""
        self.current_yaw = math.radians(msg.x)
    
    # ============================================================================
    # CALLBACKS - FLARE
    # ============================================================================
    
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
    
    # ============================================================================
    # MAIN CONTROL LOOP
    # ============================================================================
    
    def control_loop(self):
        """Main control loop - runs at 20Hz"""
        
        # Check if we have critical sensor data
        if not self.depth_received:
            self.get_logger().warn(
                'Waiting for depth sensor data on /depth topic',
                throttle_duration_sec=2.0
            )
            return
        
        # PRIORITY 1: Check for flare threats (except in ALIGNED state)
        if self.state not in [self.ALIGNED]:
            if self.check_flare_threat():
                return
        
        cmd = Twist()
        
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
        elif self.state == self.GATE_LOST_RECOVERY:
            cmd = self.gate_lost_recovery(cmd)
        elif self.state == self.ALIGNED:
            cmd = self.aligned(cmd)
        
        self.cmd_vel_pub.publish(cmd)
    
    # ============================================================================
    # FLARE AVOIDANCE
    # ============================================================================
    
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
        """Calculate avoidance direction"""
        if self.flare_direction > 0:
            self.avoidance_target_yaw = self.current_yaw - math.radians(70)
            self.get_logger().warn('Avoiding: Turning LEFT')
        else:
            self.avoidance_target_yaw = self.current_yaw + math.radians(70)
            self.get_logger().warn('Avoiding: Turning RIGHT')
    
    # ============================================================================
    # STATE MACHINES
    # ============================================================================
    
    def stabilizing(self, cmd: Twist) -> Twist:
        """Stabilize depth before searching"""
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        
        # Depth control only
        depth_error = self.TARGET_DEPTH - self.current_depth
        if abs(depth_error) < self.DEPTH_TOLERANCE:
            cmd.linear.z = 0.0
        else:
            cmd.linear.z = depth_error * 0.8
            cmd.linear.z = max(-0.6, min(cmd.linear.z, 0.6))
        
        depth_stable = abs(depth_error) < 0.15
        elapsed = time.time() - self.state_start_time
        
        if depth_stable and elapsed > 3.0:
            self.get_logger().info('✅ Stabilized at target depth - Searching for gate')
            self.transition_to(self.SEARCHING_GATE)
        
        return cmd
    
    def searching_gate(self, cmd: Twist) -> Twist:
        """Search for gate with spiral pattern"""
        if self.gate_detected and self.gate_distance < 999:
            self.get_logger().info('🎯 Gate found - approaching')
            self.transition_to(self.APPROACHING_GATE)
            return cmd
        
        cmd.linear.x = self.SEARCH_SPEED
        cmd.linear.y = 0.0
        
        # Spiral search pattern
        elapsed = time.time() - self.state_start_time
        sweep_phase = (elapsed % 8.0) / 8.0
        cmd.angular.z = 0.3 if sweep_phase < 0.5 else -0.3
        
        # Maintain depth
        depth_error = self.TARGET_DEPTH - self.current_depth
        cmd.linear.z = depth_error * 0.8
        cmd.linear.z = max(-0.6, min(cmd.linear.z, 0.6))
        
        return cmd
    
    def approaching_gate(self, cmd: Twist) -> Twist:
        """Approach gate using horizontal alignment control"""
        
        # Check if gate lost
        time_since_gate = time.time() - self.last_gate_detection_time
        if not self.gate_detected and time_since_gate > self.GATE_LOST_TIMEOUT:
            self.get_logger().warn('⚠️ Gate lost - entering recovery')
            self.transition_to(self.GATE_LOST_RECOVERY)
            return cmd
        
        alignment = self.gate_alignment_error if self.gate_detected else self.last_known_alignment
        distance = self.gate_distance if self.gate_detected else 1.5
        
        # Transition to alignment when close
        if distance <= self.GATE_ALIGNMENT_DISTANCE:
            self.get_logger().info('📍 Close enough - starting fine alignment')
            self.transition_to(self.ALIGNING_GATE)
            return cmd
        
        # Distance-based speed ramping
        distance_ratio = (distance - self.GATE_ALIGNMENT_DISTANCE) / (self.GATE_APPROACH_DISTANCE - self.GATE_ALIGNMENT_DISTANCE)
        distance_ratio = max(0.0, min(1.0, distance_ratio))
        speed = self.APPROACH_SPEED_MIN + distance_ratio * (self.APPROACH_SPEED_MAX - self.APPROACH_SPEED_MIN)
        
        cmd.linear.x = speed
        cmd.linear.y = 0.0
        cmd.angular.z = -alignment * self.APPROACH_YAW_GAIN
        
        # Maintain depth
        depth_error = self.TARGET_DEPTH - self.current_depth
        cmd.linear.z = depth_error * 0.8
        cmd.linear.z = max(-0.6, min(cmd.linear.z, 0.6))
        
        return cmd
    
    def aligning_gate(self, cmd: Twist) -> Twist:
        """PRECISE ALIGNMENT - Horizontal + Vertical + Depth"""
        
        # Check if gate lost
        time_since_gate = time.time() - self.last_gate_detection_time
        if not self.gate_detected and time_since_gate > self.GATE_LOST_TIMEOUT:
            self.get_logger().warn('⚠️ Gate lost during alignment')
            self.transition_to(self.GATE_LOST_RECOVERY)
            return cmd
        
        # Get errors
        h_error = self.gate_alignment_error if self.gate_detected else self.last_known_alignment
        v_error = self.gate_vertical_error if self.gate_detected else self.last_known_vertical
        depth_error = self.TARGET_DEPTH - self.current_depth
        distance = self.gate_distance if self.gate_detected else 0.8
        
        # Check if perfectly aligned on all axes
        h_aligned = abs(h_error) < self.PRECISE_ALIGNMENT
        v_aligned = abs(v_error) < self.PRECISE_VERTICAL
        depth_aligned = abs(depth_error) < self.DEPTH_TOLERANCE
        
        if h_aligned and v_aligned and depth_aligned:
            self.get_logger().info('='*70)
            self.get_logger().info('✅ PERFECTLY ALIGNED!')
            self.get_logger().info(f'   H:{h_error:+.3f} V:{v_error:+.3f} D:{depth_error:+.3f}m')
            self.get_logger().info('   Maintaining alignment...')
            self.get_logger().info('='*70)
            self.transition_to(self.ALIGNED)
            return cmd
        
        # Distance-based speed ramping
        distance_ratio = max(0.0, min(1.0, (distance - 0.3) / (self.GATE_ALIGNMENT_DISTANCE - 0.3)))
        fwd_speed = self.ALIGNMENT_SPEED_MIN + distance_ratio * (self.ALIGNMENT_SPEED_MAX - self.ALIGNMENT_SPEED_MIN)
        
        # Forward surge
        cmd.linear.x = fwd_speed if abs(h_error) < self.ROUGH_ALIGNMENT else fwd_speed * 0.5
        
        # Horizontal alignment (YAW) - with aggressive gain
        cmd.angular.z = -h_error * self.ALIGNMENT_YAW_GAIN
        
        # Vertical alignment (HEAVE/DEPTH)
        heave_cmd = depth_error * self.HEAVE_GAIN
        cmd.linear.z = max(-self.MAX_HEAVE_SPEED, min(heave_cmd, self.MAX_HEAVE_SPEED))
        
        # Lateral alignment (SWAY) - for camera/vision center alignment
        sway_cmd = -v_error * self.SWAY_GAIN
        cmd.linear.y = max(-self.MAX_SWAY_SPEED, min(sway_cmd, self.MAX_SWAY_SPEED))
        
        if int(time.time() * 2) % 4 == 0:  # Log every 2 seconds
            self.get_logger().info(
                f'🎯 ALIGNING: H={h_error:+.3f} V={v_error:+.3f} D={depth_error:+.3f}m | '
                f'Cmd: Fwd={cmd.linear.x:.2f} Sway={cmd.linear.y:.2f} Heave={cmd.linear.z:.2f}',
                throttle_duration_sec=1.9
            )
        
        return cmd
    
    def aligned(self, cmd: Twist) -> Twist:
        """Perfect alignment achieved - maintain position"""
        
        if not self.gate_detected:
            self.get_logger().warn('⚠️ Gate lost after alignment - searching...')
            self.transition_to(self.SEARCHING_GATE)
            return cmd
        
        h_error = self.gate_alignment_error
        v_error = self.gate_vertical_error
        depth_error = self.TARGET_DEPTH - self.current_depth
        
        # Very gentle corrections to maintain alignment
        cmd.linear.x = 0.0  # Stop forward movement
        cmd.linear.y = -v_error * 0.3  # Gentle sway
        cmd.linear.z = depth_error * 0.4  # Gentle heave
        cmd.angular.z = -h_error * 0.8  # Gentle yaw
        
        self.get_logger().info(
            f'✅ ALIGNED | H={h_error:+.3f} V={v_error:+.3f} D={depth_error:+.3f}m',
            throttle_duration_sec=2.0
        )
        
        return cmd
    
    def gate_lost_recovery(self, cmd: Twist) -> Twist:
        """Recover from gate loss"""
        
        elapsed = time.time() - self.state_start_time
        
        if self.gate_detected and self.gate_distance < 999:
            self.get_logger().info('✅ Gate reacquired - resuming approach')
            self.transition_to(self.APPROACHING_GATE)
            return cmd
        
        if elapsed > self.RECOVERY_TIMEOUT:
            self.get_logger().warn('Recovery timeout - returning to search')
            self.transition_to(self.SEARCHING_GATE)
            return cmd
        
        cmd.linear.x = self.RECOVERY_SPEED
        cmd.linear.y = 0.0
        cmd.angular.z = -self.last_known_alignment * 1.5
        
        # Maintain depth
        depth_error = self.TARGET_DEPTH - self.current_depth
        cmd.linear.z = depth_error * 0.8
        cmd.linear.z = max(-0.6, min(cmd.linear.z, 0.6))
        
        self.get_logger().warn(f'🔍 Recovering... ({elapsed:.1f}s)', throttle_duration_sec=0.5)
        
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
            cmd.linear.y = 0.0
            cmd.angular.z = yaw_error * 3.0
        else:
            cmd.linear.x = self.AVOID_SPEED
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
        
        # Maintain depth
        depth_error = self.TARGET_DEPTH - self.current_depth
        cmd.linear.z = depth_error * 0.8
        cmd.linear.z = max(-0.6, min(cmd.linear.z, 0.6))
        
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
            cmd.linear.y = 0.0
            cmd.angular.z = yaw_error * 2.0
        else:
            cmd.linear.x = self.AVOID_SPEED
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
        
        # Maintain depth
        depth_error = self.TARGET_DEPTH - self.current_depth
        cmd.linear.z = depth_error * 0.8
        cmd.linear.z = max(-0.6, min(cmd.linear.z, 0.6))
        
        return cmd
    
    # ============================================================================
    # UTILITY FUNCTIONS
    # ============================================================================
    
    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def transition_to(self, new_state: int):
        """Transition to new state"""
        state_names = {
            self.STABILIZING: 'STABILIZING',
            self.SEARCHING_GATE: 'SEARCHING_GATE',
            self.APPROACHING_GATE: 'APPROACHING_GATE',
            self.ALIGNING_GATE: 'ALIGNING_GATE',
            self.FLARE_EMERGENCY_AVOID: 'FLARE_EMERGENCY_AVOID',
            self.FLARE_CONTROLLED_AVOID: 'FLARE_CONTROLLED_AVOID',
            self.GATE_LOST_RECOVERY: 'GATE_LOST_RECOVERY',
            self.ALIGNED: 'ALIGNED'
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
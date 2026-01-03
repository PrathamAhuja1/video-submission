#!/usr/bin/env python3
"""
Autonomous Gate Alignment Test
Aligns bot with gate center (vertical + horizontal) then surges through
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool, Float32, String
import time


class GateAlignmentTest(Node):
    def __init__(self):
        super().__init__('gate_alignment_test')
        
        # Parameters
        self.declare_parameter('vertical_tolerance', 0.08)  # Vertical alignment tolerance (normalized)
        self.declare_parameter('horizontal_tolerance', 0.08)  # Horizontal alignment tolerance (normalized)
        self.declare_parameter('surge_duration', 5.0)  # Time to surge forward (seconds)
        self.declare_parameter('surge_speed', 0.85)  # Surge speed (0-1)
        self.declare_parameter('alignment_timeout', 30.0)  # Max time to align (seconds)
        self.declare_parameter('heave_gain', 0.5)  # Heave control gain
        self.declare_parameter('yaw_gain', 2.5)  # Yaw control gain
        self.declare_parameter('max_heave_speed', 0.4)  # Max heave speed
        self.declare_parameter('max_yaw_speed', 0.6)  # Max yaw speed
        
        # Get parameters
        self.vertical_tolerance = self.get_parameter('vertical_tolerance').value
        self.horizontal_tolerance = self.get_parameter('horizontal_tolerance').value
        self.surge_duration = self.get_parameter('surge_duration').value
        self.surge_speed = self.get_parameter('surge_speed').value
        self.alignment_timeout = self.get_parameter('alignment_timeout').value
        self.heave_gain = self.get_parameter('heave_gain').value
        self.yaw_gain = self.get_parameter('yaw_gain').value
        self.max_heave_speed = self.get_parameter('max_heave_speed').value
        self.max_yaw_speed = self.get_parameter('max_yaw_speed').value
        
        # State machine
        self.INIT = 0
        self.WAITING_FOR_GATE = 1
        self.ALIGNING = 2
        self.SURGING = 3
        self.COMPLETE = 4
        
        self.state = self.INIT
        self.state_start_time = time.time()
        
        # Sensor data
        self.gate_detected = False
        self.gate_center_x = 0.0  # pixels
        self.gate_center_y = 0.0  # pixels
        self.gate_distance = 999.0
        self.horizontal_alignment_error = 0.0  # -1 to 1
        
        self.image_width = 640  # default
        self.image_height = 480  # default
        self.camera_info_received = False
        
        # Alignment tracking
        self.vertical_aligned = False
        self.horizontal_aligned = False
        self.alignment_start_time = None
        self.stable_alignment_count = 0
        self.stable_alignment_threshold = 20  # frames (~1 second at 20Hz)
        
        # Statistics
        self.frame_count = 0
        
        # Subscriptions
        self.create_subscription(
            Bool,
            '/gate/detected',
            self.gate_detected_callback,
            10
        )
        
        self.create_subscription(
            Point,
            '/gate/center_point',
            self.gate_center_callback,
            10
        )
        
        self.create_subscription(
            Float32,
            '/gate/alignment_error',
            self.alignment_error_callback,
            10
        )
        
        self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/gate_alignment/status', 10)
        
        # Control loop at 20Hz
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('='*70)
        self.get_logger().info('🎯 Gate Alignment Test Started')
        self.get_logger().info('='*70)
        self.get_logger().info(f'  Vertical Tolerance: ±{self.vertical_tolerance:.3f}')
        self.get_logger().info(f'  Horizontal Tolerance: ±{self.horizontal_tolerance:.3f}')
        self.get_logger().info(f'  Surge Duration: {self.surge_duration}s')
        self.get_logger().info(f'  Surge Speed: {self.surge_speed}')
        self.get_logger().info(f'  Alignment Timeout: {self.alignment_timeout}s')
        self.get_logger().info('='*70)
        self.get_logger().info('')
        self.get_logger().info('Sequence:')
        self.get_logger().info('  1. Wait for gate detection')
        self.get_logger().info('  2. Align vertically (heave) and horizontally (yaw)')
        self.get_logger().info('  3. Surge forward through gate')
        self.get_logger().info('='*70)
    
    def gate_detected_callback(self, msg: Bool):
        """Gate detection status"""
        was_detected = self.gate_detected
        self.gate_detected = msg.data
        
        if self.gate_detected and not was_detected:
            self.get_logger().info('✅ Gate detected!')
    
    def gate_center_callback(self, msg: Point):
        """Gate center point in image (x, y in pixels, z as distance)"""
        self.gate_center_x = msg.x
        self.gate_center_y = msg.y
        self.gate_distance = msg.z
    
    def alignment_error_callback(self, msg: Float32):
        """Horizontal alignment error from gate detector"""
        self.horizontal_alignment_error = msg.data
    
    def camera_info_callback(self, msg: CameraInfo):
        """Camera parameters"""
        if not self.camera_info_received:
            self.image_width = msg.width
            self.image_height = msg.height
            self.camera_info_received = True
            self.get_logger().info(f'📷 Camera info: {self.image_width}x{self.image_height}')
    
    def compute_vertical_alignment_error(self) -> float:
        """
        Calculate vertical alignment error
        Returns normalized error: -1 (gate above center) to +1 (gate below center)
        """
        if not self.camera_info_received:
            return 0.0
        
        image_center_y = self.image_height / 2.0
        pixel_error = self.gate_center_y - image_center_y
        
        # Normalize to -1 to +1 range
        normalized_error = pixel_error / (self.image_height / 2.0)
        
        return normalized_error
    
    def control_loop(self):
        """Main control loop"""
        self.frame_count += 1
        cmd = Twist()
        
        # State machine
        if self.state == self.INIT:
            cmd = self.state_init(cmd)
        elif self.state == self.WAITING_FOR_GATE:
            cmd = self.state_waiting_for_gate(cmd)
        elif self.state == self.ALIGNING:
            cmd = self.state_aligning(cmd)
        elif self.state == self.SURGING:
            cmd = self.state_surging(cmd)
        elif self.state == self.COMPLETE:
            cmd = self.state_complete(cmd)
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
    
    def state_init(self, cmd: Twist) -> Twist:
        """Initialization - wait for sensors"""
        elapsed = time.time() - self.state_start_time
        
        if elapsed < 2.0:
            self.publish_status(f"Initializing... {2.0 - elapsed:.1f}s")
            return cmd
        
        if not self.camera_info_received:
            self.publish_status("Waiting for camera info...")
            return cmd
        
        # All sensors ready
        self.get_logger().info('✅ All sensors ready - searching for gate')
        self.transition_to(self.WAITING_FOR_GATE)
        return cmd
    
    def state_waiting_for_gate(self, cmd: Twist) -> Twist:
        """Wait for gate detection"""
        if self.gate_detected and self.gate_distance < 999.0:
            self.get_logger().info(f'🎯 Gate found at {self.gate_distance:.2f}m - starting alignment')
            self.alignment_start_time = time.time()
            self.transition_to(self.ALIGNING)
            return cmd
        
        # Slow rotation to search for gate
        elapsed = time.time() - self.state_start_time
        
        if elapsed < 1.0:
            # Wait 1 second before rotating
            cmd.angular.z = 0.0
        else:
            # Slow rotation
            phase = ((elapsed - 1.0) % 8.0) / 8.0
            cmd.angular.z = 0.3 if phase < 0.5 else -0.3
        
        self.publish_status(f"Searching for gate... {elapsed:.1f}s")
        return cmd
    
    def state_aligning(self, cmd: Twist) -> Twist:
        """Align with gate center (vertical + horizontal)"""
        # Check if gate lost
        if not self.gate_detected:
            self.get_logger().warn('⚠️ Gate lost during alignment')
            self.transition_to(self.WAITING_FOR_GATE)
            return cmd
        
        # Check alignment timeout
        elapsed = time.time() - self.alignment_start_time
        if elapsed > self.alignment_timeout:
            self.get_logger().error('❌ Alignment timeout - aborting')
            self.transition_to(self.COMPLETE)
            return cmd
        
        # Calculate alignment errors
        vertical_error = self.compute_vertical_alignment_error()
        horizontal_error = self.horizontal_alignment_error
        
        # Check if aligned
        self.vertical_aligned = abs(vertical_error) < self.vertical_tolerance
        self.horizontal_aligned = abs(horizontal_error) < self.horizontal_tolerance
        
        both_aligned = self.vertical_aligned and self.horizontal_aligned
        
        if both_aligned:
            self.stable_alignment_count += 1
            if self.stable_alignment_count >= self.stable_alignment_threshold:
                self.get_logger().info('✅ ALIGNED - Starting surge!')
                self.transition_to(self.SURGING)
                return cmd
        else:
            # Reset counter if not aligned
            self.stable_alignment_count = max(0, self.stable_alignment_count - 2)
        
        # Depth control (maintain target depth)
        depth_error = self.target_depth - self.current_depth
        depth_cmd = -depth_error * 0.8
        depth_cmd = max(-0.3, min(depth_cmd, 0.3))
        
        # Vertical alignment using heave (UP/DOWN)
        # Positive vertical_error = gate below center = need to heave DOWN
        # Negative vertical_error = gate above center = need to heave UP
        heave_cmd = vertical_error * self.heave_gain
        heave_cmd = max(-self.max_heave_speed, min(heave_cmd, self.max_heave_speed))
        
        # Combine depth control and vertical alignment
        cmd.linear.z = -(depth_cmd + heave_cmd)
        
        # Horizontal alignment using yaw
        # Positive horizontal_error = gate right of center = need to yaw RIGHT (negative)
        # Negative horizontal_error = gate left of center = need to yaw LEFT (positive)
        yaw_cmd = -horizontal_error * self.yaw_gain
        yaw_cmd = max(-self.max_yaw_speed, min(yaw_cmd, self.max_yaw_speed))
        cmd.angular.z = yaw_cmd
        
        # Status
        status = (
            f"ALIGNING | V:{vertical_error:+.3f} ({'✓' if self.vertical_aligned else '✗'}) | "
            f"H:{horizontal_error:+.3f} ({'✓' if self.horizontal_aligned else '✗'}) | "
            f"Stable:{self.stable_alignment_count}/{self.stable_alignment_threshold} | "
            f"Time:{elapsed:.1f}s"
        )
        self.publish_status(status)
        
        # Periodic detailed logging
        if self.frame_count % 20 == 0:
            self.get_logger().info(
                f'🎯 V_err:{vertical_error:+.3f} H_err:{horizontal_error:+.3f} | '
                f'Heave:{cmd.linear.z:+.2f} Yaw:{cmd.angular.z:+.2f} | '
                f'Stable:{self.stable_alignment_count}/{self.stable_alignment_threshold}',
                throttle_duration_sec=0.9
            )
        
        return cmd
    
    def state_surging(self, cmd: Twist) -> Twist:
        """Surge forward through gate"""
        elapsed = time.time() - self.state_start_time
        
        if elapsed >= self.surge_duration:
            self.get_logger().info('='*70)
            self.get_logger().info('🎉 GATE PASSAGE COMPLETE!')
            self.get_logger().info('='*70)
            self.transition_to(self.COMPLETE)
            return cmd
        
        # Surge forward at configured speed
        cmd.linear.x = self.surge_speed
        
        # Maintain alignment if gate still visible
        if self.gate_detected:
            vertical_error = self.compute_vertical_alignment_error()
            horizontal_error = self.horizontal_alignment_error
            
            # Gentle corrections while surging
            cmd.linear.z = -vertical_error * 0.3
            cmd.angular.z = -horizontal_error * 1.5
        else:
            # Gate lost (probably behind us), maintain straight
            cmd.linear.z = 0.0
            cmd.angular.z = 0.0
        
        remaining = self.surge_duration - elapsed
        self.publish_status(f"SURGING | Speed:{self.surge_speed} | Remaining:{remaining:.1f}s")
        
        if self.frame_count % 10 == 0:
            self.get_logger().info(
                f'🚀 SURGING @ {self.surge_speed} | {remaining:.1f}s remaining',
                throttle_duration_sec=0.4
            )
        
        return cmd
    
    def state_complete(self, cmd: Twist) -> Twist:
        """Mission complete - stop"""
        cmd.linear.x = 0.0
        cmd.linear.z = 0.0
        cmd.angular.z = 0.0
        
        elapsed = time.time() - self.state_start_time
        if elapsed < 2.0:
            self.publish_status("✅ MISSION COMPLETE")
        
        return cmd
    
    def transition_to(self, new_state: int):
        """Transition to new state"""
        state_names = {
            self.INIT: 'INIT',
            self.WAITING_FOR_GATE: 'WAITING_FOR_GATE',
            self.ALIGNING: 'ALIGNING',
            self.SURGING: 'SURGING',
            self.COMPLETE: 'COMPLETE'
        }
        
        old_name = state_names.get(self.state, 'UNKNOWN')
        new_name = state_names.get(new_state, 'UNKNOWN')
        
        self.get_logger().info('')
        self.get_logger().info(f'🔄 STATE: {old_name} → {new_name}')
        self.get_logger().info('')
        
        self.state = new_state
        self.state_start_time = time.time()
        
        # Reset alignment tracking on state change
        if new_state == self.ALIGNING:
            self.stable_alignment_count = 0
    
    def publish_status(self, status_text: str):
        """Publish status message"""
        msg = String()
        msg.data = status_text
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GateAlignmentTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command
        stop_cmd = Twist()
        node.cmd_vel_pub.publish(stop_cmd)
        node.get_logger().info('Emergency stop sent')
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
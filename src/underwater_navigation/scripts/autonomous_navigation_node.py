#!/usr/bin/env python3
"""
Autonomous Navigation Node
Detects white gate and autonomously navigates through it
Sends direct thruster PWM commands for precise control
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math


class AutonomousNavigationNode(Node):
    def __init__(self):
        super().__init__('autonomous_navigation')
        
        self.bridge = CvBridge()
        
        # --- PWM CONSTANTS (mapped to thruster velocities) ---
        self.HEAVE_NEUTRAL = 0.0
        self.SURGE_NEUTRAL = 0.0
        self.SWAY_NEUTRAL = 0.0
        
        # Thruster limits (angular velocity in rad/s)
        self.THRUST_MIN = -15.0
        self.THRUST_MAX = 15.0
        
        # --- VISION SETTINGS ---
        self.FRAME_WIDTH = 1280
        self.FRAME_HEIGHT = 720
        self.CENTER_X = self.FRAME_WIDTH // 2
        self.CENTER_Y = self.FRAME_HEIGHT // 2
        
        # White/Off-white gate detection (HSV)
        self.LOWER_WHITE = np.array([0, 0, 200])
        self.UPPER_WHITE = np.array([180, 40, 255])
        
        # Detection thresholds
        self.MIN_DETECT_AREA = 2000
        self.STOP_DISTANCE_AREA = 100000  # When gate fills most of frame
        
        # Control gains (tuning parameters)
        self.K_SWAY = 0.015      # Horizontal alignment
        self.K_HEAVE = 0.012     # Vertical alignment
        self.K_SURGE = 0.00008   # Forward speed based on distance
        
        # State variables
        self.current_depth = 0.0
        self.gate_detected = False
        self.gate_cx = 0
        self.gate_cy = 0
        self.gate_area = 0
        self.mission_active = True
        
        # Starting position tracking
        self.start_position = None
        self.current_position = None
        
        # --- SUBSCRIBERS ---
        self.camera_sub = self.create_subscription(
            Image,
            '/orca4_ign/front_left/image_raw',
            self.camera_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/orca4_ign/odom',
            self.odom_callback,
            10
        )
        
        # --- PUBLISHERS (6 thrusters) ---
        self.thruster_pubs = []
        for i in range(1, 7):
            pub = self.create_publisher(
                Float64,
                f'/orca4_ign/thruster{i}/cmd',
                10
            )
            self.thruster_pubs.append(pub)
        
        # Processed image publisher for visualization
        self.processed_image_pub = self.create_publisher(
            Image,
            '/autonomous/processed_image',
            10
        )
        
        # Control loop timer (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('='*70)
        self.get_logger().info('🤖 AUTONOMOUS NAVIGATION NODE STARTED')
        self.get_logger().info('='*70)
        self.get_logger().info(f'Frame Size: {self.FRAME_WIDTH}x{self.FRAME_HEIGHT}')
        self.get_logger().info(f'Detection Area Threshold: {self.MIN_DETECT_AREA} px')
        self.get_logger().info(f'Control Gains - Sway: {self.K_SWAY}, Heave: {self.K_HEAVE}, Surge: {self.K_SURGE}')
        self.get_logger().info('='*70)
    
    def clamp(self, value, min_val, max_val):
        """Clamp value between min and max"""
        return max(min(max_val, value), min_val)
    
    def odom_callback(self, msg):
        """Track robot position"""
        self.current_position = msg.pose.pose.position
        self.current_depth = self.current_position.z
        
        if self.start_position is None:
            self.start_position = self.current_position
            self.get_logger().info(
                f'📍 Starting Position: X={self.current_position.x:.2f}, '
                f'Y={self.current_position.y:.2f}, Z={self.current_position.z:.2f}'
            )
    
    def calculate_distance_traveled(self):
        """Calculate distance from start"""
        if self.start_position is None or self.current_position is None:
            return 0.0
        
        dx = self.current_position.x - self.start_position.x
        dy = self.current_position.y - self.start_position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def detect_white_gate(self, cv_image):
        """
        Detect white/off-white gate using HSV color filtering
        Returns: (detected, center_x, center_y, area, processed_image)
        """
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(cv_image, (11, 11), 0)
        
        # Convert to HSV color space
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # Create mask for white/off-white regions
        mask = cv2.inRange(hsv, self.LOWER_WHITE, self.UPPER_WHITE)
        
        # Morphological operations to clean up noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)
        
        # Find contours
        contours, _ = cv2.findContours(
            mask, 
            cv2.RETR_EXTERNAL, 
            cv2.CHAIN_APPROX_SIMPLE
        )
        
        # Create visualization
        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv_image = cv2.addWeighted(cv_image, 0.7, mask_colored, 0.3, 0)
        
        # Draw crosshair at image center
        cv2.line(cv_image, (self.CENTER_X, 0), 
                (self.CENTER_X, self.FRAME_HEIGHT), (255, 255, 0), 2)
        cv2.line(cv_image, (0, self.CENTER_Y), 
                (self.FRAME_WIDTH, self.CENTER_Y), (255, 255, 0), 2)
        
        if len(contours) == 0:
            cv2.putText(cv_image, 'SEARCHING FOR GATE...', (10, 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            return False, 0, 0, 0, cv_image
        
        # Find the largest contour (assumed to be the gate)
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        # Filter small detections
        if area < self.MIN_DETECT_AREA:
            cv2.putText(cv_image, 'OBJECT TOO SMALL', (10, 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 2)
            return False, 0, 0, 0, cv_image
        
        # Get moments to find center
        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return False, 0, 0, 0, cv_image
        
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        
        # Get bounding rectangle
        x, y, w, h = cv2.boundingRect(largest_contour)
        
        # Draw detection visualization
        cv2.drawContours(cv_image, [largest_contour], -1, (0, 255, 0), 3)
        cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Center point
        cv2.circle(cv_image, (cx, cy), 10, (0, 0, 255), -1)
        cv2.circle(cv_image, (cx, cy), 12, (255, 255, 255), 2)
        
        # Line from center to gate
        cv2.line(cv_image, (self.CENTER_X, self.CENTER_Y),
                (cx, cy), (0, 255, 255), 2)
        
        # Text annotations
        cv2.putText(cv_image, '🎯 GATE DETECTED', (x, y - 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv2.putText(cv_image, f'Area: {int(area)} px', (x, y - 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(cv_image, f'Center: ({cx}, {cy})', (x, y - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Calculate alignment errors
        error_x = cx - self.CENTER_X
        error_y = cy - self.CENTER_Y
        
        cv2.putText(cv_image, f'Error X: {error_x:+d} px', (10, 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(cv_image, f'Error Y: {error_y:+d} px', (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # Distance indicator
        if area < 20000:
            distance_text = "FAR"
            color = (0, 0, 255)
        elif area < 50000:
            distance_text = "MEDIUM"
            color = (0, 165, 255)
        elif area < self.STOP_DISTANCE_AREA:
            distance_text = "CLOSE"
            color = (0, 255, 255)
        else:
            distance_text = "VERY CLOSE!"
            color = (0, 255, 0)
        
        cv2.putText(cv_image, f'Distance: {distance_text}', (10, 100),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        
        return True, cx, cy, area, cv_image
    
    def camera_callback(self, msg):
        """Process camera image and detect gate"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Detect gate
            detected, cx, cy, area, processed = self.detect_white_gate(cv_image)
            
            # Update state
            self.gate_detected = detected
            if detected:
                self.gate_cx = cx
                self.gate_cy = cy
                self.gate_area = area
            
            # Publish processed image
            try:
                processed_msg = self.bridge.cv2_to_imgmsg(processed, encoding='bgr8')
                processed_msg.header = msg.header
                self.processed_image_pub.publish(processed_msg)
            except CvBridgeError as e:
                self.get_logger().error(f'Failed to publish processed image: {str(e)}')
                
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {str(e)}')
    
    def send_thruster_commands(self, heave, surge, sway):
        """
        Send commands to thrusters
        Thruster configuration (BlueROV2):
        T1-T4: Horizontal (45° angled)
        T5-T6: Vertical
        """
        cos45 = 0.7071
        
        # Horizontal thrusters (T1-T4)
        # Inverted surge for correct direction
        surge = -surge
        
        t1 = cos45 * (surge + sway)  # Front-left
        t2 = cos45 * (surge - sway)  # Front-right
        t3 = cos45 * (-surge + sway) # Back-left
        t4 = cos45 * (-surge - sway) # Back-right
        
        # Vertical thrusters (T5-T6)
        t5 = -heave  # Front vertical
        t6 = -heave  # Back vertical
        
        # Clamp and publish
        thrusts = [t1, t2, t3, t4, t5, t6]
        
        for i, thrust in enumerate(thrusts):
            thrust = self.clamp(thrust, self.THRUST_MIN, self.THRUST_MAX)
            msg = Float64()
            msg.data = float(thrust)
            self.thruster_pubs[i].publish(msg)
    
    def control_loop(self):
        """Main autonomous control loop"""
        
        # Check if mission complete
        distance = self.calculate_distance_traveled()
        if distance > 3.0:  # 3 meters traveled
            if self.mission_active:
                self.mission_active = False
                self.get_logger().info('='*70)
                self.get_logger().info('🎉 MISSION COMPLETE!')
                self.get_logger().info(f'Distance traveled: {distance:.2f} m')
                self.get_logger().info('='*70)
            
            # Stop thrusters
            self.send_thruster_commands(0.0, 0.0, 0.0)
            return
        
        # Default: Neutral
        thrust_heave = self.HEAVE_NEUTRAL
        thrust_surge = self.SURGE_NEUTRAL
        thrust_sway = self.SWAY_NEUTRAL
        
        if not self.gate_detected:
            # ============================================================
            # SEARCHING MODE - Slow forward and gentle rotation
            # ============================================================
            thrust_surge = 3.0   # Slow forward
            thrust_sway = 1.0    # Gentle rotation to search
            
            self.get_logger().warn(
                '🔍 SEARCHING FOR GATE...',
                throttle_duration_sec=2.0
            )
        
        else:
            # ============================================================
            # TRACKING MODE - Align and approach gate
            # ============================================================
            
            # Horizontal alignment (Sway)
            error_x = self.gate_cx - self.CENTER_X
            thrust_sway = error_x * self.K_SWAY
            
            # Vertical alignment (Heave)
            # If cy > Center (gate is low), we need to dive
            error_y = self.gate_cy - self.CENTER_Y
            thrust_heave = -(error_y * self.K_HEAVE)
            
            # Forward speed (Surge) - based on gate size/distance
            if self.gate_area < self.STOP_DISTANCE_AREA:
                size_error = self.STOP_DISTANCE_AREA - self.gate_area
                thrust_surge = size_error * self.K_SURGE
                thrust_surge = min(10.0, thrust_surge)  # Cap max speed
            else:
                # Very close - maintain moderate speed through gate
                thrust_surge = 8.0
                thrust_sway = 0.0  # No more corrections
                thrust_heave = 0.0
            
            # Log status
            self.get_logger().info(
                f'🎯 TRACKING | Area: {int(self.gate_area):6d} | '
                f'ErrX: {error_x:+5.0f} | ErrY: {error_y:+5.0f} | '
                f'Sway: {thrust_sway:+5.1f} | Heave: {thrust_heave:+5.1f} | Surge: {thrust_surge:5.1f}',
                throttle_duration_sec=0.5
            )
        
        # Clamp all thrust values
        thrust_heave = self.clamp(thrust_heave, self.THRUST_MIN, self.THRUST_MAX)
        thrust_surge = self.clamp(thrust_surge, self.THRUST_MIN, self.THRUST_MAX)
        thrust_sway = self.clamp(thrust_sway, self.THRUST_MIN, self.THRUST_MAX)
        
        # Send commands to thrusters
        self.send_thruster_commands(thrust_heave, thrust_surge, thrust_sway)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down gracefully...')
    finally:
        # Stop all thrusters
        node.send_thruster_commands(0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
White Gate Detector - HSV-based detection for underwater environment
Detects bright white/off-white gate frames
Integrated with auv_slam package
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from collections import deque


class WhiteGateDetector(Node):
    def __init__(self):
        super().__init__('gate_detector_node')
        self.bridge = CvBridge()
        self.camera_matrix = None
        
        # Gate specifications
        self.gate_width_meters = 1.5
        self.gate_height_meters = 1.5
        
        # HSV ranges for WHITE/OFF-WHITE detection
        # White in underwater lighting has slight blue/green tint
        # H: 0-180 (all hues), S: 0-40 (low saturation), V: 180-255 (high brightness)
        self.white_lower = np.array([0, 0, 180])
        self.white_upper = np.array([180, 40, 255])
        
        # Detection parameters
        self.min_contour_area = 500  # Lowered for distant detection
        
        # Detection history for stability
        self.detection_history = deque(maxlen=3)
        self.min_confirmations = 2
        self.frame_count = 0
        
        # QoS profiles
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscriptions
        self.image_sub = self.create_subscription(
            Image,
            '/camera_forward/image_raw',
            self.image_callback,
            qos_sensor
        )
        
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            '/camera_forward/camera_info',
            self.cam_info_callback,
            qos_reliable
        )
        
        # Publishers (matching navigator expectations)
        self.gate_detected_pub = self.create_publisher(Bool, '/gate/detected', 10)
        self.alignment_pub = self.create_publisher(Float32, '/gate/alignment_error', 10)
        self.distance_pub = self.create_publisher(Float32, '/gate/estimated_distance', 10)
        self.gate_center_pub = self.create_publisher(Point, '/gate/center_point', 10)
        self.debug_pub = self.create_publisher(Image, '/gate/debug_image', 10)
        self.status_pub = self.create_publisher(String, '/gate/status', 10)
        
        self.get_logger().info('='*70)
        self.get_logger().info('✅ White Gate Detector initialized')
        self.get_logger().info('   HSV: H[0-180], S[0-40], V[180-255]')
        self.get_logger().info('   Min Area: 500 pixels')
        self.get_logger().info('='*70)
    
    def cam_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.image_width = msg.width
            self.image_height = msg.height
            self.fx = self.camera_matrix[0, 0]
            self.fy = self.camera_matrix[1, 1]
            self.cx = self.camera_matrix[0, 2]
            self.cy = self.camera_matrix[1, 2]
            self.get_logger().info(f'📷 Camera: {self.image_width}x{self.image_height}, fx={self.fx:.1f}')
    
    def image_callback(self, msg: Image):
        if self.camera_matrix is None:
            return
        
        self.frame_count += 1
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return
        
        # Convert to HSV for white detection
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        debug_img = cv_image.copy()
        h, w = cv_image.shape[:2]
        
        # Create white mask
        white_mask = cv2.inRange(hsv_image, self.white_lower, self.white_upper)
        
        # Morphological operations to clean noise
        kernel = np.ones((5, 5), np.uint8)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Initialize detection variables
        gate_detected = False
        alignment_error = 0.0
        estimated_distance = 999.0
        gate_center_x = w // 2
        gate_center_y = h // 2
        
        # Find best rectangular contour (gate frame)
        best_gate = None
        best_score = 0
        
        if contours:
            for cnt in contours:
                area = cv2.contourArea(cnt)
                
                if area < self.min_contour_area:
                    continue
                
                # Get bounding box
                x, y, w_box, h_box = cv2.boundingRect(cnt)
                
                if w_box == 0 or h_box == 0:
                    continue
                
                # Calculate aspect ratio (should be ~1.0 for square gate)
                aspect_ratio = float(h_box) / w_box
                
                # Gate should be roughly square (0.7 to 1.5 range)
                if 0.7 < aspect_ratio < 1.5:
                    # Score based on area and aspect ratio
                    aspect_score = 1.0 - abs(1.0 - aspect_ratio)
                    score = area * aspect_score
                    
                    if score > best_score:
                        M = cv2.moments(cnt)
                        if M["m00"] > 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            
                            best_gate = {
                                'center': (cx, cy),
                                'bbox': (x, y, w_box, h_box),
                                'area': area,
                                'aspect': aspect_ratio,
                                'contour': cnt
                            }
                            best_score = score
        
        # Process best gate candidate
        if best_gate is not None:
            gate_detected = True
            
            gate_center_x, gate_center_y = best_gate['center']
            x, y, w_box, h_box = best_gate['bbox']
            
            # Calculate alignment error
            pixel_error = gate_center_x - (w / 2)
            alignment_error = pixel_error / (w / 2)
            
            # Estimate distance using gate width
            if w_box > 20 and h_box > 20:
                distance_from_width = (self.gate_width_meters * self.fx) / w_box
                distance_from_height = (self.gate_height_meters * self.fy) / h_box
                estimated_distance = (distance_from_width + distance_from_height) / 2
                estimated_distance = max(0.3, min(estimated_distance, 20.0))
            
            # Visualization
            cv2.rectangle(debug_img, (x, y), (x+w_box, y+h_box), (0, 255, 0), 3)
            cv2.circle(debug_img, (gate_center_x, gate_center_y), 15, (0, 255, 0), -1)
            cv2.circle(debug_img, (gate_center_x, gate_center_y), 17, (255, 255, 255), 2)
            cv2.line(debug_img, (gate_center_x, gate_center_y), 
                    (w//2, gate_center_y), (255, 0, 0), 2)
            cv2.line(debug_img, (gate_center_x, 0), (gate_center_x, h), (0, 255, 0), 3)
            
            cv2.putText(debug_img, f"WHITE GATE {estimated_distance:.2f}m", 
                       (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(debug_img, f"Align: {alignment_error:+.3f}", 
                       (x, y+h_box+25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(debug_img, f"Area: {best_gate['area']:.0f}", 
                       (x, y+h_box+50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Draw center reference line
        cv2.line(debug_img, (w//2, 0), (w//2, h), (255, 255, 0), 2)
        
        # Status overlay
        status_color = (0, 255, 0) if gate_detected else (100, 100, 100)
        status_text = f"Frame {self.frame_count} | "
        status_text += f"{'GATE DETECTED' if gate_detected else 'SEARCHING'} | "
        status_text += f"{estimated_distance:.2f}m"
        
        cv2.putText(debug_img, status_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
        cv2.putText(debug_img, f"Contours: {len(contours)}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Temporal filtering
        self.detection_history.append(gate_detected)
        confirmed = sum(self.detection_history) >= self.min_confirmations
        
        # Publish detection data
        self.gate_detected_pub.publish(Bool(data=confirmed))
        
        if confirmed:
            self.alignment_pub.publish(Float32(data=float(alignment_error)))
            self.distance_pub.publish(Float32(data=float(estimated_distance)))
            
            center_msg = Point()
            center_msg.x = float(gate_center_x)
            center_msg.y = float(gate_center_y)
            center_msg.z = float(estimated_distance)
            self.gate_center_pub.publish(center_msg)
            
            status = f"WHITE GATE | Dist:{estimated_distance:.2f}m | Align:{alignment_error:+.3f}"
            self.status_pub.publish(String(data=status))
            
            # Log detection periodically
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f'🎯 Gate: {estimated_distance:.2f}m, Align: {alignment_error:+.3f}, '
                    f'Area: {best_gate["area"]:.0f}',
                    throttle_duration_sec=0.9
                )
        
        # Publish debug image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'Debug image error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = WhiteGateDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
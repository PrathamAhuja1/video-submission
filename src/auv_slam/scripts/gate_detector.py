#!/usr/bin/env python3
"""
Combined Gate and Flare Detector - OPTIMIZED FOR ORANGE FLUORESCENT GATES
Detects orange/fluorescent gates AND red flares simultaneously
UPDATED: Single HSV range for both gate and flare detection
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


class CombinedDetector(Node):
    def __init__(self):
        super().__init__('gate_detector_node')
        self.bridge = CvBridge()
        self.camera_matrix = None
        
        # Gate specifications
        self.gate_width_meters = 1.5
        self.gate_height_meters = 1.5
        
        # Flare specifications
        self.flare_width_meters = 0.3
        
        # ====================================================================
        # GATE DETECTION - NEW SINGLE HSV RANGE
        # ====================================================================
        self.gate_lower = np.array([20, 98, 57])
        self.gate_upper = np.array([47, 255, 195])
        
        # ====================================================================
        # FLARE DETECTION - NEW SINGLE HSV RANGE
        # ====================================================================
        self.flare_lower = np.array([20, 55, 53])
        self.flare_upper = np.array([138, 255, 200])
        
        # ====================================================================
        # DETECTION PARAMETERS - TUNED FOR PVC PIPE GATES UNDERWATER
        # ====================================================================
        self.min_gate_area = 400           # Minimum contour area (lowered for distance)
        self.max_gate_area = 150000        # Maximum contour area (larger for close range)
        self.min_gate_circularity = 0.3    # PVC frames are rectangular, not circular
        self.min_gate_aspect_ratio = 0.5   # Wider range for rectangular frames
        self.max_gate_aspect_ratio = 2.0   # Can be vertical or horizontal
        self.min_flare_area = 500
        
        # Additional pipe-specific parameters
        self.min_gate_solidity = 0.4       # Frame structure, not solid
        self.max_gate_solidity = 0.95      # But not too hollow
        
        # Detection history for temporal filtering
        self.gate_history = deque(maxlen=5)      # Increased from 3 for more stability
        self.flare_history = deque(maxlen=3)
        self.min_confirmations = 3               # Need 3/5 frames to confirm
        self.frame_count = 0
        
        # Adaptive thresholding
        self.use_adaptive = True
        self.brightness_threshold = 180
        
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
            '/image_raw',
            self.image_callback,
            qos_sensor
        )
        
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.cam_info_callback,
            qos_reliable
        )
        
        # Publishers - Gate
        self.gate_detected_pub = self.create_publisher(Bool, '/gate/detected', 10)
        self.gate_alignment_pub = self.create_publisher(Float32, '/gate/alignment_error', 10)
        self.gate_distance_pub = self.create_publisher(Float32, '/gate/estimated_distance', 10)
        self.gate_center_pub = self.create_publisher(Point, '/gate/center_point', 10)
        
        # Publishers - Flare
        self.flare_detected_pub = self.create_publisher(Bool, '/flare/detected', 10)
        self.flare_direction_pub = self.create_publisher(Float32, '/flare/direction_angle', 10)
        self.flare_distance_pub = self.create_publisher(Float32, '/flare/estimated_distance', 10)
        self.flare_center_pub = self.create_publisher(Point, '/flare/center_point', 10)
        self.flare_danger_pub = self.create_publisher(Bool, '/flare/danger_zone', 10)
        
        # Publishers - Common
        self.debug_pub = self.create_publisher(Image, '/gate/debug_image', 10)
        self.mask_pub = self.create_publisher(Image, '/gate/mask_image', 10)
        self.status_pub = self.create_publisher(String, '/gate/status', 10)
        
        self.get_logger().info('='*70)
        self.get_logger().info('🎯 UNDERWATER Gate & Flare Detector - UPDATED HSV')
        self.get_logger().info('='*70)
        self.get_logger().info('GATE Detection:')
        self.get_logger().info(f'  HSV Range: Lower={self.gate_lower.tolist()}, Upper={self.gate_upper.tolist()}')
        self.get_logger().info('FLARE Detection:')
        self.get_logger().info(f'  HSV Range: Lower={self.flare_lower.tolist()}, Upper={self.flare_upper.tolist()}')
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
    
    def analyze_gate_contour(self, contour, image_shape):
        """
        Advanced contour analysis for PVC pipe gate detection
        Returns: (is_valid, score, properties)
        """
        area = cv2.contourArea(contour)
        
        # Area check
        if area < self.min_gate_area or area > self.max_gate_area:
            return False, 0.0, None
        
        # Bounding rectangle
        x, y, w, h = cv2.boundingRect(contour)
        if w == 0 or h == 0:
            return False, 0.0, None
        
        # Aspect ratio check (gates can be square or rectangular)
        aspect_ratio = float(h) / w
        if aspect_ratio < self.min_gate_aspect_ratio or aspect_ratio > self.max_gate_aspect_ratio:
            return False, 0.0, None
        
        # Circularity/compactness check (relaxed for pipe frames)
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            return False, 0.0, None
        
        circularity = 4 * np.pi * area / (perimeter * perimeter)
        if circularity < self.min_gate_circularity:
            return False, 0.0, None
        
        # Solidity check (pipe frames have moderate solidity)
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        if hull_area > 0:
            solidity = float(area) / hull_area
        else:
            solidity = 0
        
        # PVC pipe frames should have moderate solidity (not too hollow, not too solid)
        if solidity < self.min_gate_solidity or solidity > self.max_gate_solidity:
            return False, 0.0, None
        
        # Rectangularity check (how well does it fit a rectangle?)
        rect_area = w * h
        extent = float(area) / rect_area if rect_area > 0 else 0
        
        # Pipe frames should fill most of their bounding box
        if extent < 0.4:  # Too sparse
            return False, 0.0, None
        
        # Moments for centroid
        M = cv2.moments(contour)
        if M["m00"] == 0:
            return False, 0.0, None
        
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        
        # Position check (gates usually not at extreme edges)
        h_img, w_img = image_shape[:2]
        edge_margin = 0.03  # 3% margin (reduced from 5% for underwater)
        if (cx < w_img * edge_margin or cx > w_img * (1 - edge_margin) or
            cy < h_img * edge_margin or cy > h_img * (1 - edge_margin)):
            edge_penalty = 0.7  # Stronger penalty for edge positions
        else:
            edge_penalty = 1.0
        
        # Calculate composite score optimized for PVC pipe gates
        # Prefer: large area, moderate aspect ratio, good rectangularity, moderate solidity
        aspect_score = 1.0 - abs(1.0 - aspect_ratio)  # Best at 1.0 (square)
        aspect_score = max(0.0, min(1.0, aspect_score)) * 0.7 + 0.3  # Don't penalize too much
        
        size_score = min(area / 8000.0, 1.0)  # Normalize to 0-1
        
        # Rectangularity is important for pipe frames
        rect_score = extent
        
        score = (area * 0.35 +                 # Area weight (increased)
                 circularity * 4000 * 0.20 +   # Circularity weight
                 solidity * 4000 * 0.15 +      # Solidity weight
                 aspect_score * 4000 * 0.15 +  # Aspect ratio weight
                 size_score * 4000 * 0.10 +    # Size score
                 rect_score * 4000 * 0.05) * edge_penalty
        
        properties = {
            'center': (cx, cy),
            'bbox': (x, y, w, h),
            'area': area,
            'contour': contour,
            'circularity': circularity,
            'aspect_ratio': aspect_ratio,
            'solidity': solidity,
            'extent': extent
        }
        
        return True, score, properties
    
    def image_callback(self, msg: Image):
        if self.camera_matrix is None:
            return
        
        self.frame_count += 1
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return
        
        debug_img = cv_image.copy()
        h, w = cv_image.shape[:2]
        
        # ========================================
        # GATE DETECTION - SIMPLIFIED SINGLE RANGE
        # ========================================
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Apply single gate HSV range
        gate_mask = cv2.inRange(hsv_image, self.gate_lower, self.gate_upper)
        
        # Publish combined mask for debugging
        try:
            mask_msg = self.bridge.cv2_to_imgmsg(gate_mask, "mono8")
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'Mask publish error: {e}')
        
        # Morphological operations
        kernel_close = np.ones((9, 9), np.uint8)
        gate_mask = cv2.morphologyEx(gate_mask, cv2.MORPH_CLOSE, kernel_close, iterations=2)
        
        kernel_open = np.ones((5, 5), np.uint8)
        gate_mask = cv2.morphologyEx(gate_mask, cv2.MORPH_OPEN, kernel_open, iterations=1)
        
        kernel_dilate = np.ones((7, 7), np.uint8)
        gate_mask = cv2.dilate(gate_mask, kernel_dilate, iterations=1)
        
        gate_mask = cv2.GaussianBlur(gate_mask, (7, 7), 0)
        _, gate_mask = cv2.threshold(gate_mask, 100, 255, cv2.THRESH_BINARY)
        
        # Find contours
        gate_contours, _ = cv2.findContours(gate_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        gate_detected = False
        gate_alignment_error = 0.0
        gate_distance = 999.0
        gate_center_x = w // 2
        gate_center_y = h // 2
        
        best_gate = None
        best_gate_score = 0
        
        # Analyze all contours
        valid_gates = []
        for cnt in gate_contours:
            is_valid, score, properties = self.analyze_gate_contour(cnt, cv_image.shape)
            
            if is_valid:
                valid_gates.append((score, properties))
                
                # Draw all valid candidates in yellow
                cv2.drawContours(debug_img, [properties['contour']], -1, (0, 255, 255), 2)
        
        # Select best gate
        if valid_gates:
            valid_gates.sort(key=lambda x: x[0], reverse=True)
            best_gate_score, best_gate = valid_gates[0]
            gate_detected = True
            
            gate_center_x, gate_center_y = best_gate['center']
            x, y, w_box, h_box = best_gate['bbox']
            
            # Calculate alignment error
            pixel_error = gate_center_x - (w / 2)
            gate_alignment_error = pixel_error / (w / 2)
            
            # Estimate distance
            if w_box > 20 and h_box > 20:
                distance_from_width = (self.gate_width_meters * self.fx) / w_box
                distance_from_height = (self.gate_height_meters * self.fy) / h_box
                gate_distance = (distance_from_width + distance_from_height) / 2
                gate_distance = max(0.3, min(gate_distance, 20.0))
            
            # Draw best gate in GREEN with details
            cv2.rectangle(debug_img, (x, y), (x+w_box, y+h_box), (0, 255, 0), 4)
            cv2.circle(debug_img, (gate_center_x, gate_center_y), 15, (0, 255, 0), -1)
            cv2.circle(debug_img, (gate_center_x, gate_center_y), 17, (255, 255, 255), 3)
            cv2.line(debug_img, (gate_center_x, 0), (gate_center_x, h), (0, 255, 0), 2)
            
            # Detailed text info
            info_y = y - 60 if y > 120 else y + h_box + 30
            cv2.putText(debug_img, f"GATE {gate_distance:.2f}m", 
                       (x, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 3)
            cv2.putText(debug_img, f"Score: {best_gate_score:.0f}", 
                       (x, info_y + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(debug_img, f"AR:{best_gate['aspect_ratio']:.2f} Ext:{best_gate['extent']:.2f}", 
                       (x, info_y + 52), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(debug_img, f"Sol:{best_gate['solidity']:.2f} Circ:{best_gate['circularity']:.2f}", 
                       (x, info_y + 72), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # ========================================
        # FLARE DETECTION - SIMPLIFIED SINGLE RANGE
        # ========================================
        flare_mask = cv2.inRange(hsv_image, self.flare_lower, self.flare_upper)
        
        kernel = np.ones((5, 5), np.uint8)
        flare_mask = cv2.morphologyEx(flare_mask, cv2.MORPH_CLOSE, kernel)
        flare_mask = cv2.morphologyEx(flare_mask, cv2.MORPH_OPEN, kernel)
        flare_mask = cv2.GaussianBlur(flare_mask, (5, 5), 0)
        
        flare_contours, _ = cv2.findContours(flare_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        flare_detected = False
        flare_direction = 0.0
        flare_distance = 999.0
        flare_center_x = w // 2
        flare_center_y = h // 2
        flare_in_danger = False
        
        best_flare = None
        best_flare_area = 0
        
        for cnt in flare_contours:
            area = cv2.contourArea(cnt)
            if area < self.min_flare_area:
                continue
            
            if area > best_flare_area:
                x, y, w_box, h_box = cv2.boundingRect(cnt)
                if w_box == 0 or h_box == 0:
                    continue
                
                M = cv2.moments(cnt)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    best_flare = {
                        'center': (cx, cy),
                        'bbox': (x, y, w_box, h_box),
                        'area': area,
                        'contour': cnt
                    }
                    best_flare_area = area
        
        if best_flare is not None:
            flare_detected = True
            flare_center_x, flare_center_y = best_flare['center']
            x, y, w_box, h_box = best_flare['bbox']
            
            pixel_offset = flare_center_x - (w / 2)
            flare_direction = pixel_offset / (w / 2)
            
            if w_box > 20:
                flare_distance = (self.flare_width_meters * self.fx) / w_box
                flare_distance = max(0.2, min(flare_distance, 20.0))
            
            flare_in_danger = flare_distance < 1.5
            
            flare_color = (0, 0, 255) if flare_in_danger else ((0, 165, 255) if flare_distance < 3.0 else (0, 255, 255))
            status_text = "⚠️ DANGER" if flare_in_danger else ("⚠️ WARNING" if flare_distance < 3.0 else "FLARE")
            
            cv2.rectangle(debug_img, (x, y), (x+w_box, y+h_box), flare_color, 3)
            cv2.circle(debug_img, (flare_center_x, flare_center_y), 12, flare_color, -1)
            cv2.putText(debug_img, f"{status_text} {flare_distance:.2f}m", 
                       (x, y+h_box+25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, flare_color, 2)
            
            if flare_in_danger:
                cv2.rectangle(debug_img, (0, 0), (w, h), (0, 0, 255), 10)
                cv2.putText(debug_img, "!!! DANGER !!!", 
                           (w//2-100, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
        
        # ========================================
        # VISUALIZATION
        # ========================================
        cv2.line(debug_img, (w//2, 0), (w//2, h), (255, 255, 0), 2)
        
        status_line = f"Frame {self.frame_count} | Gate: {'YES' if gate_detected else 'NO'} {gate_distance:.1f}m"
        if gate_detected:
            status_line += f" (Score:{best_gate_score:.0f})"
        status_line += f" | Flare: {'YES' if flare_detected else 'NO'} {flare_distance:.1f}m"
        
        status_color = (0, 0, 255) if flare_in_danger else ((0, 255, 0) if gate_detected else (100, 100, 100))
        
        cv2.putText(debug_img, status_line, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
        cv2.putText(debug_img, f"Valid gates found: {len(valid_gates)}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # ========================================
        # TEMPORAL FILTERING & PUBLISHING
        # ========================================
        self.gate_history.append(gate_detected)
        self.flare_history.append(flare_detected)
        
        gate_confirmed = sum(self.gate_history) >= self.min_confirmations
        flare_confirmed = sum(self.flare_history) >= self.min_confirmations
        
        # Publish gate data
        self.gate_detected_pub.publish(Bool(data=gate_confirmed))
        if gate_confirmed:
            self.gate_alignment_pub.publish(Float32(data=float(gate_alignment_error)))
            self.gate_distance_pub.publish(Float32(data=float(gate_distance)))
            
            center_msg = Point()
            center_msg.x = float(gate_center_x)
            center_msg.y = float(gate_center_y)
            center_msg.z = float(gate_distance)
            self.gate_center_pub.publish(center_msg)
        
        # Publish flare data
        self.flare_detected_pub.publish(Bool(data=flare_confirmed))
        self.flare_danger_pub.publish(Bool(data=flare_in_danger))
        if flare_confirmed:
            self.flare_direction_pub.publish(Float32(data=float(flare_direction)))
            self.flare_distance_pub.publish(Float32(data=float(flare_distance)))
            
            center_msg = Point()
            center_msg.x = float(flare_center_x)
            center_msg.y = float(flare_center_y)
            center_msg.z = float(flare_distance)
            self.flare_center_pub.publish(center_msg)
        
        status = f"Gate:{gate_distance:.1f}m | Flare:{flare_distance:.1f}m"
        if flare_in_danger:
            status = "⚠️ DANGER! " + status
        self.status_pub.publish(String(data=status))
        
        # Periodic logging
        if self.frame_count % 30 == 0:
            if gate_detected:
                self.get_logger().info(
                    f'🎯 GATE: {gate_distance:.1f}m | Score:{best_gate_score:.0f} | '
                    f'AR:{best_gate["aspect_ratio"]:.2f} Sol:{best_gate["solidity"]:.2f} '
                    f'Ext:{best_gate["extent"]:.2f} | Flare:{flare_distance:.1f}m',
                    throttle_duration_sec=0.9
                )
            else:
                self.get_logger().info(
                    f'❌ No gate detected | Valid candidates: {len(valid_gates)} | '
                    f'Flare:{flare_distance:.1f}m',
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
    node = CombinedDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
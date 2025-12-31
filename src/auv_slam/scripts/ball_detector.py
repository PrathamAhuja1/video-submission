#!/usr/bin/env python3
"""
Orange Ball Detector - HSV-based detection for underwater environment
Detects orange/darker orange ball (cricket ball texture)
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


class BallDetector(Node):
    def __init__(self):
        super().__init__('ball_detector_node')
        self.bridge = CvBridge()
        self.camera_matrix = None
        
        # Ball specifications (cricket ball size: ~7.2cm diameter)
        self.ball_diameter_meters = 0.06318
        
        # HSV ranges for ORANGE ball in underwater conditions
        # Underwater: orange shifts toward darker red-orange
        # Lower range: Dark orange to red-orange
        self.orange_lower1 = np.array([0, 100, 50])    # Dark red-orange
        self.orange_upper1 = np.array([15, 255, 200])
        # Upper range: Orange to yellow-orange
        self.orange_lower2 = np.array([10, 120, 60])
        self.orange_upper2 = np.array([25, 255, 220])
        
        # Detection parameters
        self.min_contour_area = 200
        self.min_circularity = 0.65  # Ball should be circular
        
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
        
        # Publishers
        self.ball_detected_pub = self.create_publisher(Bool, '/ball/detected', 10)
        self.alignment_pub = self.create_publisher(Float32, '/ball/alignment_error', 10)
        self.distance_pub = self.create_publisher(Float32, '/ball/estimated_distance', 10)
        self.ball_center_pub = self.create_publisher(Point, '/ball/center_point', 10)
        self.debug_pub = self.create_publisher(Image, '/ball/debug_image', 10)
        self.status_pub = self.create_publisher(String, '/ball/status', 10)
        
        self.get_logger().info('='*70)
        self.get_logger().info('🏀 Orange Ball Detector initialized')
        self.get_logger().info('   HSV: [0-15,100-255,50-200] + [10-25,120-255,60-220]')
        self.get_logger().info('   Min Area: 200px | Circularity: 0.65+')
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
        
        # Convert to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        debug_img = cv_image.copy()
        h, w = cv_image.shape[:2]
        
        # Create orange masks (two ranges to cover full orange spectrum)
        mask1 = cv2.inRange(hsv_image, self.orange_lower1, self.orange_upper1)
        mask2 = cv2.inRange(hsv_image, self.orange_lower2, self.orange_upper2)
        orange_mask = cv2.bitwise_or(mask1, mask2)
        
        # Morphological operations
        kernel = np.ones((5, 5), np.uint8)
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel)
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, kernel)
        
        # Gaussian blur to reduce noise
        orange_mask = cv2.GaussianBlur(orange_mask, (5, 5), 0)
        
        # Find contours
        contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Initialize detection variables
        ball_detected = False
        alignment_error = 0.0
        estimated_distance = 999.0
        ball_center_x = w // 2
        ball_center_y = h // 2
        
        # Find best circular contour (ball)
        best_ball = None
        best_score = 0
        
        if contours:
            for cnt in contours:
                area = cv2.contourArea(cnt)
                
                if area < self.min_contour_area:
                    continue
                
                # Calculate circularity: 4*pi*area / perimeter^2
                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0:
                    continue
                
                circularity = 4 * np.pi * area / (perimeter * perimeter)
                
                # Ball should be circular
                if circularity < self.min_circularity:
                    continue
                
                # Get enclosing circle
                (cx, cy), radius = cv2.minEnclosingCircle(cnt)
                
                if radius < 5:
                    continue
                
                # Score based on area, circularity, and size
                score = area * circularity * radius
                
                if score > best_score:
                    best_ball = {
                        'center': (int(cx), int(cy)),
                        'radius': radius,
                        'area': area,
                        'circularity': circularity,
                        'contour': cnt
                    }
                    best_score = score
        
        # Process best ball candidate
        if best_ball is not None:
            ball_detected = True
            
            ball_center_x, ball_center_y = best_ball['center']
            radius = best_ball['radius']
            
            # Calculate alignment error
            pixel_error = ball_center_x - (w / 2)
            alignment_error = pixel_error / (w / 2)
            
            # Estimate distance using ball diameter
            if radius > 5:
                # Distance = (real_diameter * focal_length) / (2 * pixel_radius)
                estimated_distance = (self.ball_diameter_meters * self.fx) / (2 * radius)
                estimated_distance = max(0.2, min(estimated_distance, 10.0))
            
            # Visualization
            cv2.circle(debug_img, (ball_center_x, ball_center_y), 
                      int(radius), (0, 255, 0), 3)
            cv2.circle(debug_img, (ball_center_x, ball_center_y), 
                      8, (0, 255, 0), -1)
            cv2.circle(debug_img, (ball_center_x, ball_center_y), 
                      10, (255, 255, 255), 2)
            
            # Draw alignment line
            cv2.line(debug_img, (ball_center_x, ball_center_y), 
                    (w//2, ball_center_y), (255, 0, 255), 2)
            cv2.line(debug_img, (ball_center_x, 0), 
                    (ball_center_x, h), (0, 255, 0), 2)
            
            # Text annotations
            cv2.putText(debug_img, f"BALL {estimated_distance:.2f}m", 
                       (ball_center_x - 60, ball_center_y - int(radius) - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(debug_img, f"Align: {alignment_error:+.3f}", 
                       (ball_center_x - 60, ball_center_y + int(radius) + 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(debug_img, f"Circ: {best_ball['circularity']:.2f}", 
                       (ball_center_x - 60, ball_center_y + int(radius) + 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Draw center reference line
        cv2.line(debug_img, (w//2, 0), (w//2, h), (255, 255, 0), 2)
        
        # Status overlay
        status_color = (0, 255, 0) if ball_detected else (100, 100, 100)
        status_text = f"Frame {self.frame_count} | "
        status_text += f"{'BALL DETECTED' if ball_detected else 'SEARCHING'} | "
        status_text += f"{estimated_distance:.2f}m"
        
        cv2.putText(debug_img, status_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
        cv2.putText(debug_img, f"Contours: {len(contours)}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Temporal filtering
        self.detection_history.append(ball_detected)
        confirmed = sum(self.detection_history) >= self.min_confirmations
        
        # Publish detection data
        self.ball_detected_pub.publish(Bool(data=confirmed))
        
        if confirmed:
            self.alignment_pub.publish(Float32(data=float(alignment_error)))
            self.distance_pub.publish(Float32(data=float(estimated_distance)))
            
            center_msg = Point()
            center_msg.x = float(ball_center_x)
            center_msg.y = float(ball_center_y)
            center_msg.z = float(estimated_distance)
            self.ball_center_pub.publish(center_msg)
            
            status = f"BALL | Dist:{estimated_distance:.2f}m | Align:{alignment_error:+.3f}"
            self.status_pub.publish(String(data=status))
            
            # Log detection periodically
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f'🏀 Ball: {estimated_distance:.2f}m, Align: {alignment_error:+.3f}, '
                    f'Circ: {best_ball["circularity"]:.2f}',
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
    node = BallDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
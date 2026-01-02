#!/usr/bin/env python3
"""
Combined Gate and Flare Detector
Detects white gates AND red flares simultaneously
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
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory


class CombinedDetector(Node):
    def __init__(self):
        super().__init__('gate_detector_node')
        self.bridge = CvBridge()
        self.camera_matrix = None
        
        # Gate specifications
        self.gate_width_meters = 1.12
        self.gate_height_meters = 0.62
        
        # Flare specifications
        self.flare_width_meters = 0.15
        
        # HSV ranges for WHITE gate detection
    #    self.white_lower = np.array([0, 0, 180])
    #    self.white_upper = np.array([180, 40, 255])
        

    #    self.red_lower1 = np.array([0, 100, 50])
    #    self.red_upper1 = np.array([10, 255, 255])
    #    self.red_lower2 = np.array([160, 100, 50])
    #    self.red_upper2 = np.array([180, 255, 255])


        self.gate_model = YOLO('/home/pratham/Documents/video-submission/src/auv_slam/models/gate_best.pt') 
        self.flare_model = YOLO('/home/pratham/Documents/video-submission/src/auv_slam/models/flare_best.pt')
        self.gate_conf_thresh = 0.8
        self.flare_conf_thresh = 0.8
        
        # Detection parameters
        self.min_gate_area = 500
        self.min_flare_area = 500
        
        # Detection history for stability
        self.gate_history = deque(maxlen=3)
        self.flare_history = deque(maxlen=3)
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
        self.status_pub = self.create_publisher(String, '/gate/status', 10)
        
        self.get_logger().info('='*70)
        self.get_logger().info('🎯 Combined Gate + Flare Detector initialized')
        self.get_logger().info('   WHITE Gate: H[0-180], S[0-40], V[180-255]')
        self.get_logger().info('   RED Flare: H[0-10,160-180], S[100-255], V[50-255]')
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
            
        debug_img = cv_image.copy()
        h, w = cv_image.shape[:2]

        # ========================================
        # 1. GATE DETECTION (YOLO)
        # ========================================
        gate_detected = False
        gate_alignment_error = 0.0
        gate_distance = 999.0
        gate_center_x = w // 2
        gate_center_y = h // 2
        
        # Run inference
        gate_results = self.gate_model.predict(cv_image, conf=self.gate_conf_thresh, verbose=False)
        
        if len(gate_results[0].boxes) > 0:
            # Get the box with highest confidence
            best_box = max(gate_results[0].boxes, key=lambda x: x.conf)
            
            # Extract box coordinates (xywh format is easier for center/width)
            x_c, y_c, box_w, box_h = best_box.xywh[0].cpu().numpy()
            x1, y1, x2, y2 = best_box.xyxy[0].cpu().numpy() # For drawing
            
            gate_detected = True
            gate_center_x = int(x_c)
            gate_center_y = int(y_c)
            
            # Calculate alignment error (-1.0 to 1.0)
            pixel_error = gate_center_x - (w / 2)
            gate_alignment_error = pixel_error / (w / 2)
            
            # Calculate Distance (using known physical width)
            # Distance = (Real Width * Focal Length) / Pixel Width
            if box_w > 0:
                gate_distance = (self.gate_width_meters * self.fx) / box_w
                # Clamp distance to avoid noise spikes
                gate_distance = max(0.3, min(gate_distance, 20.0))
            
            # Visualization
            cv2.rectangle(debug_img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 3)
            cv2.putText(debug_img, f"GATE {gate_distance:.2f}m", 
                       (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # ========================================
        # 2. FLARE DETECTION (YOLO)
        # ========================================
        flare_detected = False
        flare_direction = 0.0
        flare_distance = 999.0
        flare_center_x = w // 2
        flare_center_y = h // 2
        flare_in_danger = False
        
        # Run inference
        flare_results = self.flare_model.predict(cv_image, conf=self.flare_conf_thresh, verbose=False)
        
        if len(flare_results[0].boxes) > 0:
            # Get the largest/closest flare (biggest width) or highest confidence
            # Usually closest is safest to avoid
            best_flare = max(flare_results[0].boxes, key=lambda x: x.xywh[0][2]) 
            
            x_c, y_c, box_w, box_h = best_flare.xywh[0].cpu().numpy()
            x1, y1, x2, y2 = best_flare.xyxy[0].cpu().numpy()
            
            flare_detected = True
            flare_center_x = int(x_c)
            flare_center_y = int(y_c)
            
            # Calculate direction for avoidance
            pixel_offset = flare_center_x - (w / 2)
            flare_direction = pixel_offset / (w / 2)
            
            # Calculate Distance
            if box_w > 0:
                flare_distance = (self.flare_width_meters * self.fx) / box_w
                flare_distance = max(0.2, min(flare_distance, 20.0))
            
            # Danger check (YOLO boxes are tighter than contours, so 1.5m is safer)
            flare_in_danger = flare_distance < 1.5
            
            # Visualization colors
            color = (0, 0, 255) if flare_in_danger else (0, 165, 255)
            
            cv2.rectangle(debug_img, (int(x1), int(y1)), (int(x2), int(y2)), color, 3)
            cv2.putText(debug_img, f"FLARE {flare_distance:.2f}m", 
                       (int(x1), int(y2)+25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # ========================================
        # COMMON VISUALIZATION
        # ========================================
        cv2.line(debug_img, (w//2, 0), (w//2, h), (255, 255, 0), 2)
        
        # Status overlay
        status_line1 = f"Frame {self.frame_count} | "
        status_line1 += f"Gate: {'YES' if gate_detected else 'NO'} {gate_distance:.1f}m | "
        status_line1 += f"Flare: {'YES' if flare_detected else 'NO'} {flare_distance:.1f}m"
        
        status_color = (0, 255, 0)
        if flare_in_danger:
            status_color = (0, 0, 255)
        elif flare_detected:
            status_color = (0, 165, 255)
        
        cv2.putText(debug_img, status_line1, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
        
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
        self.flare_danger_pub.publish(Bool(data=bool(flare_in_danger)))
        if flare_confirmed:
            self.flare_direction_pub.publish(Float32(data=float(flare_direction)))
            self.flare_distance_pub.publish(Float32(data=float(flare_distance)))
            
            center_msg = Point()
            center_msg.x = float(flare_center_x)
            center_msg.y = float(flare_center_y)
            center_msg.z = float(flare_distance)
            self.flare_center_pub.publish(center_msg)
        
        # Status message
        status = f"Gate:{gate_distance:.1f}m | Flare:{flare_distance:.1f}m"
        if flare_in_danger:
            status = "⚠️ DANGER! " + status
        self.status_pub.publish(String(data=status))
        
        # Periodic logging
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'🎯 Gate:{gate_distance:.1f}m | 🔴 Flare:{flare_distance:.1f}m',
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
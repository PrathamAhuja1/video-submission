#!/usr/bin/env python3
"""
Camera Processor Node
Detects white/off-white gate using computer vision
Publishes detection results for navigation
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Bool
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class CameraProcessorNode(Node):
    def __init__(self):
        super().__init__('camera_processor')
        
        self.bridge = CvBridge()
        
        # Detection parameters
        self.min_gate_area = 1000  # Minimum contour area to be considered gate
        self.image_width = 1280
        self.image_height = 720
        
        # Subscribers for camera feeds
        self.front_left_sub = self.create_subscription(
            Image,
            '/orca4_ign/front_left/image_raw',
            self.front_left_callback,
            10
        )
        
        # Publishers
        self.gate_detection_pub = self.create_publisher(
            Float32MultiArray,
            '/gate_detection',
            10
        )
        
        self.gate_detected_flag_pub = self.create_publisher(
            Bool,
            '/gate_detected',
            10
        )
        
        self.processed_image_pub = self.create_publisher(
            Image,
            '/processed_image',
            10
        )
        
        self.get_logger().info('Camera Processor Node Started')
        self.get_logger().info('Looking for white/off-white gate')
    
    def detect_white_gate(self, cv_image):
        """
        Detect white/off-white gate using HSV color filtering
        Returns: (detected, center_x, center_y, area, processed_image)
        """
        
        # Convert to HSV color space
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Define HSV range for white/off-white
        # White has: Low Saturation, High Value
        # Hue can be anything (full range)
        lower_white = np.array([0, 0, 200])      # H, S, V
        upper_white = np.array([180, 40, 255])   # H, S, V
        
        # Create mask for white regions
        mask = cv2.inRange(hsv, lower_white, upper_white)
        
        # Apply morphological operations to clean up noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        
        # Find contours
        contours, _ = cv2.findContours(
            mask, 
            cv2.RETR_EXTERNAL, 
            cv2.CHAIN_APPROX_SIMPLE
        )
        
        # Draw mask overlay for debugging
        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv_image = cv2.addWeighted(cv_image, 0.7, mask_colored, 0.3, 0)
        
        if len(contours) == 0:
            return False, 0, 0, 0, cv_image
        
        # Find the largest contour (assumed to be the gate)
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        # Filter small detections
        if area < self.min_gate_area:
            return False, 0, 0, 0, cv_image
        
        # Get moments to find center
        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return False, 0, 0, 0, cv_image
        
        center_x = int(M["m10"] / M["m00"])
        center_y = int(M["m01"] / M["m00"])
        
        # Get bounding rectangle
        x, y, w, h = cv2.boundingRect(largest_contour)
        
        # Draw detection visualization
        # Contour
        cv2.drawContours(cv_image, [largest_contour], -1, (0, 255, 0), 3)
        
        # Bounding box
        cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Center point
        cv2.circle(cv_image, (center_x, center_y), 10, (0, 0, 255), -1)
        cv2.circle(cv_image, (center_x, center_y), 12, (255, 255, 255), 2)
        
        # Crosshair to image center
        cv2.line(cv_image, (self.image_width//2, 0), 
                (self.image_width//2, self.image_height), (255, 255, 0), 2)
        cv2.line(cv_image, (0, self.image_height//2), 
                (self.image_width, self.image_height//2), (255, 255, 0), 2)
        
        # Line from center to gate
        cv2.line(cv_image, (self.image_width//2, self.image_height//2),
                (center_x, center_y), (0, 255, 255), 2)
        
        # Text annotations
        cv2.putText(cv_image, f'GATE DETECTED', (x, y - 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(cv_image, f'Area: {int(area)} px', (x, y - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(cv_image, f'Center: ({center_x}, {center_y})', (x, y - 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Calculate alignment error
        error_x = center_x - self.image_width // 2
        error_text = f'Error X: {error_x:+d} px'
        cv2.putText(cv_image, error_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return True, center_x, center_y, area, cv_image
    
    def front_left_callback(self, msg):
        """Process front left camera image"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Detect gate
            detected, cx, cy, area, processed = self.detect_white_gate(cv_image)
            
            if detected:
                # Publish detection data
                detection_msg = Float32MultiArray()
                detection_msg.data = [float(cx), float(cy), float(area)]
                self.gate_detection_pub.publish(detection_msg)
                
                # Publish detection flag
                flag_msg = Bool()
                flag_msg.data = True
                self.gate_detected_flag_pub.publish(flag_msg)
                
                self.get_logger().info(
                    f'Gate detected: center=({cx}, {cy}), area={int(area)}',
                    throttle_duration_sec=1.0
                )
            else:
                # Publish no detection
                flag_msg = Bool()
                flag_msg.data = False
                self.gate_detected_flag_pub.publish(flag_msg)
            
            # Publish processed image for visualization
            try:
                processed_msg = self.bridge.cv2_to_imgmsg(processed, encoding='bgr8')
                processed_msg.header = msg.header
                self.processed_image_pub.publish(processed_msg)
            except CvBridgeError as e:
                self.get_logger().error(f'Failed to publish processed image: {str(e)}')
            
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraProcessorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
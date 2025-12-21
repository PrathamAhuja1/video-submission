#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraProcessorNode(Node):
    def __init__(self):
        super().__init__('camera_processor')
        
        self.bridge = CvBridge()
        
        # Subscribers for camera feeds
        self.front_left_sub = self.create_subscription(
            Image,
            '/orca4/front_left/image_raw',
            self.front_left_callback,
            10
        )
        
        self.front_right_sub = self.create_subscription(
            Image,
            '/orca4/front_right/image_raw',
            self.front_right_callback,
            10
        )
        
        # Publisher for gate detection results
        self.gate_detection_pub = self.create_publisher(
            Float32MultiArray,
            '/gate_detection',
            10
        )
        
        # Publisher for processed image
        self.processed_image_pub = self.create_publisher(
            Image,
            '/processed_image',
            10
        )
        
        self.get_logger().info('Camera Processor Node Started')
        
    def detect_gate(self, cv_image):
        """
        Detect white/off-white gate in the image
        Returns: (detected, center_x, center_y, area)
        """
        # Convert to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Define range for white/off-white color
        # White has high value and low saturation
        lower_white = np.array([0, 0, 180])
        upper_white = np.array([180, 50, 255])
        
        # Create mask
        mask = cv2.inRange(hsv, lower_white, upper_white)
        
        # Apply morphological operations to remove noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) == 0:
            return False, 0, 0, 0, cv_image
        
        # Find the largest contour (assumed to be the gate)
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        # Filter small detections
        if area < 500:
            return False, 0, 0, 0, cv_image
        
        # Get bounding rectangle
        x, y, w, h = cv2.boundingRect(largest_contour)
        center_x = x + w // 2
        center_y = y + h // 2
        
        # Draw detection on image
        cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)
        cv2.putText(cv_image, f'Gate: {area:.0f}px', (x, y - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return True, center_x, center_y, area, cv_image
    
    def front_left_callback(self, msg):
        """Process front left camera image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            detected, cx, cy, area, processed = self.detect_gate(cv_image)
            
            if detected:
                # Publish detection data
                detection_msg = Float32MultiArray()
                detection_msg.data = [float(cx), float(cy), float(area)]
                self.gate_detection_pub.publish(detection_msg)
                
                self.get_logger().info(f'Gate detected at ({cx}, {cy}) with area {area}')
            
            # Publish processed image
            processed_msg = self.bridge.cv2_to_imgmsg(processed, encoding='bgr8')
            self.processed_image_pub.publish(processed_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing front left image: {str(e)}')
    
    def front_right_callback(self, msg):
        """Process front right camera image (backup detection)"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            detected, cx, cy, area, processed = self.detect_gate(cv_image)
            
            if detected:
                self.get_logger().info(f'Gate also visible in right camera')
                
        except Exception as e:
            self.get_logger().error(f'Error processing front right image: {str(e)}')


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
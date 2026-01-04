#!/usr/bin/env python3
"""
Simple Flare Detector - HSV-based detection
Detects flares in underwater environment
UPDATED: New single HSV range for flare detection
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np


class SimpleFlareDetector(Node):
    def __init__(self):
        super().__init__('simple_flare_detector')

        self.bridge = CvBridge()

        # ===============================
        # HSV RANGE FOR FLARE
        # ===============================
        # UPDATED: Single optimized HSV range
        self.flare_lower = np.array([20, 55, 53])
        self.flare_upper = np.array([138, 255, 200])

        # ===============================
        # SHAPE FILTERING
        # ===============================
        self.min_area = 400
        self.min_aspect_ratio = 3.0   # thin rectangle
        self.max_aspect_ratio = 15.0

        # ===============================
        # ROS I/O
        # ===============================
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.flare_detected_pub = self.create_publisher(Bool, '/flare/detected', 10)
        self.flare_center_pub = self.create_publisher(Point, '/flare/center', 10)
        self.flare_direction_pub = self.create_publisher(Float32, '/flare/direction_error', 10)
        self.debug_pub = self.create_publisher(Image, '/flare/debug_image', 10)

        self.get_logger().info('='*70)
        self.get_logger().info('🔥 Simple Flare Detector Started')
        self.get_logger().info('='*70)
        self.get_logger().info('FLARE Detection:')
        self.get_logger().info(f'  HSV Range: Lower={self.flare_lower.tolist()}, Upper={self.flare_upper.tolist()}')
        self.get_logger().info(f'  Min Area: {self.min_area}px')
        self.get_logger().info(f'  Aspect Ratio: {self.min_aspect_ratio}-{self.max_aspect_ratio}')
        self.get_logger().info('='*70)

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        h, w = frame.shape[:2]
        debug = frame.copy()

        # ===============================
        # HSV MASK - SINGLE RANGE
        # ===============================
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Apply single optimized flare HSV range
        flare_mask = cv2.inRange(hsv, self.flare_lower, self.flare_upper)

        # Clean mask
        kernel = np.ones((5, 5), np.uint8)
        flare_mask = cv2.morphologyEx(flare_mask, cv2.MORPH_CLOSE, kernel)
        flare_mask = cv2.morphologyEx(flare_mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(flare_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        flare_detected = False
        best_contour = None
        best_area = 0

        # ===============================
        # RECTANGLE-BASED FILTERING
        # ===============================
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue

            x, y, bw, bh = cv2.boundingRect(cnt)
            if bw == 0 or bh == 0:
                continue

            aspect_ratio = max(bw, bh) / min(bw, bh)

            if not (self.min_aspect_ratio <= aspect_ratio <= self.max_aspect_ratio):
                continue

            if area > best_area:
                best_area = area
                best_contour = cnt

        # ===============================
        # OUTPUT
        # ===============================
        if best_contour is not None:
            flare_detected = True

            x, y, bw, bh = cv2.boundingRect(best_contour)
            cx = x + bw // 2
            cy = y + bh // 2

            direction_error = (cx - w / 2) / (w / 2)

            # Draw
            cv2.rectangle(debug, (x, y), (x + bw, y + bh), (0, 0, 255), 3)
            cv2.circle(debug, (cx, cy), 8, (0, 0, 255), -1)
            cv2.line(debug, (w // 2, 0), (w // 2, h), (255, 255, 0), 2)
            
            # Add text annotations
            cv2.putText(debug, f'FLARE - Area: {best_area:.0f}', 
                       (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(debug, f'Dir: {direction_error:+.3f}', 
                       (x, y+bh+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Publish data
            self.flare_detected_pub.publish(Bool(data=True))

            center_msg = Point()
            center_msg.x = float(cx)
            center_msg.y = float(cy)
            center_msg.z = 0.0
            self.flare_center_pub.publish(center_msg)

            self.flare_direction_pub.publish(Float32(data=float(direction_error)))

        else:
            self.flare_detected_pub.publish(Bool(data=False))

        # Debug image
        debug_msg = self.bridge.cv2_to_imgmsg(debug, 'bgr8')
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleFlareDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
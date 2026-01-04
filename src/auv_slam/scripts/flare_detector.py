#!/usr/bin/env python3

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
        # HSV RANGE FOR RED FLARE
        # ===============================

        # Lower=[0,39,200], Upper=[80,170,255]
        # Lower=[0,17,109], Upper=[43,114,172]


        self.red_lower1 = np.array([0,17,109])
        self.red_upper1 = np.array([43,114,172])

        self.red_lower2 = np.array([0,39,200])
        self.red_upper2 = np.array([80,170,255])

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

        self.get_logger().info('🔥 Simple HSV Flare Detector Started')

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        h, w = frame.shape[:2]
        debug = frame.copy()

        # ===============================
        # HSV MASK
        # ===============================
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
        mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Clean mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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

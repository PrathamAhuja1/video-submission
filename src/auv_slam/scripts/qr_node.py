#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from pyzbar.pyzbar import decode
from sensor_msgs.msg import CompressedImage

class QRDetector(Node):
    def __init__(self):
        super().__init__('qr_detector_node')
        
        self.image_pub = self.create_publisher(Image, '/qr/debug_image', 10)
        self.image_compressed_pub = self.create_publisher(CompressedImage, '/qr/debug_image_compressed', 10)
        self.text_pub = self.create_publisher(String, '/qr/text', 10)
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        
        self.bridge = CvBridge()
        self.get_logger().info("QR Detector Node started")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Bridge Error: {e}")
            return

        qr_codes = decode(frame)

        for obj in qr_codes:
            qr_data = obj.data.decode("utf-8")
            
            self.text_pub.publish(String(data=qr_data))
            
            points = obj.polygon
            if len(points) == 4:
                pts = np.array([[p.x, p.y] for p in points], np.int32)
                pts = pts.reshape((-1, 1, 2))
                cv2.polylines(frame, [pts], True, (0, 255, 0), 3)
            
            (x, y, w, h) = obj.rect
            cv2.putText(frame, qr_data, (x, y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            self.get_logger().info(f"Published QR: {qr_data}")

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = QRDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
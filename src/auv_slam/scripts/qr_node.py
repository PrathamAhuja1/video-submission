#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import subprocess
import os
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

WIDTH = 640
HEIGHT = 480
FRAME_SIZE = WIDTH * HEIGHT * 3

class QRDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_detector_node')
        
        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()
        
        self.debug_image_pub = self.create_publisher(Image, '/qr/debug_image', 10)
        self.text_pub = self.create_publisher(String, '/qr/text', 10)
        
        package_share_directory = get_package_share_directory('auv_slam')
        self.sdp_path = os.path.join(package_share_directory, 'config', 'stream.sdp')
        self.get_logger().info(f"Using SDP file: {self.sdp_path}")

        if not os.path.exists(self.sdp_path):
            self.get_logger().error(f"SDP file not found at: {self.sdp_path}")
 
        self.cmd = [
            'ffmpeg',
            '-protocol_whitelist', 'file,rtp,udp',
            '-i', self.sdp_path,
            '-f', 'image2pipe',
            '-pix_fmt', 'bgr24',
            '-vcodec', 'rawvideo',
            '-'
        ]
        
    def run(self):
        process = subprocess.Popen(
            self.cmd,
            stdout=subprocess.PIPE,
            bufsize=10**8
        )
        
        self.get_logger().info("QR Detector Node started")
        self.get_logger().info("Publishing to /qr/debug_image and /qr/text")
        
        while rclpy.ok():
            raw = process.stdout.read(FRAME_SIZE)
            if len(raw) != FRAME_SIZE:
                continue
            
            frame = np.frombuffer(raw, dtype=np.uint8).reshape((HEIGHT, WIDTH, 3)).copy()
            
            retval, data, points, _ = self.detector.detectAndDecodeMulti(frame)
            
            if retval and points is not None:
                for i, qr_text in enumerate(data):
                    if qr_text:
                        qr_points = points[i].astype(int)
                        
                        for j in range(4):
                            start_point = tuple(qr_points[j])
                            end_point = tuple(qr_points[(j + 1) % 4])
                            cv2.line(frame, start_point, end_point, (0, 255, 0), 3)
                        
                        text_position = (qr_points[0][0], qr_points[0][1] - 10)
                        cv2.putText(frame, qr_text, text_position, 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                        
                        msg = String()
                        msg.data = qr_text
                        self.text_pub.publish(msg)
                        self.get_logger().info(f"Detected QR {i+1}: {qr_text}")
            
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.debug_image_pub.publish(ros_image)
        
        process.terminate()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = QRDetectorNode()
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
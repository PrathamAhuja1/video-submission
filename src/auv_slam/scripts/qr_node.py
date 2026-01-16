#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import subprocess
import os
from ament_index_python.packages import get_package_share_directory

class QRDetector(Node):
    def __init__(self):
        super().__init__('qr_detector_node')
        
        # ROS2 Publishers
        self.image_pub = self.create_publisher(Image, '/qr/debug_image', 10)
        self.text_pub = self.create_publisher(String, '/qr/text', 10)
        self.bridge = CvBridge()
        
        # QR Detector initialization (using standard OpenCV detector)
        self.detector = cv2.QRCodeDetector()
        
        # Video resolution
        self.width, self.height = 640, 480
        self.frame_size = self.width * self.height * 3
        
        # Resolve SDP file path
        try:
            share_dir = get_package_share_directory('auv_slam')
            self.sdp_path = os.path.join(share_dir, 'config', 'stream.sdp')
            if not os.path.exists(self.sdp_path):
                self.get_logger().error(f"SDP file NOT found at: {self.sdp_path}")
                return
            self.get_logger().info(f"Using SDP file: {self.sdp_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to resolve share directory: {e}")
            return
        
        # FFmpeg command
        self.cmd = [
            'ffmpeg',
            '-loglevel', 'quiet',
            '-protocol_whitelist', 'file,rtp,udp',
            '-i', self.sdp_path,
            '-f', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-vf', f'scale={self.width}:{self.height}',
            '-'
        ]
        
        self.pipe = subprocess.Popen(
            self.cmd, 
            stdout=subprocess.PIPE, 
            bufsize=10**8
        )
        
        # Timer for processing frames
        self.timer = self.create_timer(0.03, self.process_frame)
        self.get_logger().info("QR Detector Node started and waiting for stream...")
    
    def process_frame(self):
        raw_frame = self.pipe.stdout.read(self.frame_size)
        if len(raw_frame) != self.frame_size:
            return
        
        # .copy() ensures the buffer is writable for drawing
        frame = np.frombuffer(raw_frame, dtype=np.uint8).reshape((self.height, self.width, 3)).copy()
        
        # Detect and decode multiple QR codes
        retval, data, points, _ = self.detector.detectAndDecodeMulti(frame)
        
        if retval and points is not None:
            for i, qr_text in enumerate(data):
                if qr_text:
                    # Publish QR text to ROS topic
                    self.text_pub.publish(String(data=qr_text))
                    self.get_logger().info(f"Detected QR {i+1}: {qr_text}")
                    
                    # Draw bounding box
                    qr_points = points[i].astype(int)
                    for j in range(4):
                        start_point = tuple(qr_points[j])
                        end_point = tuple(qr_points[(j + 1) % 4])
                        cv2.line(frame, start_point, end_point, (0, 255, 0), 3)
                    
                    # Add text label
                    text_position = (qr_points[0][0], qr_points[0][1] - 10)
                    cv2.putText(frame, qr_text, text_position, 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        
        # Publish debug image to ROS topic
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = QRDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
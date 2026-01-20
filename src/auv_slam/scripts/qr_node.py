#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import threading
import queue
import numpy as np
from qreader import QReader

class QRDetector(Node):
    def __init__(self):
        super().__init__('qr_detector_node')
        
        # Keep 'l' model for best detection capability
        self.qreader = QReader(model_size='s')
        self.bridge = CvBridge()
        self.frame_queue = queue.Queue(maxsize=1)
        
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            1)
            
        self.text_pub = self.create_publisher(String, '/qr/text', 10)
        self.image_pub = self.create_publisher(Image, '/qr/debug_image', 10)
        self.image_compressed_pub = self.create_publisher(CompressedImage, '/qr/debug_image_compressed', 10)

        self.worker_thread = threading.Thread(target=self.process_frames, daemon=True)
        self.worker_thread.start()
        
        self.get_logger().info("QR Detector Started")
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    pass
            self.frame_queue.put(frame)
        except Exception:
            pass

    def process_frames(self):
        while rclpy.ok():
            try:
                frame = self.frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            # --- HYBRID PIPELINE START ---
            
            # 1. Downscale for fast detection (0.5x scale = 1/4 pixels to process)
            scale_factor = 0.5
            small_frame = cv2.resize(frame, None, fx=scale_factor, fy=scale_factor)
            
            # 2. Detect on small frame
            detections_small = self.qreader.detect(image=small_frame, is_bgr=True)
            
            for det_small in detections_small:
                # 3. Scale coordinates back to original resolution
                # We multiply by 2.0 (1/0.5) to match the full 'frame'
                det_full = self.scale_detection(det_small, 1.0 / scale_factor)
                
                # 4. Decode using the FULL resolution frame for maximum accuracy
                content = self.qreader.decode(image=frame, detection_result=det_full)

                if content:
                    self.text_pub.publish(String(data=content))
                    
                    x1, y1, x2, y2 = map(int, det_full['bbox_xyxy'])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, content, (x1, y1 - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    self.get_logger().info(f'QR: {content}')
            
            # --- HYBRID PIPELINE END ---

            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
            
            try:
                self.image_compressed_pub.publish(self.bridge.cv2_to_compressed_imgmsg(frame))
            except Exception:
                pass

    def scale_detection(self, detection, scale):
        """Helper to resize detection coordinates safely"""
        new_det = detection.copy()
        # Scale numpy array keys
        for key in ['bbox_xyxy', 'polygon_xy', 'quad_xy', 'padded_quad_xy']:
            if key in new_det:
                new_det[key] = new_det[key] * scale
        
        # Scale tuple keys
        if 'cxcy' in new_det:
            new_det['cxcy'] = (new_det['cxcy'][0] * scale, new_det['cxcy'][1] * scale)
        if 'wh' in new_det:
            new_det['wh'] = (new_det['wh'][0] * scale, new_det['wh'][1] * scale)
            
        return new_det

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

if __name__ == "__main__":
    main()
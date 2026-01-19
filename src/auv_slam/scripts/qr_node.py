#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory


class EnhancedQRDetector(Node):
    def __init__(self):
        super().__init__('qr_detector_node')
        
        self.declare_parameter('use_wechat', True)
        self.declare_parameter('model_path', '')
        
        use_wechat = self.get_parameter('use_wechat').value
        model_path = self.get_parameter('model_path').value
        
        self.bridge = CvBridge()
        
        if not model_path:
            try:
                package_share = get_package_share_directory('auv_slam')
                model_path = os.path.join(package_share, 'config')
            except Exception as e:
                self.get_logger().warn(f'Could not find package share directory: {e}')
                model_path = os.path.join(os.path.dirname(__file__), '..', 'config')
        
        self.detector = None
        self.use_wechat = False
        
        if use_wechat:
            try:
                detect_prototxt = os.path.join(model_path, 'detect.prototxt')
                detect_caffemodel = os.path.join(model_path, 'detect.caffemodel')
                sr_prototxt = os.path.join(model_path, 'sr.prototxt')
                sr_caffemodel = os.path.join(model_path, 'sr.caffemodel')
                
                if all(os.path.exists(f) for f in [detect_prototxt, detect_caffemodel, sr_prototxt, sr_caffemodel]):
                    self.detector = cv2.wechat_qrcode_WeChatQRCode(
                        detect_prototxt,
                        detect_caffemodel,
                        sr_prototxt,
                        sr_caffemodel
                    )
                    self.use_wechat = True
                    self.get_logger().info('Using WeChat QR detector with super-resolution')
                    self.get_logger().info(f'   Model path: {model_path}')
                else:
                    missing = [f for f in [detect_prototxt, detect_caffemodel, sr_prototxt, sr_caffemodel] 
                              if not os.path.exists(f)]
                    self.get_logger().warn(f'WeChat model files not found: {missing}')
                    self.get_logger().warn('Falling back to standard OpenCV detector')
                    
            except Exception as e:
                self.get_logger().warn(f'Failed to initialize WeChat detector: {e}')
                self.get_logger().warn('Falling back to standard OpenCV detector')
        
        if not self.use_wechat:
            self.detector = cv2.QRCodeDetector()
            self.get_logger().info('Using standard OpenCV QR detector')

        
        self.detected_qrs = {}
        self.qr_counter = 0
        
        self.image_pub = self.create_publisher(Image, '/qr/debug_image', 10)
        self.image_compressed_pub = self.create_publisher(
            CompressedImage, '/qr/debug_image_compressed', 10
        )
        self.text_pub = self.create_publisher(String, '/qr/text', 10)
        
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        
        self.get_logger().info('='*70)
        self.get_logger().info('Detector Node Started')
        self.get_logger().info(f'   Detector Type: {"WeChat (Super-Resolution)" if self.use_wechat else "OpenCV Standard"}')
        self.get_logger().info('   Subscribing to: /image_raw')
        self.get_logger().info('   Publishing to:')
        self.get_logger().info('     - /qr/debug_image')
        self.get_logger().info('     - /qr/debug_image_compressed')
        self.get_logger().info('     - /qr/text')
        self.get_logger().info('='*70)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Bridge Error: {e}')
            return

        display_frame = frame.copy()
        current_qr_ids = set()
        
        if self.use_wechat:
            data, points = self.detector.detectAndDecode(frame)
            
            if len(data) > 0:
                for i, qr_data in enumerate(data):
                    if qr_data:
                        pts = points[i].astype(int)
                        
                        center_x = int(pts[:, 0].mean())
                        center_y = int(pts[:, 1].mean())
                        
                        matched = False
                        for qr_id, qr_info in self.detected_qrs.items():
                            prev_x, prev_y = qr_info['center']
                            if abs(center_x - prev_x) < 50 and abs(center_y - prev_y) < 50:
                                self.detected_qrs[qr_id]['center'] = (center_x, center_y)
                                self.detected_qrs[qr_id]['points'] = pts
                                self.detected_qrs[qr_id]['data'] = qr_data
                                current_qr_ids.add(qr_id)
                                matched = True
                                break
                        
                        if not matched:
                            self.qr_counter += 1
                            self.detected_qrs[self.qr_counter] = {
                                'data': qr_data,
                                'center': (center_x, center_y),
                                'points': pts
                            }
                            current_qr_ids.add(self.qr_counter)
                            self.get_logger().info(f'QR {self.qr_counter} detected: {qr_data}')
                            
                            text_msg = String()
                            text_msg.data = qr_data
                            self.text_pub.publish(text_msg)
        else:
            retval, data, points, _ = self.detector.detectAndDecodeMulti(frame)
            
            if retval and points is not None:
                for i, qr_data in enumerate(data):
                    if qr_data:
                        pts = points[i].astype(int)
                        
                        center_x = int(pts[:, 0].mean())
                        center_y = int(pts[:, 1].mean())
                        
                        matched = False
                        for qr_id, qr_info in self.detected_qrs.items():
                            prev_x, prev_y = qr_info['center']
                            if abs(center_x - prev_x) < 50 and abs(center_y - prev_y) < 50:
                                self.detected_qrs[qr_id]['center'] = (center_x, center_y)
                                self.detected_qrs[qr_id]['points'] = pts
                                self.detected_qrs[qr_id]['data'] = qr_data
                                current_qr_ids.add(qr_id)
                                matched = True
                                break
                        
                        if not matched:
                            self.qr_counter += 1
                            self.detected_qrs[self.qr_counter] = {
                                'data': qr_data,
                                'center': (center_x, center_y),
                                'points': pts
                            }
                            current_qr_ids.add(self.qr_counter)
                            self.get_logger().info(f'QR {self.qr_counter} detected: {qr_data}')
                            
                            text_msg = String()
                            text_msg.data = qr_data
                            self.text_pub.publish(text_msg)
        
        stale_qrs = set(self.detected_qrs.keys()) - current_qr_ids
        for qr_id in stale_qrs:
            del self.detected_qrs[qr_id]
        
        for qr_id, qr_info in self.detected_qrs.items():
            pts = qr_info['points']
            qr_data = qr_info['data']
            
            for j in range(len(pts)):
                cv2.line(
                    display_frame,
                    tuple(pts[j]),
                    tuple(pts[(j + 1) % len(pts)]),
                    (0, 255, 0), 2
                )
            
            x, y = pts[0]
            cv2.putText(
                display_frame, qr_data, (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
            )
        
        detector_type = "WeChat SR" if self.use_wechat else "OpenCV"
        cv2.putText(
            display_frame,
            f'{detector_type} | QR Codes: {len(self.detected_qrs)}',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
        )
        
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(display_frame, encoding='bgr8')
            debug_msg.header = msg.header
            self.image_pub.publish(debug_msg)
            
            success, buffer = cv2.imencode('.jpg', display_frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if success:
                compressed_msg = CompressedImage()
                compressed_msg.header = msg.header
                compressed_msg.format = 'jpeg'
                compressed_msg.data = buffer.tobytes()
                self.image_compressed_pub.publish(compressed_msg)
                
        except Exception as e:
            self.get_logger().error(f'Publishing error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = EnhancedQRDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
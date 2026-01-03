#!/usr/bin/env python3
"""
Stream any ROS Image topic over UDP
Usage: ros2 run auv_slam ros_gst_udp.py
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class RosTopicUdpStreamer(Node):
    def __init__(self):
        super().__init__('ros_topic_udp_streamer')
        
        # Parameters
        self.declare_parameter('topic_name', '/gate/debug_image')
        self.declare_parameter('target_ip', '192.168.11.4')
        self.declare_parameter('target_port', 5600)
        self.declare_parameter('fps', 30)
        self.declare_parameter('width', 640)  # Resize if needed
        self.declare_parameter('height', 480)
        
        topic = self.get_parameter('topic_name').value
        ip = self.get_parameter('target_ip').value
        port = self.get_parameter('target_port').value
        fps = self.get_parameter('fps').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        
        # GStreamer pipeline for UDP streaming
        gst_str = (
            f'appsrc ! '
            f'videoconvert ! '
            f'video/x-raw,format=I420 ! '
            f'x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast key-int-max=30 ! '
            f'rtph264pay config-interval=1 pt=96 ! '
            f'udpsink host={ip} port={port} sync=false'
        )
        
        self.get_logger().info(f'Streaming {topic} -> {ip}:{port}')
        
        # Initialize VideoWriter with GStreamer
        self.out = cv2.VideoWriter(
            gst_str,
            cv2.CAP_GSTREAMER,
            0,  # Ignored for appsrc
            fps,
            (self.width, self.height),
            True
        )
        
        if not self.out.isOpened():
            self.get_logger().error('Failed to open GStreamer pipeline!')
            raise RuntimeError('GStreamer pipeline failed')
        
        self.bridge = CvBridge()
        
        # Subscribe to ROS topic
        self.sub = self.create_subscription(
            Image,
            topic,
            self.image_callback,
            10
        )
        
        self.frame_count = 0

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Resize if needed
            if cv_img.shape[1] != self.width or cv_img.shape[0] != self.height:
                cv_img = cv2.resize(cv_img, (self.width, self.height))
            
            # Write to GStreamer pipeline (streams over UDP)
            self.out.write(cv_img)
            
            self.frame_count += 1
            if self.frame_count % 100 == 0:
                self.get_logger().info(f'Streamed {self.frame_count} frames')
                
        except Exception as e:
            self.get_logger().error(f'Stream error: {e}')

    def destroy_node(self):
        if self.out:
            self.out.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RosTopicUdpStreamer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
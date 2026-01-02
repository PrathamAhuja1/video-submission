#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class RosUdpStreamer(Node):
    def __init__(self):
        super().__init__('ros_udp_streamer')
        
        # === CONFIGURATION ===
        self.declare_parameter('topic_name', '/image_raw')
        self.declare_parameter('target_ip', '192.168.11.4')
        self.declare_parameter('target_port', 5600)
        self.declare_parameter('fps', 30)
        
        topic = self.get_parameter('topic_name').value
        ip = self.get_parameter('target_ip').value
        port = self.get_parameter('target_port').value
        self.fps = self.get_parameter('fps').value
        
        # === GSTREAMER PIPELINE (ROS -> UDP) ===
        # appsrc: Takes frames from Python (OpenCV)
        # x264enc: Compresses them for network
        # udpsink: Sends to your laptop
        self.gst_str = (
            f'appsrc ! videoconvert ! '
            f'x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast ! '
            f'rtph264pay config-interval=1 pt=96 ! '
            f'udpsink host={ip} port={port} sync=false'
        )
        
        self.get_logger().info(f"Streaming {topic} -> {ip}:{port}")
        
        # Initialize Video Writer
        self.out = None
        self.bridge = CvBridge()
        
        # Subscribe to the ROS Topic you want to see
        self.sub = self.create_subscription(
            Image, 
            topic, 
            self.image_callback, 
            10
        )

    def image_callback(self, msg):
        try:
            # Convert ROS Image -> OpenCV
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Initialize GStreamer on first frame (once we know size)
            if self.out is None:
                h, w = cv_img.shape[:2]
                self.out = cv2.VideoWriter(
                    self.gst_str, 
                    cv2.CAP_GSTREAMER, 
                    0, 
                    self.fps, 
                    (w, h), 
                    True
                )
                if not self.out.isOpened():
                    self.get_logger().error("Failed to open GStreamer pipeline!")
            
            # Write frame to GStreamer
            self.out.write(cv_img)
            
        except Exception as e:
            self.get_logger().error(f"Stream Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RosUdpStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.out: node.out.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
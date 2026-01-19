#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from pyzbar.pyzbar import decode
import threading
import queue
import time

class QRDetector(Node):
    def __init__(self):
        super().__init__('qr_detector_node')

        # Publishers
        self.image_pub = self.create_publisher(Image, '/qr/debug_image', 10)
        self.image_compressed_pub = self.create_publisher(CompressedImage, '/qr/debug_image_compressed', 10)
        self.text_pub = self.create_publisher(String, '/qr/text', 10)

        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()

        # Queue to hand off frames to the worker thread.
        # Keep maxsize small to avoid memory growth and latency.
        self.frame_queue = queue.Queue(maxsize=4)

        # Worker thread control
        self.running = True
        self.worker_thread = threading.Thread(target=self._decode_worker, daemon=True)
        self.worker_thread.start()

        self.get_logger().info("QR Detector Node initialized. Awaiting frames on /image_raw...")

    def image_callback(self, msg: Image):
        """Quickly convert incoming ROS Image to cv2 frame and enqueue it for decoding.
        If the queue is full, drop the frame to avoid blocking the callback."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Try to enqueue without blocking. Drop frame if queue is full.
        try:
            self.frame_queue.put_nowait(frame)
        except queue.Full:
            # Throttle logging to avoid spamming; it's okay to occasionally drop frames.
            # Use debug level to reduce log noise during normal operation.
            self.get_logger().debug("Frame queue full — dropping frame to keep callback fast.")

    def _decode_worker(self):
        """Worker thread: pull frames from the queue, decode QR codes, annotate and publish."""
        while self.running:
            try:
                frame = self.frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            start_time = self.get_clock().now()
            try:
                qr_codes = decode(frame)
            except Exception as e:
                self.get_logger().error(f"Error during decode: {e}")
                qr_codes = []
            end_time = self.get_clock().now()
            duration_ms = (end_time - start_time).nanoseconds / 1e6

            if not qr_codes:
                # Reduced noise: debug-level periodic message
                self.get_logger().debug(f"Scan complete ({duration_ms:.2f}ms): No QR codes found.")
            else:
                self.get_logger().info(f"Found {len(qr_codes)} QR code(s) in {duration_ms:.2f}ms")

            # Annotate and publish results
            annotated = frame.copy()
            for obj in qr_codes:
                try:
                    qr_data = obj.data.decode("utf-8")
                except Exception:
                    qr_data = ""

                if qr_data:
                    # Publish text
                    try:
                        self.text_pub.publish(String(data=qr_data))
                    except Exception as e:
                        self.get_logger().error(f"Failed to publish QR text: {e}")

                    self.get_logger().info(f"Captured Data: '{qr_data}'")

                    # Draw polygon/bounding box if available
                    points = obj.polygon
                    if points and len(points) >= 3:
                        pts = np.array([[p.x, p.y] for p in points], np.int32)
                        pts = pts.reshape((-1, 1, 2))
                        cv2.polylines(annotated, [pts], True, (0, 255, 0), 3)

                    # Draw text label near bounding rect
                    (x, y, w, h) = obj.rect
                    # clamp coordinates
                    x = max(0, x)
                    y = max(0, y)
                    cv2.putText(annotated, qr_data, (x, max(10, y - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                else:
                    self.get_logger().warning("Detected QR code but could not extract data.")

            # Publish annotated image
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8"))
            except Exception as e:
                self.get_logger().error(f"Failed to publish debug image: {e}")

            # Publish compressed image (useful for lower-bandwidth viewers)
            try:
                compressed_msg = self.bridge.cv2_to_compressed_imgmsg(annotated)
                self.image_compressed_pub.publish(compressed_msg)
            except Exception as e:
                self.get_logger().error(f"Compression/publish failed: {e}")

            # Mark task done for queue bookkeeping
            try:
                self.frame_queue.task_done()
            except Exception:
                pass

        self.get_logger().info("Decode worker thread exiting.")

    def shutdown(self):
        """Signal the worker thread to stop and wait for it to finish."""
        self.get_logger().info("Shutting down QR Detector worker thread...")
        self.running = False
        # Wait briefly for thread to exit
        self.worker_thread.join(timeout=1.0)
        # Optionally clear the queue
        with self.frame_queue.mutex:
            self.frame_queue.queue.clear()

def main(args=None):
    rclpy.init(args=args)
    node = QRDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received — shutting down.")
    finally:
        # Ensure worker thread stops before destroying node
        try:
            node.shutdown()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Ball Follower - Continuous ball tracking and following
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist, Vector3
import time
import math


class BallFollower(Node):
    def __init__(self):
        super().__init__('ball_follower')
        
        self.STABILIZING = 0
        self.SEARCHING = 1
        self.FOLLOWING = 2
        
        self.state = self.STABILIZING
        
        self.TARGET_DEPTH = 0.6
        self.DEPTH_TOLERANCE = 0.1
        
        self.MIN_FOLLOW_DISTANCE = 0.3
        self.MAX_FOLLOW_DISTANCE = 5.0
        
        self.SEARCH_SPEED = 0.7
        self.FOLLOW_SPEED_MIN = 0.25
        self.FOLLOW_SPEED_MAX = 0.95
        
        self.SEARCH_YAW_RATE = 0.5
        self.YAW_GAIN_FAR = 3.5
        self.YAW_GAIN_CLOSE = 2.5
        self.YAW_TRANSITION_DISTANCE = 1.0
        
        self.ALIGNMENT_THRESHOLD = 0.1
        
        self.ball_detected = False
        self.alignment_error = 0.0
        self.estimated_distance = 999.0
        
        self.current_depth = 0.0
        self.current_yaw = 0.0
        
        self.state_start_time = time.time()
        self.last_detection_time = 0.0
        self.detection_timeout = 3.0
        
        self.create_subscription(Bool, '/ball/detected', 
                                self.ball_detected_callback, 10)
        self.create_subscription(Float32, '/ball/alignment_error',
                                self.alignment_callback, 10)
        self.create_subscription(Float32, '/ball/estimated_distance',
                                self.distance_callback, 10)
        self.create_subscription(Float32, '/depth',
                                self.depth_callback, 10)
        self.create_subscription(Vector3, '/vn100/ypr',
                                self.ypr_callback, 10)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('Ball Follower initialized')
        self.get_logger().info(f'Follow distance range: {self.MIN_FOLLOW_DISTANCE}-{self.MAX_FOLLOW_DISTANCE}m')
        self.get_logger().info(f'Speed range: {self.FOLLOW_SPEED_MIN}-{self.FOLLOW_SPEED_MAX}')
    
    def ball_detected_callback(self, msg: Bool):
        was_detected = self.ball_detected
        self.ball_detected = msg.data
        
        if self.ball_detected:
            self.last_detection_time = time.time()
            
            if not was_detected and self.state == self.SEARCHING:
                self.get_logger().info(f'Ball detected at {self.estimated_distance:.2f}m')
    
    def alignment_callback(self, msg: Float32):
        self.alignment_error = msg.data
    
    def distance_callback(self, msg: Float32):
        self.estimated_distance = msg.data
    
    def depth_callback(self, msg: Float32):
        self.current_depth = msg.data
    
    def ypr_callback(self, msg: Vector3):
        self.current_yaw = math.radians(msg.x)
    
    def control_loop(self):
        cmd = Twist()
        
        cmd.linear.z = self.compute_depth_control()
        
        if self.state == self.STABILIZING:
            cmd = self.stabilizing(cmd)
        elif self.state == self.SEARCHING:
            cmd = self.searching(cmd)
        elif self.state == self.FOLLOWING:
            cmd = self.following(cmd)
        
        self.cmd_vel_pub.publish(cmd)
    
    def compute_depth_control(self) -> float:
        depth_error = self.TARGET_DEPTH - self.current_depth
        
        if abs(depth_error) < self.DEPTH_TOLERANCE:
            return 0.0
        
        z_cmd = depth_error * 0.8
        return max(-0.6, min(z_cmd, 0.6))
    
    def stabilizing(self, cmd: Twist) -> Twist:
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        
        depth_stable = abs(self.TARGET_DEPTH - self.current_depth) < 0.15
        elapsed = time.time() - self.state_start_time
        
        if depth_stable and elapsed > 3.0:
            self.get_logger().info('Stabilized - Starting ball search')
            self.transition_to(self.SEARCHING)
        
        return cmd
    
    def searching(self, cmd: Twist) -> Twist:
        if self.ball_detected and self.estimated_distance < 999:
            self.get_logger().info('Ball found - Starting tracking')
            self.transition_to(self.FOLLOWING)
            return cmd
        
        cmd.linear.x = self.SEARCH_SPEED
        
        elapsed = time.time() - self.state_start_time
        sweep_phase = (elapsed % 10.0) / 10.0
        
        if sweep_phase < 0.5:
            cmd.angular.z = self.SEARCH_YAW_RATE
        else:
            cmd.angular.z = -self.SEARCH_YAW_RATE
        
        return cmd
    
    def following(self, cmd: Twist) -> Twist:
        if not self.ball_detected or (time.time() - self.last_detection_time) > self.detection_timeout:
            self.get_logger().warn('Ball lost - returning to search')
            self.transition_to(self.SEARCHING)
            return cmd
        
        distance_clamped = max(self.MIN_FOLLOW_DISTANCE, 
                              min(self.estimated_distance, self.MAX_FOLLOW_DISTANCE))
        
        distance_normalized = (distance_clamped - self.MIN_FOLLOW_DISTANCE) / \
                             (self.MAX_FOLLOW_DISTANCE - self.MIN_FOLLOW_DISTANCE)
        
        speed = self.FOLLOW_SPEED_MIN + distance_normalized * \
                (self.FOLLOW_SPEED_MAX - self.FOLLOW_SPEED_MIN)
        
        if self.estimated_distance > self.YAW_TRANSITION_DISTANCE:
            yaw_gain = self.YAW_GAIN_FAR
        else:
            blend = (self.estimated_distance - self.MIN_FOLLOW_DISTANCE) / \
                    (self.YAW_TRANSITION_DISTANCE - self.MIN_FOLLOW_DISTANCE)
            blend = max(0.0, min(1.0, blend))
            yaw_gain = self.YAW_GAIN_CLOSE + blend * (self.YAW_GAIN_FAR - self.YAW_GAIN_CLOSE)
        
        if abs(self.alignment_error) > 0.3:
            speed *= 0.6
        elif abs(self.alignment_error) > 0.15:
            speed *= 0.8
        
        cmd.linear.x = speed
        cmd.angular.z = -self.alignment_error * yaw_gain
        
        if self.frame_count % 20 == 0:
            self.get_logger().info(
                f'Following: {self.estimated_distance:.2f}m, Speed: {speed:.2f}, Align: {self.alignment_error:+.3f}',
                throttle_duration_sec=0.9
            )
        
        return cmd
    
    def transition_to(self, new_state: int):
        state_names = {
            self.STABILIZING: 'STABILIZING',
            self.SEARCHING: 'SEARCHING',
            self.FOLLOWING: 'FOLLOWING'
        }
        
        old_name = state_names.get(self.state, 'UNKNOWN')
        self.state = new_state
        self.state_start_time = time.time()
        new_name = state_names.get(new_state, 'UNKNOWN')
        
        self.get_logger().info(f'STATE: {old_name} -> {new_name}')
    
    frame_count = 0
    
    def control_loop(self):
        self.frame_count += 1
        cmd = Twist()
        
        cmd.linear.z = self.compute_depth_control()
        
        if self.state == self.STABILIZING:
            cmd = self.stabilizing(cmd)
        elif self.state == self.SEARCHING:
            cmd = self.searching(cmd)
        elif self.state == self.FOLLOWING:
            cmd = self.following(cmd)
        
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BallFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_cmd = Twist()
        node.cmd_vel_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
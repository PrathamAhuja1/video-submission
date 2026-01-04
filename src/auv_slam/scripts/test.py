#!/usr/bin/env python3
"""
Test Sequence Node - Automated Movement Test
Performs sequential movements to test all thruster axes:
1. Heave DOWN for 4 seconds
2. Heave UP for 3 seconds
3. Surge (forward) for 3 seconds
4. Yaw left for 3 seconds
5. Yaw right for 3 seconds
6. Return to neutral
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time


class TestSequenceNode(Node):
    def __init__(self):
        super().__init__('test_sequence_node')
        
        # Test parameters
        self.declare_parameter('heave_speed', 0.9)   # 1680 PWM
        self.declare_parameter('surge_speed', 0.95)  # 1690 PWM
        self.declare_parameter('yaw_speed', 0.5)
        self.declare_parameter('heave_down_duration', 5.0)  # seconds for heave down
        self.declare_parameter('heave_up_duration', 3.0)    # seconds for heave up
        self.declare_parameter('test_duration', 1.0)  # seconds per other tests
        self.declare_parameter('pause_duration', 1.0)  # seconds between tests
        
        self.heave_speed = self.get_parameter('heave_speed').value
        self.surge_speed = self.get_parameter('surge_speed').value
        self.yaw_speed = self.get_parameter('yaw_speed').value
        self.heave_down_duration = self.get_parameter('heave_down_duration').value
        self.heave_up_duration = self.get_parameter('heave_up_duration').value
        self.test_duration = self.get_parameter('test_duration').value
        self.pause_duration = self.get_parameter('pause_duration').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/test_sequence/status', 10)
        
        # State machine
        self.INIT = 0
        self.HEAVE_DOWN = 1       
        self.PAUSE_1 = 2
        self.HEAVE_UP = 3        
        self.PAUSE_2 = 4
        self.SURGE = 5
        self.PAUSE_3 = 6
        self.YAW_LEFT = 7
        self.PAUSE_4 = 8
        self.YAW_RIGHT = 9
        self.PAUSE_5 = 10
        self.COMPLETE = 11
        
        self.state = self.INIT
        self.state_start_time = None
        
        # Control loop at 20Hz
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('='*70)
        self.get_logger().info('🧪 Test Sequence Node Started')
        self.get_logger().info('='*70)
        self.get_logger().info(f'  Heave Speed: ±{self.heave_speed}')
        self.get_logger().info(f'  Heave Down Duration: {self.heave_down_duration}s')
        self.get_logger().info(f'  Heave Up Duration: {self.heave_up_duration}s')
        self.get_logger().info(f'  Surge Speed: {self.surge_speed}')
        self.get_logger().info(f'  Yaw Speed: ±{self.yaw_speed}')
        self.get_logger().info(f'  Other Test Duration: {self.test_duration}s each')
        self.get_logger().info(f'  Pause Duration: {self.pause_duration}s')
        self.get_logger().info('='*70)
        self.get_logger().info('')
        self.get_logger().info('Test Sequence:')
        self.get_logger().info(f'  1. Heave DOWN for {self.heave_down_duration}s')
        self.get_logger().info(f'  2. Heave UP for {self.heave_up_duration}s')
        self.get_logger().info(f'  3. Surge FORWARD for {self.test_duration}s')
        self.get_logger().info(f'  4. Yaw LEFT for {self.test_duration}s')
        self.get_logger().info(f'  5. Yaw RIGHT for {self.test_duration}s')
        self.get_logger().info('')
        self.get_logger().info('Starting test sequence in 3 seconds...')
        self.get_logger().info('')
        
    def control_loop(self):
        """Main control loop - executes test sequence"""
        
        # Initialize state timing
        if self.state_start_time is None:
            self.state_start_time = time.time()
        
        elapsed = time.time() - self.state_start_time
        cmd = Twist()
        
        # State machine
        if self.state == self.INIT:
            if elapsed >= 3.0:  # 3 second initialization
                self.transition_to(self.HEAVE_DOWN)
            else:
                self.publish_status(f"Initializing... {3.0 - elapsed:.1f}s")
        
        elif self.state == self.HEAVE_DOWN:
            cmd.linear.z = -self.heave_speed  # Negative = down (FIRST: 4 seconds)
            self.publish_status(f"HEAVE DOWN | {elapsed:.1f}s / {self.heave_down_duration}s")
            
            if elapsed >= self.heave_down_duration:
                self.transition_to(self.PAUSE_1)
        
        elif self.state == self.PAUSE_1:
            cmd = Twist()  # Neutral
            self.publish_status(f"Pause {elapsed:.1f}s / {self.pause_duration}s")
            
            if elapsed >= self.pause_duration:
                self.transition_to(self.HEAVE_UP)
        
        elif self.state == self.HEAVE_UP:
            cmd.linear.z = self.heave_speed  # Positive = up (SECOND: 3 seconds)
            self.publish_status(f"HEAVE UP | {elapsed:.1f}s / {self.heave_up_duration}s")
            
            if elapsed >= self.heave_up_duration:
                self.transition_to(self.PAUSE_2)
        
        elif self.state == self.PAUSE_2:
            cmd = Twist()
            self.publish_status(f"Pause {elapsed:.1f}s / {self.pause_duration}s")
            
            if elapsed >= self.pause_duration:
                self.transition_to(self.SURGE)
        
        elif self.state == self.SURGE:
            cmd.linear.x = self.surge_speed  # Forward
            self.publish_status(f"SURGE FORWARD | {elapsed:.1f}s / {self.test_duration}s")
            
            if elapsed >= self.test_duration:
                self.transition_to(self.PAUSE_3)
        
        elif self.state == self.PAUSE_3:
            cmd = Twist()
            self.publish_status(f"Pause {elapsed:.1f}s / {self.pause_duration}s")
            
            if elapsed >= self.pause_duration:
                self.transition_to(self.YAW_LEFT)
        
        elif self.state == self.YAW_LEFT:
            cmd.angular.z = self.yaw_speed  # Positive = left
            self.publish_status(f"YAW LEFT | {elapsed:.1f}s / {self.test_duration}s")
            
            if elapsed >= self.test_duration:
                self.transition_to(self.PAUSE_4)
        
        elif self.state == self.PAUSE_4:
            cmd = Twist()
            self.publish_status(f"Pause {elapsed:.1f}s / {self.pause_duration}s")
            
            if elapsed >= self.pause_duration:
                self.transition_to(self.YAW_RIGHT)
        
        elif self.state == self.YAW_RIGHT:
            cmd.angular.z = -self.yaw_speed  # Negative = right
            self.publish_status(f"YAW RIGHT | {elapsed:.1f}s / {self.test_duration}s")
            
            if elapsed >= self.test_duration:
                self.transition_to(self.PAUSE_5)
        
        elif self.state == self.PAUSE_5:
            cmd = Twist()
            self.publish_status(f"Final pause {elapsed:.1f}s / {self.pause_duration}s")
            
            if elapsed >= self.pause_duration:
                self.transition_to(self.COMPLETE)
        
        elif self.state == self.COMPLETE:
            cmd = Twist()  # Neutral forever
            if elapsed < 2.0:  # Only log for first 2 seconds
                self.publish_status("✅ TEST SEQUENCE COMPLETE")
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
    
    def transition_to(self, new_state):
        """Transition to a new state"""
        state_names = {
            self.INIT: 'INIT',
            self.HEAVE_DOWN: 'HEAVE_DOWN',
            self.PAUSE_1: 'PAUSE_1',
            self.HEAVE_UP: 'HEAVE_UP',
            self.PAUSE_2: 'PAUSE_2',
            self.SURGE: 'SURGE',
            self.PAUSE_3: 'PAUSE_3',
            self.YAW_LEFT: 'YAW_LEFT',
            self.PAUSE_4: 'PAUSE_4',
            self.YAW_RIGHT: 'YAW_RIGHT',
            self.PAUSE_5: 'PAUSE_5',
            self.COMPLETE: 'COMPLETE'
        }
        
        old_name = state_names.get(self.state, 'UNKNOWN')
        new_name = state_names.get(new_state, 'UNKNOWN')
        
        self.get_logger().info('')
        self.get_logger().info(f'🔄 {old_name} → {new_name}')
        self.get_logger().info('')
        
        self.state = new_state
        self.state_start_time = time.time()
    
    def publish_status(self, status_text):
        """Publish status message"""
        msg = String()
        msg.data = status_text
        self.status_pub.publish(msg)
        
        # Also log to console every second
        if hasattr(self, '_last_log_time'):
            if time.time() - self._last_log_time >= 1.0:
                self.get_logger().info(status_text)
                self._last_log_time = time.time()
        else:
            self._last_log_time = time.time()


def main(args=None):
    rclpy.init(args=args)
    node = TestSequenceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command
        stop_cmd = Twist()
        node.cmd_vel_pub.publish(stop_cmd)
        node.get_logger().info('Emergency stop sent')
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
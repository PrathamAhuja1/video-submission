#!/usr/bin/env python3
"""
Heave PID Controller - Maintains target depth using PID control
Subscribes to /depth, outputs vertical thrust commands via /cmd_vel
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Twist
import time


class PIDController:
    """Simple PID controller for depth control"""
    def __init__(self, kp=1.0, ki=0.0, kd=0.2, output_limits=(-1.0, 1.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        
    def reset(self):
        """Reset PID state"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        
    def compute(self, setpoint, measured_value):
        """
        Compute PID output
        Returns: control output value
        """
        current_time = time.time()
        
        # Calculate error
        error = setpoint - measured_value
        
        # Initialize time on first call
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error = error
            return 0.0
        
        # Calculate dt
        dt = current_time - self.prev_time
        if dt <= 0.0:
            dt = 0.01  # Prevent division by zero
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        # Anti-windup: clamp integral
        max_integral = abs(self.output_limits[1] - self.output_limits[0]) / 2.0
        self.integral = max(-max_integral, min(self.integral, max_integral))
        i_term = self.ki * self.integral
        
        # Derivative term
        d_term = self.kd * (error - self.prev_error) / dt
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # Apply output limits
        output = max(self.output_limits[0], min(output, self.output_limits[1]))
        
        # Store for next iteration
        self.prev_error = error
        self.prev_time = current_time
        
        return output


class HeavePIDController(Node):
    def __init__(self):
        super().__init__('heave_pid_controller')
        
        # Declare parameters
        self.declare_parameter('target_depth', 1.0)  # meters
        self.declare_parameter('depth_tolerance', 0.1)  # meters
        self.declare_parameter('kp', 1.2)
        self.declare_parameter('ki', 0.05)
        self.declare_parameter('kd', 0.3)
        self.declare_parameter('max_output', 0.6)  # Max vertical velocity
        self.declare_parameter('control_rate', 20.0)  # Hz
        self.declare_parameter('enable_on_start', True)
        
        # Get parameters
        self.target_depth = self.get_parameter('target_depth').value
        self.depth_tolerance = self.get_parameter('depth_tolerance').value
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        max_output = self.get_parameter('max_output').value
        control_rate = self.get_parameter('control_rate').value
        
        # Initialize PID controller
        self.pid = PIDController(
            kp=kp,
            ki=ki,
            kd=kd,
            output_limits=(-max_output, max_output)
        )
        
        # State variables
        self.current_depth = 0.0
        self.depth_received = False
        self.enabled = self.get_parameter('enable_on_start').value
        self.stabilized = False
        self.stabilization_count = 0
        
        # Subscriptions
        self.depth_sub = self.create_subscription(
            Float32,
            '/depth',
            self.depth_callback,
            10
        )
        
        self.enable_sub = self.create_subscription(
            Bool,
            '/heave_pid/enable',
            self.enable_callback,
            10
        )
        
        self.setpoint_sub = self.create_subscription(
            Float32,
            '/heave_pid/setpoint',
            self.setpoint_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_heave', 10)
        self.error_pub = self.create_publisher(Float32, '/heave_pid/error', 10)
        self.output_pub = self.create_publisher(Float32, '/heave_pid/output', 10)
        self.status_pub = self.create_publisher(String, '/heave_pid/status', 10)
        
        # Control timer
        control_period = 1.0 / control_rate
        self.control_timer = self.create_timer(control_period, self.control_loop)
        
        self.get_logger().info('='*70)
        self.get_logger().info('🎚️ Heave PID Controller Initialized')
        self.get_logger().info('='*70)
        self.get_logger().info(f'  Target Depth: {self.target_depth:.2f}m')
        self.get_logger().info(f'  PID Gains: Kp={kp}, Ki={ki}, Kd={kd}')
        self.get_logger().info(f'  Max Output: ±{max_output}')
        self.get_logger().info(f'  Control Rate: {control_rate}Hz')
        self.get_logger().info(f'  Status: {"ENABLED" if self.enabled else "DISABLED"}')
        self.get_logger().info('='*70)
        
    def depth_callback(self, msg: Float32):
        """Receive current depth measurement"""
        self.current_depth = msg.data
        if not self.depth_received:
            self.depth_received = True
            self.get_logger().info(f'📊 First depth reading: {self.current_depth:.3f}m')
    
    def enable_callback(self, msg: Bool):
        """Enable/disable PID controller"""
        was_enabled = self.enabled
        self.enabled = msg.data
        
        if self.enabled and not was_enabled:
            # Reset PID when re-enabling
            self.pid.reset()
            self.stabilized = False
            self.stabilization_count = 0
            self.get_logger().info('✅ Heave PID Controller ENABLED')
        elif not self.enabled and was_enabled:
            self.get_logger().info('⏸️ Heave PID Controller DISABLED')
    
    def setpoint_callback(self, msg: Float32):
        """Update target depth setpoint"""
        old_target = self.target_depth
        self.target_depth = msg.data
        self.stabilized = False
        self.stabilization_count = 0
        self.get_logger().info(
            f'🎯 Target depth changed: {old_target:.2f}m → {self.target_depth:.2f}m'
        )
    
    def control_loop(self):
        """Main PID control loop"""
        
        # Check if we have depth data
        if not self.depth_received:
            return
        
        # Calculate error
        error = self.target_depth - self.current_depth
        
        # Publish error
        error_msg = Float32()
        error_msg.data = float(error)
        self.error_pub.publish(error_msg)
        
        # Initialize command
        cmd = Twist()
        
        if not self.enabled:
            # Controller disabled - send zero command
            cmd.linear.z = 0.0
            output = 0.0
            status = "DISABLED"
        else:
            # Controller enabled - compute PID
            output = self.pid.compute(self.target_depth, self.current_depth)
            
            # Note: Negative output for positive depth error
            # (need to go down if current < target)
            cmd.linear.z = -output
            
            # Check if stabilized
            if abs(error) < self.depth_tolerance:
                self.stabilization_count += 1
                if self.stabilization_count > 20:  # ~1 second at 20Hz
                    self.stabilized = True
            else:
                self.stabilization_count = 0
                self.stabilized = False
            
            status = "STABLE" if self.stabilized else "ACTIVE"
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
        
        # Publish output
        output_msg = Float32()
        output_msg.data = float(output)
        self.output_pub.publish(output_msg)
        
        # Publish status
        status_msg = String()
        status_msg.data = f"{status} | Error: {error:+.3f}m | Output: {output:+.3f}"
        self.status_pub.publish(status_msg)
        
        # Periodic logging
        if hasattr(self, '_log_counter'):
            self._log_counter += 1
        else:
            self._log_counter = 0
        
        if self._log_counter % 40 == 0:  # Every 2 seconds at 20Hz
            self.get_logger().info(
                f'📊 {status} | Depth: {self.current_depth:.3f}m | '
                f'Target: {self.target_depth:.3f}m | Error: {error:+.3f}m | '
                f'Output: {output:+.3f}',
                throttle_duration_sec=1.9
            )


def main(args=None):
    rclpy.init(args=args)
    node = HeavePIDController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send zero command on exit
        stop_cmd = Twist()
        node.cmd_vel_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
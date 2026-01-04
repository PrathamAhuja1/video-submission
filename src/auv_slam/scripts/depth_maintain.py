#!/usr/bin/env python3
"""
Depth Maintainer - Maintain specific depth via terminal input
Usage: ros2 run auv_slam depth_maintainer.py
Or: python3 depth_maintainer.py <depth_in_meters>

Examples:
  python3 depth_maintainer.py 0.5      # Maintain 0.5m depth
  python3 depth_maintainer.py 1.0      # Maintain 1.0m depth
  python3 depth_maintainer.py 0.8      # Maintain 0.8m depth

Features:
  - Simple PID depth control
  - Real-time depth monitoring
  - Status display
  - Safety limits
  - Easy emergency stop (Ctrl+C)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import sys
import time
import argparse


class DepthMaintainer(Node):
    def __init__(self, target_depth=0.6):
        super().__init__('depth_maintainer_node')
        
        # Target depth in meters
        self.target_depth = target_depth
        
        # Safety constraints
        self.MIN_DEPTH = -0.2  # 20cm below surface
        self.MAX_DEPTH = 1.3   # 1.3m max (pool depth: 1.37m)
        self.DEPTH_TOLERANCE = 0.05  # ±5cm tolerance for "stable"
        
        # PID Controller Parameters
        self.Kp = 1.2   # Proportional gain
        self.Ki = 0.08  # Integral gain
        self.Kd = 0.4   # Derivative gain
        
        # Output limits (heave velocity command range)
        self.MAX_HEAVE_OUTPUT = 0.7   # Maximum upward/downward command
        
        # State tracking
        self.current_depth = 0.0
        self.previous_error = 0.0
        self.integral_error = 0.0
        self.last_time = time.time()
        
        self.depth_samples = 0
        self.stable_count = 0
        self.unstable_count = 0
        
        # Statistics
        self.start_time = time.time()
        self.max_error = 0.0
        self.min_depth_achieved = 999.0
        self.max_depth_achieved = -999.0
        
        # Subscribe to depth sensor
        self.depth_sub = self.create_subscription(
            Float32,
            '/depth',
            self.depth_callback,
            10
        )
        
        # Publish heave commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control loop at 20Hz
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        # Status display timer (1Hz)
        self.status_timer = self.create_timer(1.0, self.display_status)
        
        # Print header
        self.print_header()
    
    def print_header(self):
        """Print startup header with information"""
        print("\n" + "="*80)
        print("🎚️  DEPTH MAINTAINER - ROS2 Underwater Depth Control")
        print("="*80)
        print(f"Target Depth: {self.target_depth:.3f} m")
        print(f"Safety Range: {self.MIN_DEPTH:.3f} m to {self.MAX_DEPTH:.3f} m")
        print(f"Tolerance: ±{self.DEPTH_TOLERANCE:.3f} m")
        print(f"PID Gains: Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}")
        print(f"Max Output: ±{self.MAX_HEAVE_OUTPUT:.2f}")
        print("="*80)
        print("Status: Waiting for sensor data...")
        print("Press Ctrl+C to stop\n")
    
    def depth_callback(self, msg: Float32):
        """Receive current depth from sensor"""
        self.current_depth = msg.data
        self.depth_samples += 1
        
        # Track min/max
        self.min_depth_achieved = min(self.min_depth_achieved, self.current_depth)
        self.max_depth_achieved = max(self.max_depth_achieved, self.current_depth)
    
    def control_loop(self):
        """Main PID depth control loop - runs at 20Hz"""
        
        # Check if we have sensor data
        if self.depth_samples == 0:
            return
        
        # Calculate error
        error = self.target_depth - self.current_depth
        
        # Track max error
        self.max_error = max(self.max_error, abs(error))
        
        # Time delta
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0:
            dt = 0.05
        
        self.last_time = current_time
        
        # PID calculations
        # Proportional term
        p_term = self.Kp * error
        
        # Integral term (with anti-windup)
        self.integral_error += error * dt
        max_integral = self.MAX_HEAVE_OUTPUT / self.Ki if self.Ki > 0 else 0
        self.integral_error = max(-max_integral, min(self.integral_error, max_integral))
        i_term = self.Ki * self.integral_error
        
        # Derivative term
        d_term = self.Kd * (error - self.previous_error) / dt if dt > 0 else 0
        self.previous_error = error
        
        # Combine PID terms
        output = p_term + i_term + d_term
        
        # Clamp output
        output = max(-self.MAX_HEAVE_OUTPUT, min(output, self.MAX_HEAVE_OUTPUT))
        
        # ============================================================
        # SAFETY CHECKS
        # ============================================================
        
        # Check depth bounds
        if self.current_depth > self.MAX_DEPTH:
            self.get_logger().error(
                f"❌ DEPTH EXCEEDED MAX: {self.current_depth:.3f}m > {self.MAX_DEPTH:.3f}m"
            )
            # Force upward movement
            output = -self.MAX_HEAVE_OUTPUT
        
        elif self.current_depth < self.MIN_DEPTH:
            self.get_logger().error(
                f"❌ DEPTH BELOW MIN: {self.current_depth:.3f}m < {self.MIN_DEPTH:.3f}m"
            )
            # Force downward movement
            output = self.MAX_HEAVE_OUTPUT
        
        # Check stability
        if abs(error) < self.DEPTH_TOLERANCE:
            self.stable_count += 1
            self.unstable_count = 0
        else:
            self.unstable_count += 1
            self.stable_count = max(0, self.stable_count - 1)
        
        # ============================================================
        # PUBLISH COMMAND
        # ============================================================
        cmd = Twist()
        cmd.linear.z = float(output)
        self.cmd_vel_pub.publish(cmd)
    
    def display_status(self):
        """Display status every 1 second"""
        
        if self.depth_samples == 0:
            return
        
        # Calculate status indicators
        error = self.target_depth - self.current_depth
        error_percent = abs(error) / self.target_depth * 100 if self.target_depth > 0 else 0
        
        # Status bar
        if abs(error) < self.DEPTH_TOLERANCE:
            status_icon = "✅"
            status_text = "STABLE"
        elif abs(error) < self.DEPTH_TOLERANCE * 2:
            status_icon = "🟡"
            status_text = "ADJUSTING"
        else:
            status_icon = "🔴"
            status_text = "CORRECTING"
        
        # Elapsed time
        elapsed = time.time() - self.start_time
        
        # Print status line
        print(
            f"{status_icon} {status_text:12} | "
            f"Target: {self.target_depth:.3f}m | "
            f"Current: {self.current_depth:.3f}m | "
            f"Error: {error:+.4f}m ({error_percent:+.1f}%) | "
            f"Stable: {self.stable_count}s | "
            f"Time: {elapsed:.0f}s"
        )
    
    def print_summary(self):
        """Print summary when shutting down"""
        elapsed = time.time() - self.start_time
        
        print("\n" + "="*80)
        print("📊 MISSION SUMMARY")
        print("="*80)
        print(f"Target Depth: {self.target_depth:.3f} m")
        print(f"Final Depth: {self.current_depth:.3f} m")
        print(f"Max Error: {self.max_error:.4f} m")
        print(f"Min Depth: {self.min_depth_achieved:.3f} m")
        print(f"Max Depth: {self.max_depth_achieved:.3f} m")
        print(f"Samples Received: {self.depth_samples}")
        print(f"Elapsed Time: {elapsed:.1f} seconds")
        print(f"Stable Time: {self.stable_count} seconds")
        print("="*80)
        print("✅ Depth maintenance stopped safely\n")
    
    def destroy_node(self):
        """Clean shutdown"""
        # Send zero command to thrusters
        stop_cmd = Twist()
        stop_cmd.linear.z = 0.0
        self.cmd_vel_pub.publish(stop_cmd)
        
        # Wait a moment for command to be sent
        time.sleep(0.1)
        
        # Print summary
        self.print_summary()
        
        super().destroy_node()


def main(args=None):
    """
    Main entry point
    
    Usage:
      python3 depth_maintainer.py 0.5      # Maintain 0.5m depth
      python3 depth_maintainer.py 1.0      # Maintain 1.0m depth
    """
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='ROS2 Depth Maintainer - Maintain underwater depth via PID control'
    )
    parser.add_argument(
        'depth',
        type=float,
        nargs='?',
        default=0.6,
        help='Target depth in meters (default: 0.6m)'
    )
    parser.add_argument(
        '--min',
        type=float,
        default=-0.2,
        help='Minimum safe depth (default: -0.2m)'
    )
    parser.add_argument(
        '--max',
        type=float,
        default=1.3,
        help='Maximum safe depth (default: 1.3m)'
    )
    parser.add_argument(
        '--tolerance',
        type=float,
        default=0.05,
        help='Depth tolerance for stability (default: 0.05m)'
    )
    parser.add_argument(
        '--kp',
        type=float,
        default=1.2,
        help='PID Proportional gain (default: 1.2)'
    )
    parser.add_argument(
        '--ki',
        type=float,
        default=0.08,
        help='PID Integral gain (default: 0.08)'
    )
    parser.add_argument(
        '--kd',
        type=float,
        default=0.4,
        help='PID Derivative gain (default: 0.4)'
    )
    
    # Parse args
    parsed_args = parser.parse_args(args[1:] if args else sys.argv[1:])
    target_depth = parsed_args.depth
    
    # Validate depth
    if target_depth < -0.5 or target_depth > 2.0:
        print(f"❌ ERROR: Invalid depth {target_depth}m")
        print("   Valid range: -0.5m to 2.0m")
        sys.exit(1)
    
    print(f"\n🎯 Starting depth maintenance at {target_depth:.3f}m\n")
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    try:
        # Create node
        node = DepthMaintainer(target_depth=target_depth)
        
        # Update parameters if provided
        node.MIN_DEPTH = parsed_args.min
        node.MAX_DEPTH = parsed_args.max
        node.DEPTH_TOLERANCE = parsed_args.tolerance
        node.Kp = parsed_args.kp
        node.Ki = parsed_args.ki
        node.Kd = parsed_args.kd
        
        # Spin
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n⏹️  Received stop signal...")
        
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        
    finally:
        # Cleanup
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
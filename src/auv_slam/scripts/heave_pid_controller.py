#!/usr/bin/env python3
"""
Heave PID Controller - Simulink C++ Implementation
Uses high-performance C++ PID generated from Simulink for depth control
Maintains target depth with superior stability and response
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Twist
import time

try:
    from auv_slam.heave_pid_rt_py import HeavePID
    CPP_PID_AVAILABLE = True
except ImportError:
    CPP_PID_AVAILABLE = False
    import warnings
    warnings.warn("C++ PID module not available, using Python fallback")


class PythonPIDFallback:
    """Fallback Python PID if C++ module unavailable"""
    def __init__(self, kp=1.0, ki=0.0, kd=0.2, output_limits=(-1.0, 1.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        
    def compute(self, setpoint, measured_value):
        current_time = time.time()
        error = setpoint - measured_value
        
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error = error
            return 0.0
        
        dt = current_time - self.prev_time
        if dt <= 0.0:
            dt = 0.01
        
        p_term = self.kp * error
        self.integral += error * dt
        max_integral = abs(self.output_limits[1] - self.output_limits[0]) / 2.0
        self.integral = max(-max_integral, min(self.integral, max_integral))
        i_term = self.ki * self.integral
        d_term = self.kd * (error - self.prev_error) / dt
        
        output = p_term + i_term + d_term
        output = max(self.output_limits[0], min(output, self.output_limits[1]))
        
        self.prev_error = error
        self.prev_time = current_time
        return output


class HeavePIDController(Node):
    def __init__(self):
        super().__init__('heave_pid_controller')
        
        # Declare parameters
        self.declare_parameter('target_depth', 0.6)  # meters
        self.declare_parameter('depth_tolerance', 0.05)  # meters
        self.declare_parameter('kp', 1.2)
        self.declare_parameter('ki', 0.05)
        self.declare_parameter('kd', 0.3)
        self.declare_parameter('max_output', 0.6)  # Max vertical velocity
        self.declare_parameter('control_rate', 20.0)  # Hz
        self.declare_parameter('enable_on_start', True)
        self.declare_parameter('use_cpp_pid', True)  # Use C++ PID by default
        
        # Get parameters
        self.target_depth = self.get_parameter('target_depth').value
        self.depth_tolerance = self.get_parameter('depth_tolerance').value
        self.max_output = self.get_parameter('max_output').value
        control_rate = self.get_parameter('control_rate').value
        use_cpp = self.get_parameter('use_cpp_pid').value
        
        # PID gains
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        
        # Initialize PID controller
        self.use_cpp_pid = use_cpp and CPP_PID_AVAILABLE
        
        if self.use_cpp_pid:
            try:
                self.cpp_pid = HeavePID()
                self.cpp_pid.initialize()
                self.pid_type = "C++ Simulink"
                self.get_logger().info('✅ Using C++ Simulink PID Controller')
                
                # The Simulink PID has built-in gains, but we track for display
                self.kp = kp
                self.ki = ki
                self.kd = kd
                
            except Exception as e:
                self.get_logger().error(f'Failed to initialize C++ PID: {e}')
                self.get_logger().warn('Falling back to Python PID')
                self.use_cpp_pid = False
        
        if not self.use_cpp_pid:
            self.python_pid = PythonPIDFallback(
                kp=kp,
                ki=ki,
                kd=kd,
                output_limits=(-self.max_output, self.max_output)
            )
            self.pid_type = "Python Fallback"
            self.kp = kp
            self.ki = ki
            self.kd = kd
            self.get_logger().warn('⚠️ Using Python Fallback PID')
        
        # State variables
        self.current_depth = 0.0
        self.depth_received = False
        self.enabled = self.get_parameter('enable_on_start').value
        self.stabilized = False
        self.stabilization_count = 0
        self.stable_count_threshold = 20  # ~1 second at 20Hz
        
        # Performance tracking
        self.last_error = 0.0
        self.error_integral = 0.0
        self.max_error_seen = 0.0
        self.control_cycles = 0
        
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
        
        # Statistics timer (every 5 seconds)
        self.stats_timer = self.create_timer(5.0, self.print_statistics)
        
        self.get_logger().info('='*70)
        self.get_logger().info('🎚️ Heave PID Controller Initialized')
        self.get_logger().info('='*70)
        self.get_logger().info(f'  PID Type: {self.pid_type}')
        self.get_logger().info(f'  Target Depth: {self.target_depth:.3f}m')
        self.get_logger().info(f'  Tolerance: ±{self.depth_tolerance:.3f}m')
        self.get_logger().info(f'  PID Gains: Kp={self.kp:.2f}, Ki={self.ki:.3f}, Kd={self.kd:.2f}')
        self.get_logger().info(f'  Max Output: ±{self.max_output:.2f}')
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
            if self.use_cpp_pid:
                try:
                    self.cpp_pid = HeavePID()
                    self.cpp_pid.initialize()
                except Exception as e:
                    self.get_logger().error(f'Failed to reinitialize C++ PID: {e}')
            else:
                self.python_pid.reset()
            
            self.stabilized = False
            self.stabilization_count = 0
            self.error_integral = 0.0
            self.max_error_seen = 0.0
            self.control_cycles = 0
            self.get_logger().info('✅ Heave PID Controller ENABLED (Reset)')
            
        elif not self.enabled and was_enabled:
            self.get_logger().info('⏸️ Heave PID Controller DISABLED')
    
    def setpoint_callback(self, msg: Float32):
        """Update target depth setpoint"""
        old_target = self.target_depth
        self.target_depth = msg.data
        self.stabilized = False
        self.stabilization_count = 0
        
        # Soft reset - keep some history
        if self.use_cpp_pid:
            try:
                # Reinitialize for new setpoint
                self.cpp_pid = HeavePID()
                self.cpp_pid.initialize()
            except Exception as e:
                self.get_logger().error(f'Failed to reinitialize for new setpoint: {e}')
        else:
            # For Python PID, just reset integral
            self.python_pid.integral *= 0.5
        
        self.get_logger().info(
            f'🎯 Target depth changed: {old_target:.3f}m → {self.target_depth:.3f}m'
        )
    
    def control_loop(self):
        """Main PID control loop - runs at control_rate Hz"""
        
        # Check if we have depth data
        if not self.depth_received:
            return
        
        self.control_cycles += 1
        
        # Calculate error
        error = self.target_depth - self.current_depth
        self.last_error = error
        self.error_integral += abs(error)
        self.max_error_seen = max(self.max_error_seen, abs(error))
        
        # Publish error
        error_msg = Float32()
        error_msg.data = float(error)
        self.error_pub.publish(error_msg)
        
        # Initialize command
        cmd = Twist()
        output = 0.0
        
        if not self.enabled:
            # Controller disabled - send zero command
            cmd.linear.z = 0.0
            status = "DISABLED"
            
        else:
            # Controller enabled - compute PID output
            if self.use_cpp_pid:
                try:
                    # Set inputs for Simulink PID
                    # depth_measured is current, depth_goal is target
                    self.cpp_pid.set_inputs(
                        depth_measured=self.current_depth,
                        depth_goal=self.target_depth
                    )
                    
                    # Execute one control step
                    self.cpp_pid.step()
                    
                    # Get outputs (FL, FR, RL, RR thrusters)
                    # For heave, all thrusters should have same value
                    outputs = self.cpp_pid.get_outputs()
                    
                    # Average all thruster outputs for pure heave
                    raw_output = (outputs[0] + outputs[1] + outputs[2] + outputs[3]) / 4.0
                    
                    # Scale and limit output
                    # Simulink outputs are in range -100 to +100
                    # Scale to our max_output range
                    output = (raw_output / 100.0) * self.max_output
                    output = max(-self.max_output, min(output, self.max_output))
                    
                except Exception as e:
                    self.get_logger().error(f'C++ PID error: {e}')
                    output = 0.0
                    
            else:
                # Python PID fallback
                output = self.python_pid.compute(self.target_depth, self.current_depth)
            
            # Apply output to heave command
            # Positive error (target > current) means we need to go DOWN (positive heave)
            # The PID output is already in the correct direction
            cmd.linear.z = output
            
            # Check if stabilized
            if abs(error) < self.depth_tolerance:
                self.stabilization_count += 1
                if self.stabilization_count >= self.stable_count_threshold:
                    self.stabilized = True
            else:
                self.stabilization_count = max(0, self.stabilization_count - 1)
                self.stabilized = False
            
            status = "STABLE" if self.stabilized else "ACTIVE"
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
        
        # Publish output value
        output_msg = Float32()
        output_msg.data = float(output)
        self.output_pub.publish(output_msg)
        
        # Publish status
        status_msg = String()
        status_msg.data = (
            f"{status} | Err: {error:+.3f}m | Out: {output:+.3f} | "
            f"Stbl: {self.stabilization_count}/{self.stable_count_threshold}"
        )
        self.status_pub.publish(status_msg)
        
        # Periodic logging (every 2 seconds at 20Hz = every 40 cycles)
        if self.control_cycles % 40 == 0:
            self.get_logger().info(
                f'📊 {status} | Depth: {self.current_depth:.3f}m → {self.target_depth:.3f}m | '
                f'Error: {error:+.3f}m | Output: {output:+.3f} | '
                f'Type: {self.pid_type}',
                throttle_duration_sec=1.9
            )
    
    def print_statistics(self):
        """Print performance statistics every 5 seconds"""
        if not self.enabled or self.control_cycles == 0:
            return
        
        avg_error = self.error_integral / self.control_cycles
        
        self.get_logger().info('='*70)
        self.get_logger().info('📈 PID Performance Statistics')
        self.get_logger().info(f'   PID Type: {self.pid_type}')
        self.get_logger().info(f'   Control Cycles: {self.control_cycles}')
        self.get_logger().info(f'   Current Error: {self.last_error:+.4f}m')
        self.get_logger().info(f'   Average Error: {avg_error:.4f}m')
        self.get_logger().info(f'   Max Error: {self.max_error_seen:.4f}m')
        self.get_logger().info(f'   Stability: {"✅ STABLE" if self.stabilized else "⚠️ ADJUSTING"}')
        self.get_logger().info('='*70)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = HeavePIDController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Fatal error: {e}')
    finally:
        # Send zero command on exit
        try:
            if 'node' in locals():
                stop_cmd = Twist()
                node.cmd_vel_pub.publish(stop_cmd)
                node.get_logger().info('Sent stop command on shutdown')
        except:
            pass
        
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
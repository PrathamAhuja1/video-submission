#!/usr/bin/env python3
"""
Thruster Controller Node
Maps velocity commands (Twist) to individual thruster commands
Based on BlueROV2 thruster configuration
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math


class ThrusterControllerNode(Node):
    def __init__(self):
        super().__init__('thruster_controller')
        
        # Parameters
        self.declare_parameter('max_thrust', 10.0)
        self.max_thrust = self.get_parameter('max_thrust').value
        
        # Subscribe to velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publishers for each thruster
        self.thruster_pubs = []
        for i in range(1, 7):
            pub = self.create_publisher(
                Float64,
                f'/orca4_ign/thruster{i}/cmd',
                10
            )
            self.thruster_pubs.append(pub)
        
        self.get_logger().info('Thruster Controller Node Started')
        self.get_logger().info(f'Max thrust: {self.max_thrust}')
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Convert Twist message to thruster commands
        
        BlueROV2 Configuration:
              FRONT
           T1      T2
             \\  //
              \\/
              /\\
             //  \\
           T3      T4
              BACK
        
        T5 (front vertical), T6 (back vertical)
        
        Twist message:
        - linear.x: forward/backward (surge)
        - linear.y: left/right (sway)
        - linear.z: up/down (heave)
        - angular.z: rotation (yaw)
        """
        
        surge = msg.linear.x
        sway = msg.linear.y
        heave = msg.linear.z
        yaw = msg.angular.z
        
        # Invert surge for correct direction
        surge = -surge
        
        # 45-degree thruster angle factor
        cos45 = 0.7071
        
        # Calculate horizontal thruster forces
        # T1 (front-left): contributes to surge and sway
        t1 = cos45 * (surge + sway) - yaw * 2.0
        
        # T2 (front-right): contributes to surge and sway
        t2 = cos45 * (surge - sway) + yaw * 2.0
        
        # T3 (back-left): contributes to surge and sway
        t3 = cos45 * (-surge + sway) + yaw * 2.0
        
        # T4 (back-right): contributes to surge and sway
        t4 = cos45 * (-surge - sway) - yaw * 2.0
        
        # Vertical thrusters (point downward when positive)
        t5 = -heave * 2.5  # Front vertical
        t6 = -heave * 2.5  # Back vertical
        
        # Compile thrust values
        thrusts = [t1, t2, t3, t4, t5, t6]
        
        # Apply limits and publish
        for i, thrust in enumerate(thrusts):
            # Clamp to max thrust
            thrust = max(-self.max_thrust, min(thrust, self.max_thrust))
            
            # Create and publish message
            thrust_msg = Float64()
            thrust_msg.data = float(thrust)
            self.thruster_pubs[i].publish(thrust_msg)
        
        # Log occasionally
        if abs(surge) > 0.01 or abs(heave) > 0.01 or abs(yaw) > 0.01:
            self.get_logger().info(
                f'Cmd: surge={surge:.2f}, heave={heave:.2f}, yaw={yaw:.2f}',
                throttle_duration_sec=1.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
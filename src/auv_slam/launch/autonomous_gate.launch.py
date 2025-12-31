#!/usr/bin/env python3
"""
Autonomous Gate Detection Mission Launch with Heave PID Control
Includes automatic depth control via PID
FIXED: Removed duplicate depth_sensor_node (serial_bridge handles depth)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    
    camera_params_file = '/home/radxa-x4/Documents/usb_cam/config/params_1.yaml'
    
    return LaunchDescription([
        
        # ============================================================
        # 1. USB CAMERA NODE
        # ============================================================
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[camera_params_file],
            remappings=[
                ('/image_raw', '/camera_forward/image_raw'),
                ('/camera_info', '/camera_forward/camera_info'),
            ]
        ),
        
        # ============================================================
        # 2. VN100 IMU NODE (moved from #3)
        # ============================================================
        Node(
            package='vn100_reader',
            executable='vn100_ypr_node',
            name='vn100_imu',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 115200,
                'publish_rate': 50.0
            }]
        ),
        
        # ============================================================
        # 3. SERIAL BRIDGE - Delay 1s (handles PWM + depth telemetry)
        # ============================================================
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='auv_slam',
                    executable='serial_bridge.py',
                    name='serial_bridge',
                    output='screen',
                    parameters=[{
                        'serial_port': '/dev/ttyS4',
                        'baud_rate': 115200,
                        'depth_offset': 0.4  # Calibration offset
                    }]
                )
            ]
        ),
        
        # ============================================================
        # 4. HEAVE PID CONTROLLER - Delay 1s
        # ============================================================
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='auv_slam',
                    executable='heave_pid_controller.py',
                    name='heave_pid_controller',
                    output='screen',
                    parameters=[{
                        'target_depth': 1.0,        # 1m target depth
                        'depth_tolerance': 0.05,    # 5cm tolerance
                        'kp': 1.2,                  # Proportional gain
                        'ki': 0.05,                 # Integral gain
                        'kd': 0.3,                  # Derivative gain
                        'max_output': 0.6,          # Max vertical velocity
                        'control_rate': 20.0,       # 20Hz control
                        'enable_on_start': True     # Auto-enable
                    }]
                )
            ]
        ),
        
        # ============================================================
        # 5. COMMAND MIXER - Delay 1s
        # ============================================================
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='auv_slam',
                    executable='cmd_mixer.py',
                    name='command_mixer',
                    output='screen',
                    parameters=[{
                        'command_timeout': 1.0
                    }]
                )
            ]
        ),
        
        # ============================================================
        # 6. HARDWARE PWM MAPPER - Delay 2s
        # ============================================================
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='auv_slam',
                    executable='pwm_mapper.py',
                    name='hardware_pwm_mapper',
                    output='screen',
                    remappings=[
                        ('/cmd_vel', '/cmd_vel_combined')  # Use combined commands
                    ]
                )
            ]
        ),
        
        # ============================================================
        # 7. SAFETY MONITOR - Delay 2s
        # ============================================================
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='auv_slam',
                    executable='safety_monitor.py',
                    name='safety_monitor',
                    output='screen',
                    parameters=[{
                        'max_depth': 1.2,
                        'min_depth': -0.1,  # Allow surface operation for testing
                        'pool_bounds_x': [-3.0, 3.0],
                        'pool_bounds_y': [-3.0, 3.0],
                    }]
                )
            ]
        ),
        
        # ============================================================
        # 8. GATE DETECTOR - Delay 3s
        # ============================================================
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='auv_slam',
                    executable='gate_detector.py',
                    name='gate_detector',
                    output='screen'
                )
            ]
        ),
        
        # ============================================================
        # 9. GATE NAVIGATOR - Delay 3s
        # ============================================================
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='auv_slam',
                    executable='gate_navigator.py',
                    name='gate_navigator',
                    output='screen'
                )
            ]
        ),
    ])


if __name__ == '__main__':
    generate_launch_description()
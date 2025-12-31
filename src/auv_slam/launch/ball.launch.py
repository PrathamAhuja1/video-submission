#!/usr/bin/env python3
"""
Autonomous Ball Following Mission Launch - FIXED
CRITICAL FIX: Removed depth_sensor_node conflict
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
        # 2. VN100 IMU NODE
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
        # 3. SERIAL BRIDGE - Handles PWM + Depth Sensor
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
                        'depth_offset': 0.4
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
                        'target_depth': 1.0,
                        'depth_tolerance': 0.05,
                        'kp': 1.2,
                        'ki': 0.05,
                        'kd': 0.3,
                        'max_output': 0.6,
                        'control_rate': 20.0,
                        'enable_on_start': True
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
                        ('/cmd_vel', '/cmd_vel_combined')
                    ]
                )
            ]
        ),
        
        # ============================================================
        # 7. BALL DETECTOR - Delay 3s
        # ============================================================
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='auv_slam',
                    executable='ball_detector.py',
                    name='ball_detector',
                    output='screen'
                )
            ]
        ),
        
        # ============================================================
        # 8. BALL FOLLOWER - Delay 3s
        # ============================================================
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='auv_slam',
                    executable='ball_follower.py',
                    name='ball_follower',
                    output='screen'
                )
            ]
        ),
        
        # ============================================================
        # 9. SAFETY MONITOR - Delay 2s
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
                        'min_depth': 0.1,
                        'max_roll': 30.0,
                        'max_pitch': 30.0,
                        'pool_bounds_x': [-3.0, 3.0],
                        'pool_bounds_y': [-3.0, 3.0],
                    }]
                )
            ]
        ),
    ])


if __name__ == '__main__':
    generate_launch_description()
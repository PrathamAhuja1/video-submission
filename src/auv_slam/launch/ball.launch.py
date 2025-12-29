#!/usr/bin/env python3
"""
Autonomous Ball Following Mission Launch
Launches all necessary nodes for ball tracking and following
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
        # 2. DEPTH SENSOR NODE
        # ============================================================
        Node(
            package='depth_sensor_pkg',
            executable='depth_sensor_node',
            name='depth_sensor',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyS4',
                'baud_rate': 115200,
                'publish_rate': 20.0
            }]
        ),
        
        # ============================================================
        # 3. VN100 IMU NODE
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
        # 4. HARDWARE PWM MAPPER - Delay 2s
        # ============================================================
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='auv_slam',
                    executable='pwm_mapper.py',
                    name='hardware_pwm_mapper',
                    output='screen'
                )
            ]
        ),
        
        # ============================================================
        # 5. BALL DETECTOR - Delay 3s for camera stabilization
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
        # 6. BALL FOLLOWER - Delay 3s
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
                        'min_depth': 0.1,
                        'pool_bounds_x': [-3.0, 3.0],
                        'pool_bounds_y': [-3.0, 3.0],
                    }]
                )
            ]
        ),

        # ============================================================
        # 8. SERIAL BRIDGE - Delay 1s
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
                        'baud_rate': 115200
                    }]
                )
            ]
        ),
    ])


if __name__ == '__main__':
    generate_launch_description()
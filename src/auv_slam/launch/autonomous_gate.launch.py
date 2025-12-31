#!/usr/bin/env python3
"""
Autonomous Gate Detection Mission Launch - FIXED
CRITICAL FIX: Removed depth_sensor_node to prevent serial port conflict
Serial Bridge now handles both PWM and depth sensor
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    
    camera_params_file = '/home/radxa-x4/Documents/usb_cam/config/params_1.yaml'
    
    return LaunchDescription([
        
        # ============================================================
        # 1. USB CAMERA NODE - Launch immediately
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
        # 2. VN100 IMU NODE - Launch immediately
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
        # CRITICAL: This is the ONLY node using /dev/ttyS4
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
        # 4. HARDWARE PWM MAPPER - Delay 2s for sensor init
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
        # 5. GATE DETECTOR - Delay 3s for camera stabilization
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
        # 6. GATE NAVIGATOR - Delay 3s
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
                        'min_depth': 0.05,
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
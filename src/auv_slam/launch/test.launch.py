#!/usr/bin/env python3
"""
Test Sequence Launch File
Launches complete test setup:
- USB Camera with live feed
- Serial Bridge (communicates with RP2040)
- PWM Mapper (converts Twist to PWM)
- Test Sequence Node (executes movements)
- rqt_image_view (displays camera feed)

Test Sequence:
1. Heave DOWN for 4 seconds
2. Heave UP for 3 seconds  
3. Surge FORWARD for 3 seconds
4. Yaw LEFT for 3 seconds
5. Yaw RIGHT for 3 seconds

Usage:
  ros2 launch auv_slam test_sequence.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os


def generate_launch_description():
    
    # Camera parameters file path
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
                ('/image_raw', '/image_raw'),
                ('/camera_info', '/camera_info'),
            ]
        ),
        
        # ============================================================
        # 2. SERIAL BRIDGE - Launch after 1 second
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
        # 3. PWM MAPPER - Launch after 2 seconds
        # ============================================================
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='auv_slam',
                    executable='pwm_mapper.py',
                    name='pwm_mapper',
                    output='screen'
                )
            ]
        ),
        
        # ============================================================
        # 4. TEST SEQUENCE NODE - Launch after 3 seconds
        # ============================================================
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='auv_slam',
                    executable='test.py',
                    name='test_sequence',
                    output='screen',
                    parameters=[{
                        'heave_speed': 0.9,   # 1680 PWM
                        'surge_speed': 0.95,  # 1690 PWM  
                        'yaw_speed': 0.8,     # ~1620 PWM
                        'heave_down_duration': 4.0,  # Heave down for 4 seconds
                        'heave_up_duration': 2.5,    # Heave up for 3 seconds
                        'test_duration': 3.0,        # Other tests for 3 seconds each
                        'pause_duration': 1.0
                    }]
                )
            ]
        ),
        
        # ============================================================
        # 5. RQT IMAGE VIEW - Launch after 2 seconds to view camera
        # ============================================================
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=['rqt_image_view', '/image_raw'],
                    output='screen',
                    shell=False
                )
            ]
        ),
        
        # ============================================================
        # 6. VN100 IMU
        # ============================================================
        # TimerAction(
        #     period=1.0,
        #     actions=[
        #         Node(
        #             package='vn100_reader',
        #             executable='vn100_ypr_node',
        #             name='vn100_imu',
        #             output='screen',
        #             parameters=[{
        #                 'port': '/dev/ttyUSB1',
        #                 'baudrate': 115200,
        #                 'publish_rate': 50.0
        #             }]
        #         )
        #     ]
        # ),
    ])


if __name__ == '__main__':
    generate_launch_description()
#!/usr/bin/env python3
"""
Gate Alignment Test Launch File
Launches complete gate alignment system without depth sensor

Components:
- USB Camera (image capture)
- Gate Detector (detects gate, publishes center)
- Serial Bridge (RP2040 communication)
- PWM Mapper (Twist → PWM conversion)
- Gate Alignment Test Node (main control)
- rqt_image_view (debug visualization)

Usage:
  ros2 launch auv_slam gate_alignment_test.launch.py
  
With custom parameters:
  ros2 launch auv_slam gate_alignment_test.launch.py \
      surge_duration:=8.0 \
      surge_speed:=0.9 \
      vertical_tolerance:=0.06
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    # Camera parameters file path
    camera_params_file = '/home/radxa-x4/Documents/usb_cam/config/params_1.yaml'
    
    # ========================================================================
    # LAUNCH ARGUMENTS - Easily tune parameters from command line
    # ========================================================================
    
    vertical_tolerance_arg = DeclareLaunchArgument(
        'vertical_tolerance',
        default_value='0.08',
        description='Vertical alignment tolerance (normalized 0-1). Lower = more precise.'
    )
    
    horizontal_tolerance_arg = DeclareLaunchArgument(
        'horizontal_tolerance',
        default_value='0.08',
        description='Horizontal alignment tolerance (normalized 0-1). Lower = more precise.'
    )
    
    surge_duration_arg = DeclareLaunchArgument(
        'surge_duration',
        default_value='5.0',
        description='Duration to surge through gate (seconds)'
    )
    
    surge_speed_arg = DeclareLaunchArgument(
        'surge_speed',
        default_value='0.85',
        description='Surge speed through gate (0.0-1.0 scale)'
    )
    
    alignment_timeout_arg = DeclareLaunchArgument(
        'alignment_timeout',
        default_value='30.0',
        description='Maximum time allowed for alignment (seconds)'
    )
    
    heave_gain_arg = DeclareLaunchArgument(
        'heave_gain',
        default_value='0.5',
        description='Heave control gain (vertical alignment responsiveness)'
    )
    
    yaw_gain_arg = DeclareLaunchArgument(
        'yaw_gain',
        default_value='2.5',
        description='Yaw control gain (horizontal alignment responsiveness)'
    )
    
    max_heave_speed_arg = DeclareLaunchArgument(
        'max_heave_speed',
        default_value='0.4',
        description='Maximum heave speed (0.0-1.0)'
    )
    
    max_yaw_speed_arg = DeclareLaunchArgument(
        'max_yaw_speed',
        default_value='0.6',
        description='Maximum yaw speed (0.0-1.0)'
    )
    
    show_debug_image_arg = DeclareLaunchArgument(
        'show_debug_image',
        default_value='true',
        description='Show debug image with gate bounding box (true/false)'
    )
    
    return LaunchDescription([
        # ====================================================================
        # LAUNCH ARGUMENTS
        # ====================================================================
        vertical_tolerance_arg,
        horizontal_tolerance_arg,
        surge_duration_arg,
        surge_speed_arg,
        alignment_timeout_arg,
        heave_gain_arg,
        yaw_gain_arg,
        max_heave_speed_arg,
        max_yaw_speed_arg,
        show_debug_image_arg,
        
        # ====================================================================
        # 1. USB CAMERA NODE - Launch immediately
        # ====================================================================
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
        
        # ====================================================================
        # 2. SERIAL BRIDGE - Launch after 1 second
        # ====================================================================
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
                        'depth_offset': 0.4  # Not used in this test, but kept for compatibility
                    }]
                )
            ]
        ),
        
        # ====================================================================
        # 3. PWM MAPPER - Launch after 2 seconds
        # ====================================================================
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
        
        # ====================================================================
        # 4. GATE DETECTOR - Launch after 3 seconds
        # ====================================================================
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
        
        # ====================================================================
        # 5. GATE ALIGNMENT TEST NODE - Launch after 4 seconds
        # ====================================================================
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='auv_slam',
                    executable='test.py',
                    name='gate_alignment_test',
                    output='screen',
                    parameters=[{
                        'vertical_tolerance': LaunchConfiguration('vertical_tolerance'),
                        'horizontal_tolerance': LaunchConfiguration('horizontal_tolerance'),
                        'surge_duration': LaunchConfiguration('surge_duration'),
                        'surge_speed': LaunchConfiguration('surge_speed'),
                        'alignment_timeout': LaunchConfiguration('alignment_timeout'),
                        'heave_gain': LaunchConfiguration('heave_gain'),
                        'yaw_gain': LaunchConfiguration('yaw_gain'),
                        'max_heave_speed': LaunchConfiguration('max_heave_speed'),
                        'max_yaw_speed': LaunchConfiguration('max_yaw_speed')
                    }]
                )
            ]
        ),
        
        # ====================================================================
        # 6. RQT IMAGE VIEW - Launch after 3 seconds (conditional)
        # ====================================================================
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 
                         'if [ "$(ros2 param get /gate_alignment_test show_debug_image 2>/dev/null || echo true)" = "true" ]; then '
                         'rqt_image_view /gate/debug_image; fi'],
                    output='screen',
                    shell=False
                )
            ]
        ),
        
        # ====================================================================
        # OPTIONAL NODES (Uncomment if needed)
        # ====================================================================
        
        # VN100 IMU - Uncomment if using IMU
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
        
        # Safety Monitor - Uncomment for safety checks
        # TimerAction(
        #     period=2.0,
        #     actions=[
        #         Node(
        #             package='auv_slam',
        #             executable='safety_monitor.py',
        #             name='safety_monitor',
        #             output='screen',
        #             parameters=[{
        #                 'max_depth': 1.2,
        #                 'min_depth': -5.0,
        #                 'max_roll': 30.0,
        #                 'max_pitch': 30.0,
        #                 'max_mission_time': 300.0,
        #                 'sensor_timeout': 3.0
        #             }]
        #         )
        #     ]
        # ),
    ])


if __name__ == '__main__':
    generate_launch_description()
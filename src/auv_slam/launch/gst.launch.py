#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    

    DeclareLaunchArgument(
        'image_topic',
        default_value='/image_raw',
        description='ROS image topic for GStreamer bridge'
    ),
    DeclareLaunchArgument(
        'bitrate',
        default_value='2000',
        description='H264 encoder bitrate'
    ),

    
    return LaunchDescription([

        # 1. USB CAMERA NODE - Launch immediately
        Node(
            package='gscam2',
            executable='gscam_main',
            name='gscam2',
            output='screen',
            parameters=[
                '/home/radxa-x4/Documents/gscam_ws/config/gscam_params.yaml',
                {
                    'camera_info_url': '/home/radxa-x4/Documents/gscam_ws/config/camera_info.yaml'
                }
            ],
            remappings=[
                ('/image_raw', '/image_raw')
            ]
        ),

        TimerAction(
            period=1.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        '/bin/bash',
                        '/home/radxa-x4/Documents/gscam_ws/config/init_ros_gst_bridge.sh',
                        LaunchConfiguration('image_topic'),
                        LaunchConfiguration('bitrate')
                    ],
                    output='screen'
                )
            ]
        ),
    ])


if __name__ == '__main__':
    generate_launch_description()
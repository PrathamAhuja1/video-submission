#!/usr/bin/env python3
"""
GScam2 Launch File with UDP Streaming
Publishes camera to ROS topic AND streams over UDP to ground station
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # =============================================================================
    # LAUNCH ARGUMENTS
    # =============================================================================
    
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='USB camera device path'
    )
    
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='640',
        description='Image width'
    )
    
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='480',
        description='Image height'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='Frames per second'
    )
    
    udp_host_arg = DeclareLaunchArgument(
        'udp_host',
        default_value='192.168.11.4',
        description='UDP target IP address (ground station)'
    )
    
    udp_port_arg = DeclareLaunchArgument(
        'udp_port',
        default_value='5600',
        description='UDP port number'
    )
    
    # =============================================================================
    # GSTREAMER PIPELINE CONFIGURATION
    # =============================================================================
    # Pipeline structure:
    # 1. v4l2src: Capture from USB camera
    # 2. jpegdec: Decode MJPEG (if camera outputs JPEG)
    # 3. videoconvert: Convert to RGB for ROS
    # 4. tee: Split stream into two branches
    #    - Branch A: appsink → gscam2 → ROS topic
    #    - Branch B: x264enc → rtph264pay → udpsink → Network
    # =============================================================================
    
    # Alternative pipeline for cameras that output raw video (not MJPEG)
    # Use this if your camera doesn't support JPEG:
    # gscam_config_raw = (
    #     "v4l2src device={device} ! "
    #     "video/x-raw,width={width},height={height},framerate={fps}/1 ! "
    #     ...rest of pipeline...
    # )
    
    gscam_config = (
        "v4l2src device=$(arg camera_device) ! "
        "image/jpeg,width=$(arg width),height=$(arg height),framerate=$(arg fps)/1 ! "
        "jpegdec ! "
        "videoconvert ! "
        "video/x-raw,format=RGB ! "
        "tee name=t ! "
        "queue max-size-buffers=2 leaky=downstream ! "
        "appsink "
        "t. ! "
        "queue max-size-buffers=2 leaky=downstream ! "
        "videoconvert ! "
        "x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast key-int-max=30 ! "
        "rtph264pay config-interval=1 pt=96 ! "
        "udpsink host=$(arg udp_host) port=$(arg udp_port) sync=false"
    )
    
    # Substitute launch arguments into pipeline
    # Note: This uses Python string formatting, so we need to handle it differently
    # We'll use SetEnvironmentVariable instead
    
    return LaunchDescription([
        # Declare arguments
        camera_device_arg,
        width_arg,
        height_arg,
        fps_arg,
        udp_host_arg,
        udp_port_arg,
        
        # Set GSCAM_CONFIG environment variable
        # We need to build the pipeline string with actual values
        SetEnvironmentVariable(
            name='GSCAM_CONFIG',
            value=[
                'v4l2src device=', LaunchConfiguration('camera_device'), ' ! ',
                'image/jpeg,width=', LaunchConfiguration('width'),
                ',height=', LaunchConfiguration('height'),
                ',framerate=', LaunchConfiguration('fps'), '/1 ! ',
                'jpegdec ! ',
                'videoconvert ! ',
                'video/x-raw,format=RGB ! ',
                'tee name=t ! ',
                'queue max-size-buffers=2 leaky=downstream ! ',
                'appsink ',
                't. ! ',
                'queue max-size-buffers=2 leaky=downstream ! ',
                'videoconvert ! ',
                'x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast key-int-max=30 ! ',
                'rtph264pay config-interval=1 pt=96 ! ',
                'udpsink host=', LaunchConfiguration('udp_host'),
                ' port=', LaunchConfiguration('udp_port'),
                ' sync=false'
            ]
        ),
        
        # GScam2 node
        Node(
            package='gscam2',
            executable='gscam_main',
            name='gscam_publisher',
            output='screen',
            parameters=[{
                'camera_name': 'usb_camera',
                'frame_id': 'camera_frame',
                'sync_sink': True,
                'use_gst_timestamps': False,
            }],
            remappings=[
                ('image_raw', '/image_raw'),
                ('camera_info', '/camera_info'),
            ],
        ),
    ])
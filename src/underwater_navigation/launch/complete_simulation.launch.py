#!/usr/bin/env python3
"""
Autonomous Underwater Navigation Launch File
Complete autonomous system with gate detection and navigation
Robot spawns at X=-2.0m and navigates through gate at X=-0.5m
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
import launch_ros.descriptions


def generate_launch_description():
    pkg_share = get_package_share_directory('underwater_navigation')
    
    # ========================================================================
    # PATHS
    # ========================================================================
    urdf_file = os.path.join(pkg_share, 'urdf', 'orca4_description.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'underwater_pool_updated.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge_config.yaml')
    
    # ========================================================================
    # GAZEBO ENVIRONMENT SETUP
    # ========================================================================
    gz_models_path = os.path.join(pkg_share, "models")
    gz_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", default="")
    gz_env = {
        'GZ_SIM_SYSTEM_PLUGIN_PATH':
           ':'.join([os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
                     os.environ.get('LD_LIBRARY_PATH', default='')]),
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':  
           ':'.join([os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                     os.environ.get('LD_LIBRARY_PATH', default='')]),
        'GZ_SIM_RESOURCE_PATH':
           ':'.join([gz_resource_path, gz_models_path])
    }
    
    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================
    
    # Starting position: -2.0m X, 0.0m Y, -0.5m Z (0.5m below surface)
    # Gate position: -0.5m X (1.5m ahead of robot)
    declare_spawn_x = DeclareLaunchArgument(
        'spawn_x',
        default_value='-2.0',
        description='X position for robot spawn (gate is at -0.5m)'
    )
    
    declare_spawn_y = DeclareLaunchArgument(
        'spawn_y',
        default_value='0.0',
        description='Y position for robot spawn (centered)'
    )
    
    declare_spawn_z = DeclareLaunchArgument(
        'spawn_z',
        default_value='-0.5',
        description='Z position for robot spawn (0.5m below water surface)'
    )
    
    declare_spawn_yaw = DeclareLaunchArgument(
        'spawn_yaw',
        default_value='0.0',
        description='Yaw orientation (0.0 = facing +X direction toward gate)'
    )
    
    # ========================================================================
    # CORE SIMULATION NODES
    # ========================================================================
    
    # 1. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': launch_ros.descriptions.ParameterValue(
                Command(['xacro ', urdf_file]), value_type=str
            ),
            'use_sim_time': True
        }]
    )
    
    # 2. Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # 3. Gazebo Simulator
    gazebo_process = ExecuteProcess(
        cmd=['ruby', FindExecutable(name="ign"), 'gazebo', '-r', '-v', '3', world_file],
        output='screen',
        additional_env=gz_env,
        shell=False
    )
    
    # 4. Spawn Robot Entity (Delayed 2 seconds to allow Gazebo to initialize)
    spawn_entity = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-name", "orca4_ign",
                    "-topic", "robot_description",
                    "-z", LaunchConfiguration('spawn_z'),
                    "-x", LaunchConfiguration('spawn_x'),
                    "-y", LaunchConfiguration('spawn_y'),
                    "-Y", LaunchConfiguration('spawn_yaw'),
                    "--ros-args", "--log-level", "warn"
                ],
                parameters=[{"use_sim_time": True}],
            )
        ]
    )
    
    # 5. ROS-Gazebo Bridge (Delayed 3 seconds)
    bridge = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    '--ros-args',
                    '-p', f'config_file:={bridge_config}'
                ],
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # ========================================================================
    # AUTONOMOUS NAVIGATION NODE
    # ========================================================================
    
    # 6. Autonomous Navigation (Delayed 5 seconds to allow sensors to initialize)
    autonomous_navigation = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='underwater_navigation',
                executable='autonomous_navigation_node.py',
                name='autonomous_navigation',
                output='screen',
                parameters=[{'use_sim_time': True}],
                emulate_tty=True
            )
        ]
    )
    
    # ========================================================================
    # OPTIONAL VISUALIZATION
    # ========================================================================
    
    # 7. Image Viewer for processed camera feed (Optional - Delayed 7 seconds)
    image_viewer = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'rqt_image_view', 'rqt_image_view',
                    '/autonomous/processed_image'
                ],
                output='screen'
            )
        ]
    )
    
    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================
    return LaunchDescription([
        # Launch arguments
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        declare_spawn_yaw,
        
        # Core simulation (immediate start)
        robot_state_publisher,
        joint_state_publisher,
        gazebo_process,
        
        # Timed launches
        spawn_entity,              # 2s delay
        bridge,                    # 3s delay
        autonomous_navigation,     # 5s delay
        image_viewer,              # 7s delay (optional)
    ])


if __name__ == '__main__':
    generate_launch_description()
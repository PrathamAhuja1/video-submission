#!/usr/bin/env python3
"""
Complete Underwater Navigation Simulation Launch File
Spawns robot at starting position with gate 1.5m ahead
FIXED VERSION - Compatible with modern Gazebo
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import launch_ros.descriptions


def generate_launch_description():
    pkg_share = get_package_share_directory('underwater_navigation')
    
    # ========================================================================
    # PATHS
    # ========================================================================
    urdf_file = os.path.join(pkg_share, 'urdf', 'orca4_description.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'underwater_pool.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge_config.yaml')
    
    # ========================================================================
    # GAZEBO ENVIRONMENT SETUP
    # ========================================================================
    gz_models_path = os.path.join(pkg_share, "models")
    gz_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    
    # Updated environment variables for both Ignition and Gazebo compatibility
    gz_env = {
        'GZ_SIM_SYSTEM_PLUGIN_PATH':
           ':'.join([os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
                     os.environ.get('LD_LIBRARY_PATH', '')]),
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':  
           ':'.join([os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ''),
                     os.environ.get('LD_LIBRARY_PATH', '')]),
        'GZ_SIM_RESOURCE_PATH':
           ':'.join([gz_resource_path, gz_models_path]),
        'IGN_GAZEBO_RESOURCE_PATH':
           ':'.join([gz_resource_path, gz_models_path])
    }
    
    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================
    declare_enable_rviz = DeclareLaunchArgument(
        'enable_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )
    
    declare_spawn_x = DeclareLaunchArgument(
        'spawn_x',
        default_value='-2.0',
        description='X position for robot spawn'
    )
    
    declare_spawn_y = DeclareLaunchArgument(
        'spawn_y',
        default_value='0.0',
        description='Y position for robot spawn'
    )
    
    declare_spawn_z = DeclareLaunchArgument(
        'spawn_z',
        default_value='-0.5',
        description='Z position for robot spawn (below water surface)'
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
    
    # 3. Gazebo Simulator - FIXED VERSION
    # Try 'gz sim' first (modern Gazebo), fallback to 'ign gazebo' (older Ignition)
    gazebo_process = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '3', world_file],
        output='screen',
        additional_env=gz_env,
        shell=False
    )
    
    # 4. Spawn Robot Entity (Delayed 3 seconds - increased for reliability)
    spawn_entity = TimerAction(
        period=3.0,
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
                    "-Y", "0.0",
                    "--ros-args", "--log-level", "warn"
                ],
                parameters=[{"use_sim_time": True}],
            )
        ]
    )
    
    # 5. ROS-Gazebo Bridge (Delayed 4 seconds)
    bridge = TimerAction(
        period=4.0,
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
    # NAVIGATION NODES
    # ========================================================================
    
    # 6. Camera Processor (Delayed 6 seconds)
    camera_processor = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='underwater_navigation',
                executable='camera_processor_node.py',
                name='camera_processor',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # 7. Gate Navigation (Delayed 6 seconds)
    gate_navigation = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='underwater_navigation',
                executable='gate_navigation_node.py',
                name='gate_navigation',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # 8. Thruster Controller (Delayed 6 seconds)
    thruster_controller = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='underwater_navigation',
                executable='thruster_controller_node.py',
                name='thruster_controller',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================
    return LaunchDescription([
        # Launch arguments
        declare_enable_rviz,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        
        # Core simulation (immediate start)
        robot_state_publisher,
        joint_state_publisher,
        gazebo_process,
        
        # Timed launches
        spawn_entity,           # 3s delay
        bridge,                 # 4s delay
        camera_processor,       # 6s delay
        gate_navigation,        # 6s delay
        thruster_controller,    # 6s delay
    ])


if __name__ == '__main__':
    generate_launch_description()
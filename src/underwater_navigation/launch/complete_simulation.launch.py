#!/usr/bin/env python3
"""
Complete Underwater Navigation Simulation Launch File
Spawns robot at starting position with gate 1.5m ahead
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
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
    declare_enable_rviz = DeclareLaunchArgument(
        'enable_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )
    
    # Starting position: -2.0m X, 0.0m Y, -0.5m Z (0.5m below surface)
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
                    "-Y", "0.0",  # Facing forward (+X direction)
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
    # NAVIGATION NODES
    # ========================================================================
    
    # 6. Camera Processor (Delayed 5 seconds)
    camera_processor = TimerAction(
        period=5.0,
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
    
    # 7. Gate Navigation (Delayed 5 seconds)
    gate_navigation = TimerAction(
        period=5.0,
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
    
    # 8. Thruster Controller (Delayed 5 seconds)
    thruster_controller = TimerAction(
        period=5.0,
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
    # OPTIONAL VISUALIZATION
    # ========================================================================
    
    # 9. Image Viewer (Optional - shows camera feed)
    image_viewer = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view'],
                output='screen'
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
        spawn_entity,           # 2s delay
        bridge,                 # 3s delay
        camera_processor,       # 5s delay
        gate_navigation,        # 5s delay
        thruster_controller,    # 5s delay
        image_viewer,           # 7s delay
    ])


if __name__ == '__main__':
    generate_launch_description()
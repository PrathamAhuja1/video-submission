#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- 1. Variable & Path Setup ---
    pkg_share = FindPackageShare('underwater_navigation')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    underwater_nav_path = get_package_share_directory('underwater_navigation')
    
    # Paths to specific files
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'underwater_pool.sdf'])
    bridge_config = PathJoinSubstitution([pkg_share, 'config', 'bridge_config.yaml'])
    robot_file = PathJoinSubstitution([pkg_share, 'models', 'orca4_ign', 'model.sdf'])
    
    # --- 2. Environment Configuration ---
    # Set the Gazebo resource path so it can find your models
    model_path = os.path.join(underwater_nav_path, 'models')
    gz_model_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    
    # Update environment variable for this process
    if model_path not in gz_model_path:
        os.environ['GZ_SIM_RESOURCE_PATH'] = f"{model_path}:{gz_model_path}"

    # --- 3. Launch Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # --- 4. Core Simulation Nodes (Gazebo & Bridge) ---
    
    # Gazebo Simulator
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # ROS-Gazebo Bridge
    # Note: We use a list for the argument to correctly resolve the PathJoinSubstitution
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p', ['config_file:=', bridge_config],
        ],
        output='screen'
    )
    
    # Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'orca4',
            '-file', robot_file,
            '-x', '0.0', '-y', '0.0', '-z', '0.0',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0'
        ],
        output='screen'
    )

    # --- 5. Autonomy Nodes (Your Control Stack) ---
    
    camera_processor = Node(
        package='underwater_navigation',
        executable='camera_processor_node.py',
        name='camera_processor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    gate_navigation = Node(
        package='underwater_navigation',
        executable='gate_navigation_node.py',
        name='gate_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    thruster_controller = Node(
        package='underwater_navigation',
        executable='thruster_controller_node.py',
        name='thruster_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # --- 6. Visualization ---
    
    rqt_image_view = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view'],
        output='screen'
    )
    
    # --- 7. Return Description ---
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                            description='Use simulation clock'),
        gz_sim,
        bridge,
        spawn_robot,
        camera_processor,
        gate_navigation,
        thruster_controller,
        rqt_image_view,
    ])
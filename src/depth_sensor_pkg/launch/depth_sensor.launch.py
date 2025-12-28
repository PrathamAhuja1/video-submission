from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyS4',
        description='Serial port for MS5837 sensor'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='9600',
        description='Baud rate for serial communication'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Publishing rate in Hz'
    )
    
    # Create node
    depth_sensor_node = Node(
        package='depth_sensor_pkg',
        executable='depth_sensor_node',
        name='depth_sensor_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }]
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        publish_rate_arg,
        depth_sensor_node
    ])
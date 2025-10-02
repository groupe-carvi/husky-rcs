from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for Husky Robot Control Server."""
    
    # Declare launch arguments
    websocket_port_arg = DeclareLaunchArgument(
        'websocket_port',
        default_value='8766',
        description='Port for WebSocket server'
    )
    
    websocket_host_arg = DeclareLaunchArgument(
        'websocket_host',
        default_value='localhost',
        description='Host for WebSocket server'
    )
    
    # Define the node
    husky_control_node = Node(
        package='husky_control_server',
        executable='husky_control_server.py',
        name='husky_control_server',
        output='screen',
        parameters=[{
            'websocket_port': LaunchConfiguration('websocket_port'),
            'websocket_host': LaunchConfiguration('websocket_host'),
        }]
    )
    
    return LaunchDescription([
        websocket_port_arg,
        websocket_host_arg,
        husky_control_node
    ])
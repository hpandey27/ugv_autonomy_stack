import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('uikit_web_server')
    web_root = os.path.join(pkg_share, 'web_root')
    port = LaunchConfiguration('port', default='8000')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8000', description='Web server port'),
        
        # ROSBridge WebSocket (Port 9090 default)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen'
        ),

        # Simple HTTP Server for the Dashboard
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', port, '--directory', web_root],
            output='screen',
            name='dashboard_server'
        )
    ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('hardware_bridge')
    config_file = os.path.join(pkg_share, 'config', 'mavros_params.yaml')

    # connect to mavlink-router's local endpoint
    fcu_url = LaunchConfiguration('fcu_url', default='udp://127.0.0.1:14540@14540')
    gcs_url = LaunchConfiguration('gcs_url', default='') # MAVROS doesn't need to bridge GCS anymore, router does it

    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value='udp://127.0.0.1:14540@14540', description='Connection to mavlink-router'),
        DeclareLaunchArgument('gcs_url', default_value='', description='GCS connection (Disabled, handled by router)'),

        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            parameters=[
                config_file,
                {'fcu_url': fcu_url},
                {'gcs_url': gcs_url},
                {'system_id': 1},
                {'component_id': 1},
                {'target_system_id': 1},
                {'target_component_id': 1},
            ],
            remappings=[
                ('/mavros/setpoint_velocity/cmd_vel_unstamped', '/cmd_vel'),
                ('/mavros/local_position/odom', '/odom'),
            ]
        )
    ])

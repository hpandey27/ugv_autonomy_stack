import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('navigation_server')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_dir = os.path.join(pkg_share, 'maps')
    map_file = LaunchConfiguration('map', default=os.path.join(map_dir, 'empty.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=map_file, description='Full path to map file to load'),
        DeclareLaunchArgument('params_file', default_value=params_file, description='Full path to param file to load'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'False',
                'params_file': params_file,
                'autostart': 'True',
            }.items()
        )
    ])

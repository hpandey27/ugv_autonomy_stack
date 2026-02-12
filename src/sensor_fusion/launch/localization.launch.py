import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    sensor_fusion_dir = get_package_share_directory('sensor_fusion')
    ekf_config = os.path.join(sensor_fusion_dir, 'config', 'ekf.yaml')
    
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
            remappings=[('odometry/filtered', '/odometry/local')]
        )
    ])

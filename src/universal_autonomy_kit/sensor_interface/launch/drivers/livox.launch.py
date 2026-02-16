from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    name = LaunchConfiguration('name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')
    topic_remap = LaunchConfiguration('topic_remap')

    return LaunchDescription([
        # Static TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=[name, '_tf_publisher'],
            arguments=[x, y, z, yaw, pitch, roll, 'base_link', [name, '_link']],
        ),
        
        # Livox Driver (Mock/Wrapper)
        # In a real deployment, this would include the specific livox_ros_driver2 config
        Node(
            package='component_container', # Placeholder if driver not installed
            executable='component_container',
            name=[name, '_driver'],
            output='screen',
            # remappings=[('/livox/lidar', topic_remap)]
        )
    ])

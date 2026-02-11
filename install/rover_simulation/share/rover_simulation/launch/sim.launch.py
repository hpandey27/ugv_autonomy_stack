from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():

    desc_pkg = get_package_share_directory("rover_description")
    xacro_file = os.path.join(desc_pkg, "urdf", "rover.urdf.xacro")

    robot_desc = xacro.process_file(xacro_file).toxml()

    gazebo = ExecuteProcess(
        cmd=["gazebo", "--verbose", "-s", "libgazebo_ros_factory.so"],
        output="screen",
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
    )

    spawn = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-entity", "ugv",
                    "-topic", "robot_description",
                    "-x", "0",
                    "-y", "0",
                    "-z", "0.5"
                ],
                output="screen",
            )
        ],
    )

    joint_state_spawner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen",
            )
        ],
    )

    diff_drive_spawner = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_drive_controller"],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
        joint_state_spawner,
        diff_drive_spawner,
    ])

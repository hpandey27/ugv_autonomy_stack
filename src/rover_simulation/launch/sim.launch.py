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

    gazebo_models_path = os.path.join(desc_pkg, "models")
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_models_path

    world_file = '/usr/share/gazebo-11/worlds/shapes.world'
    
    gazebo = ExecuteProcess(
        cmd=["gzserver", "--verbose", "-s", "libgazebo_ros_factory.so", world_file],
        output="screen",
    )

    # If you want to see the GUI, launch gzclient too.
    # We use headless:=true by default in our tests.

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc, "use_sim_time": True}],
    )

    spawn = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-entity", "ugv",
                    "-topic", "robot_description",
                    "-x", "-2.0",
                    "-y", "-2.0",
                    "-z", "0.3"  # Lower spawn height to land on ground, not obstacles
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
    ])

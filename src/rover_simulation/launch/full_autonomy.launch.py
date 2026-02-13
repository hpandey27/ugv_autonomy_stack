#!/usr/bin/env python3
"""
Master launch file for complete UGV autonomy stack.
Launches simulation, localization, SLAM, and navigation with proper sequencing.
Designed for robustness with latency tolerance and graceful degradation.
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
    RegisterEventHandler,
    LogInfo,
    DeclareLaunchArgument,
)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch sequence with timing to handle latency:
    1. Gazebo simulation (0s)
    2. Localization (5s delay - wait for simulation to stabilize)
    3. SLAM Toolbox (8s delay - wait for localization TF)
    4. Nav2 (12s delay - wait for map to initialize)
    5. RViz2 (15s delay - wait for all data streams)
    """

    # Declare arguments for flexibility
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from Gazebo'
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode (no GUI)'
    )

    # Get package directories
    sim_pkg = get_package_share_directory('rover_simulation')
    loc_pkg = get_package_share_directory('rover_localization')
    map_pkg = get_package_share_directory('rover_mapping')
    nav_pkg = get_package_share_directory('rover_navigation')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')

    # 1. Simulation (starts immediately)
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg, 'launch', 'sim.launch.py')
        ),
    )

    # 2. Localization (5s delay - wait for Gazebo to stabilize)
    localization = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="[MASTER] Starting localization (EKF sensor fusion)..."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(loc_pkg, 'launch', 'localization.launch.py')
                ),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
            ),
        ],
    )

    # 3. SLAM Toolbox (8s delay - wait for localization TF tree)
    slam = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg="[MASTER] Starting SLAM Toolbox (mapping)..."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(map_pkg, 'launch', 'mapping.launch.py')
                ),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
            ),
        ],
    )

    # 4. Nav2 (12s delay - wait for map initialization)
    navigation = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg="[MASTER] Starting Nav2 navigation stack..."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_pkg, 'launch', 'navigation.launch.py')
                ),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
            ),
        ],
    )

    # 5. RViz2 (15s delay - wait for all data streams)
    rviz_config = os.path.join(sim_pkg, 'config', 'autonomy.rviz')
    rviz = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg="[MASTER] Starting RViz2 visualization..."),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
            ),
        ],
    )

    # Status monitoring (optional - helps debug timing issues)
    status_monitor = TimerAction(
        period=20.0,
        actions=[
            LogInfo(msg="[MASTER] ✅ Full autonomy stack should be operational!"),
            LogInfo(msg="[MASTER] Check RViz2 for sensor data, map, and navigation visualization."),
        ],
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        headless_arg,
        
        # Launch sequence with timing
        simulation,      # t=0s:  Gazebo + robot
        localization,    # t=5s:  EKF sensor fusion
        # slam,            # SLAM is now started by navigation.launch.py
        navigation,      # t=12s: Nav2 navigation
        rviz,            # t=15s: RViz2 visualization
        status_monitor,  # t=20s: Status message
    ])

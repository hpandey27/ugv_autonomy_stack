import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_share = get_package_share_directory('sensor_interface')
    config_path = os.path.join(pkg_share, 'config', 'sensors.yaml')
    
    launch_entities = []

    if not os.path.exists(config_path):
        launch_entities.append(LogInfo(msg=f"Sensor config not found at {config_path}"))
        return LaunchDescription(launch_entities)

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    for sensor in config.get('sensors', []):
        if not sensor.get('enabled', False):
            continue

        name = sensor['name']
        model = sensor['model']
        mount = sensor['mount']
        remap = sensor.get('topic_remap', '/unknown')

        # Determine driver launch file based on model
        # Heuristic mapping: model name -> launch filename
        # e.g. livox_mid360 -> drivers/livox.launch.py
        driver_file = 'unknown.launch.py'
        if 'livox' in model:
            driver_file = 'drivers/livox.launch.py'
        elif 'unitree' in model:
            driver_file = 'drivers/unitree.launch.py'
        elif 'orbbec' in model:
            driver_file = 'drivers/orbbec.launch.py'
        # Add more mappings for standard hardware (velodyne, sick, etc.)

        launch_path = os.path.join(pkg_share, 'launch', driver_file)
        
        if os.path.exists(launch_path):
            launch_entities.append(LogInfo(msg=f"Launching {name} ({model})"))
            launch_entities.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(launch_path),
                    launch_arguments={
                        'name': name,
                        'x': str(mount['xyz'][0]),
                        'y': str(mount['xyz'][1]),
                        'z': str(mount['xyz'][2]),
                        'roll': str(mount['rpy'][0]),
                        'pitch': str(mount['rpy'][1]),
                        'yaw': str(mount['rpy'][2]),
                        'topic_remap': remap
                    }.items()
                )
            )
        else:
            launch_entities.append(LogInfo(msg=f"Driver launch file not found for {model}: {launch_path}"))

    return LaunchDescription(launch_entities)

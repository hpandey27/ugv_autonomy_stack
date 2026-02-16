import os
import sys
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_urdf():
    # 1. Locate Config Files
    pkg_share = get_package_share_directory('rover_configurator')
    config_path = os.path.join(pkg_share, 'config', 'robot_specs.yaml')
    xacro_path = os.path.join(pkg_share, 'config', 'templates', 'base.xacro')

    if not os.path.exists(config_path):
        print(f"Error: Config file not found at {config_path}")
        return
    
    # 2. Parse YAML
    with open(config_path, 'r') as file:
        specs = yaml.safe_load(file)
    
    print(f"Generating URDF for: {specs['robot_name']} ({specs['drive_type']})")

    # 3. Map YAML to Xacro Arguments
    mappings = {
        'robot_name': specs['robot_name'],
        'chassis_length': str(specs['chassis']['length']),
        'chassis_width': str(specs['chassis']['width']),
        'chassis_height': str(specs['chassis']['height']),
        'wheel_radius': str(specs['wheels']['radius']),
        'wheel_width': str(specs['wheels']['width']),
        'wheel_separation': str(specs['wheels']['separation_width']), # Note mapping match
        'wheel_base': str(specs['wheels']['separation_length']),
    }
    
    # 4. Process Xacro
    doc = xacro.process_file(xacro_path, mappings=mappings)
    robot_desc = doc.toxml()

    # 5. Output (For now just print, usually this node would publish /robot_description)
    # In a launch file, we would usually use Command() to call xacro directly, 
    # but this script allows for complex pre-processing/validation before xacro.
    print("URDF Generation Successful.")
    return robot_desc

def main():
    try:
        urdf = generate_urdf()
        # For testing, write to a file in /tmp/
        with open('/tmp/generated_rover.urdf', 'w') as f:
            f.write(urdf)
        print(f"Preview written to /tmp/generated_rover.urdf")
    except Exception as e:
        print(f"Failed to generate URDF: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()

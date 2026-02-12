# UGV Autonomy Stack

Defense-grade autonomous navigation system for a 4-wheel skid-steer Unmanned Ground Vehicle (UGV).

## Architecture Overview

This repository implements a modular, defense-grade autonomy stack with clear functional separation:

```
src/
├── rover_description/      # Robot model (URDF, sensors, meshes)
├── rover_simulation/       # Gazebo integration & RViz configs
├── rover_localization/     # EKF-based state estimation
├── rover_mapping/          # SLAM Toolbox (async mapping)
├── rover_navigation/       # Nav2 stack (autonomous navigation)
├── sensor_drivers/         # Low-level sensor interfaces
├── tools/                  # Development utilities
└── vehicle_interface/      # Hardware abstraction layer
```

## Key Features

- **Sensor Fusion**: EKF-based localization fusing IMU and wheel odometry
- **SLAM**: Asynchronous mapping with CeresSolver for outdoor environments
- **Navigation**: Regulated Pure Pursuit controller with outdoor-optimized costmaps
- **Simulation**: Gazebo Classic integration with feature-rich test environments
- **Defense-Grade**: C++ middleware, modular composition, deterministic execution

## Package Descriptions

### rover_description
Robot model definition including URDF/Xacro files, sensor configurations (LiDAR, IMU), and collision geometry.

### rover_simulation
Gazebo simulation environment with world files, launch orchestration, and RViz visualization configs (`autonomy.rviz`).

### rover_localization
EKF-based state estimation using `robot_localization`. Fuses IMU and wheel odometry for accurate pose estimation.

### rover_mapping
SLAM Toolbox integration for asynchronous environment mapping. Configured with CeresSolver and 5cm grid resolution.

### rover_navigation
Nav2 stack integration with Regulated Pure Pursuit controller, NavFn global planner, and outdoor-optimized costmaps.

### sensor_drivers
Low-level interfaces for hardware sensors (placeholder for future expansion).

### tools
Development and debugging utilities.

### vehicle_interface
Hardware abstraction layer for platform-specific control interfaces.

## Quick Start

### Prerequisites
```bash
# ROS2 Humble
sudo apt install ros-humble-desktop

# Dependencies
sudo apt install ros-humble-robot-localization \
                 ros-humble-slam-toolbox \
                 ros-humble-navigation2 \
                 ros-humble-nav2-bringup \
                 ros-humble-nav2-regulated-pure-pursuit-controller
```

### Build
```bash
cd ugv_autonomy_stack
colcon build --symlink-install
source install/setup.bash
```

### Launch Simulation
```bash
# Terminal 1: Gazebo simulation
ros2 launch rover_simulation sim.launch.py

# Terminal 2: Localization (EKF)
ros2 launch rover_localization localization.launch.py

# Terminal 3: SLAM mapping
ros2 launch rover_mapping mapping.launch.py

# Terminal 4: Autonomous navigation
ros2 launch rover_navigation navigation.launch.py
```

### Teleoperation
```bash
ros2 topic pub --once /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

## Development Principles

This stack follows defense-grade development principles:

1. **Modularity**: Clear separation of concerns with single-responsibility packages
2. **Determinism**: Explicit parameters, no hidden state, reproducible behavior
3. **C++ Core**: Real-time critical components in C++, Python for orchestration only
4. **Explicit Configuration**: All parameters externalized in YAML files
5. **Traceability**: Comprehensive logging and monitoring capabilities

## System Requirements

- **OS**: Ubuntu 22.04 LTS
- **ROS**: ROS2 Humble
- **Simulator**: Gazebo Classic 11
- **Hardware**: 4-wheel skid-steer platform with LiDAR and IMU

## License

**Proprietary and Confidential**

Copyright © 2026 UGV Autonomy Project. All Rights Reserved.

This software is proprietary and confidential. See [LICENSE](LICENSE) and [COPYRIGHT.md](COPYRIGHT.md) for full details.

Unauthorized use, copying, modification, or distribution is strictly prohibited.

## Maintainer

UGV Development Team <maintainer@ugv-autonomy.local>

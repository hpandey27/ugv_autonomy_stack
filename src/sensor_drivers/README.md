# Sensor Drivers Package

ROS2 package for real hardware sensor driver integration.

## Supported Hardware

### LiDAR
- **Unitree 4D LiDAR**: High-resolution 3D LiDAR
  - Driver: TBD (vendor-specific ROS2 driver)
  - Topics: `/lidar/points` (sensor_msgs/PointCloud2)

- **Livox 360 LiDAR**: 360-degree coverage LiDAR
  - Driver: `livox_ros_driver2`
  - Topics: `/lidar_360/points` (sensor_msgs/PointCloud2)

### Camera
- **Orbbec Gemini XL**: RGB-D stereo camera
  - Driver: `OrbbecSDK_ROS2`
  - Topics:
    - `/camera/rgb/image_raw` (sensor_msgs/Image)
    - `/camera/depth/image_raw` (sensor_msgs/Image)
    - `/camera/depth/points` (sensor_msgs/PointCloud2)

### IMU
- **Cube Orange+**: Flight controller with high-precision IMU
  - Driver: MAVLink/MAVROS2 integration
  - Topics: `/imu/data` (sensor_msgs/Imu)

### GPS
- **Here 3 / Here 4**: RTK GPS modules
  - Driver: ublox driver or MAVROS2 GPS
  - Topics: `/gps/fix` (sensor_msgs/NavSatFix)

### Motor Controller
- **VESC 75100 V2 Pro**: Electronic speed controller
  - Driver: vesc_driver (ROS2 compatible)
  - Topics: `/cmd_vel` (geometry_msgs/Twist) → motor commands

## Installation

### Dependencies

```bash
# Livox LiDAR driver
cd ~/workspace/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd ..
colcon build --packages-select livox_ros_driver2

# Orbbec camera driver
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
cd OrbbecSDK_ROS2
# Follow installation instructions

# MAVROS2 for Cube Orange+
sudo apt install ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras
```

## Usage

### Simulation (Gazebo)
Sensors are automatically spawned via URDF plugins when launching simulation:
```bash
ros2 launch rover_simulation sim.launch.py
```

### Real Hardware
Launch individual sensor drivers:
```bash
# Livox LiDAR
ros2 launch sensor_drivers livox.launch.py

# Orbbec Camera
ros2 launch sensor_drivers orbbec.launch.py

# All sensors
ros2 launch sensor_drivers all_sensors.launch.py
```

## TODO
- [ ] Create launch files for each sensor
- [ ] Integrate Unitree 4D LiDAR driver (vendor-specific)
- [ ] Configure Livox 360 driver
- [ ] Configure Orbbec Gemini XL driver
- [ ] Configure Cube Orange+ MAVLink interface
- [ ] Configure VESC motor controller interface
- [ ] Add sensor health monitoring nodes

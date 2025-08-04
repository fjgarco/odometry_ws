# Comprehensive ROS 2 Odometry Workspace

Complete multi-sensor odometry system for ROS 2 Humble featuring LIDAR, IMU/Magnetometer, GPS, and Encoder odometry estimation with advanced sensor fusion capabilities.

## Overview

This workspace provides a comprehensive odometry estimation framework with four independent sensor-based odometry packages and an intelligent adaptive covariance system:

1. **LIDAR Odometry (FastGICP)** - High-precision point cloud registration for LIVOX HAP sensor
2. **IMU/Magnetometer Odometry** - Madgwick filter-based orientation and motion estimation  
3. **GPS Odometry** - Absolute positioning from GNSS RTK data with geodetic conversion
4. **Encoder Odometry** - Wheel-based odometry using quadrature encoders and Jetson GPIO
5. **Adaptive Covariance** - Intelligent sensor health monitoring and dynamic EKF covariance adjustment

All packages are designed to work independently or together in a sensor fusion framework, with the adaptive covariance system providing real-time optimization of fusion parameters based on sensor health assessment.

## System Architecture

```
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│   LIVOX HAP     │  │   IMU/MAG       │  │   GPS RTK       │  │  4WD ENCODERS   │
│   /livox/lidar  │  │   /mavros/imu/* │  │   /mavros/gps/* │  │  GPIO Pins      │
└─────────┬───────┘  └─────────┬───────┘  └─────────┬───────┘  └─────────┬───────┘
          │                    │                    │                    │
          ▼                    ▼                    ▼                    ▼
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│ LIDAR Odometry  │  │ IMU/MAG Odom    │  │ GPS Odometry    │  │ Encoder Odom    │
│ (FastGICP)      │  │ (Madgwick)      │  │ (Geodetic)      │  │ (Quadrature)    │
└─────────┬───────┘  └─────────┬───────┘  └─────────┬───────┘  └─────────┬───────┘
          │                    │                    │                    │
          ▼                    ▼                    ▼                    ▼
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│/odometry/lidar  │  │/odometry/imu_mag│  │ /odometry/gps   │  │/odometry/encoders│
└─────────┬───────┘  └─────────┬───────┘  └─────────┬───────┘  └─────────┬───────┘
          │                    │                    │                    │
          └──────────┬─────────┴──────────┬─────────┴────────────────────┘
                     ▼                    ▼
          ┌─────────────────┐    ┌─────────────────┐
          │ Adaptive        │    │ Sensor Fusion   │
          │ Covariance      │    │ (robot_locali-  │
          │ (Health Monitor │    │  zation, EKF)   │
          │ + EKF Tuning)   │    │                 │
          └─────────────────┘    └─────────────────┘
```

## Package Structure

```
odometry_ws/
├── src/
│   ├── fast_gicp/                     # External FastGICP library
│   ├── lidar_odometry_fastgicp/       # LIDAR odometry package
│   │   ├── src/                       # C++ implementation
│   │   ├── launch/                    # Launch files
│   │   ├── config/                    # Configuration files
│   │   └── README.md                  # Package documentation
│   ├── imu_mag_odometry/              # IMU/Magnetometer package
│   │   ├── imu_mag_odometry/          # Python implementation
│   │   ├── launch/                    # Launch files
│   │   ├── config/                    # Configuration files
│   │   └── README.md                  # Package documentation
│   ├── gps_odometry/                  # GPS odometry package
│   │   ├── gps_odometry/              # Python implementation
│   │   ├── launch/                    # Launch files
│   │   ├── config/                    # Configuration files
│   │   └── README.md                  # Package documentation
│   ├── encoder_odometry/              # Encoder odometry package
│   │   ├── encoder_odometry/          # Python implementation
│   │   ├── msg/                       # Custom message definitions
│   │   ├── launch/                    # Launch files
│   │   ├── config/                    # Configuration files
│   │   └── README.md                  # Package documentation
│   ├── adaptive_covariance/           # Adaptive covariance package
│   │   ├── adaptive_covariance/       # Python implementation
│   │   ├── launch/                    # Launch files
│   │   ├── config/                    # Configuration files
│   │   └── README.md                  # Package documentation
│   └── complete_odometry.launch.py    # Main workspace launch file
└── README.md                          # This file
```

## Quick Start

### 1. Dependencies Installation

```bash
# System dependencies
sudo apt update
sudo apt install -y \
    libeigen3-dev \
    libpcl-dev \
    pcl-tools \
    python3-pip \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-tf2-ros \
    ros-humble-mavros-msgs \
    ros-humble-message-filters \
    ros-humble-robot-localization

# Python dependencies
pip3 install pyproj numpy scipy

# Jetson GPIO for encoder odometry (Jetson boards only)
sudo pip3 install Jetson.GPIO
```

### 2. Workspace Setup

```bash
# Clone or create workspace
cd ~/Desktop
git clone <repository-url> odometry_ws  # or use existing workspace
cd odometry_ws

# Build all packages
colcon build

# Source the workspace
source install/setup.bash
```

### 3. Launch Complete System

```bash
# Launch all odometry nodes with adaptive covariance
ros2 launch src/complete_odometry.launch.py

# Launch without adaptive covariance (static EKF parameters)
ros2 launch src/complete_odometry.launch.py use_adaptive_covariance:=false

# Launch individual packages
ros2 launch lidar_odometry_fastgicp lidar_odometry.launch.py
ros2 launch imu_mag_odometry imu_mag_odometry.launch.py
ros2 launch gps_odometry gps_odometry.launch.py
ros2 launch encoder_odometry encoder_odometry.launch.py
ros2 launch adaptive_covariance adaptive_covariance.launch.py

# Launch with custom topics and adaptive covariance
ros2 launch src/complete_odometry.launch.py \
    lidar_topic:=/your/lidar/topic \
    imu_topic:=/your/imu/topic \
    gps_topic:=/your/gps/topic \
    use_encoders:=true \
    use_adaptive_covariance:=true
```

## Package Details

### LIDAR Odometry (FastGICP)

**Purpose**: High-precision point cloud registration using FastGICP algorithm optimized for LIVOX HAP sensor.

**Key Features**:
- GPU-accelerated FastGICP implementation
- Adaptive voxel grid filtering
- Configurable registration parameters
- Multi-threading support

**Topics**:
- Input: `/livox/lidar` (sensor_msgs/PointCloud2)
- Output: `/odometry/lidar` (nav_msgs/Odometry)

### IMU/Magnetometer Odometry

**Purpose**: Orientation and motion estimation using Madgwick filter for IMU and magnetometer fusion.

**Key Features**:
- Custom Madgwick filter implementation
- Magnetic declination correction
- Optional position integration
- Configurable filter parameters

**Topics**:
- Input: `/mavros/imu/data` (sensor_msgs/Imu), `/mavros/imu/mag` (sensor_msgs/MagneticField)
- Output: `/odometry/imu_mag` (nav_msgs/Odometry)

### GPS Odometry

**Purpose**: Absolute positioning from GNSS RTK data with geodetic coordinate conversion.

**Key Features**:
- ENU and UTM coordinate systems
- RTK validation and filtering
- Flexible origin modes
- High-precision coordinate conversion

**Topics**:
- Input: `/mavros/global_position/raw/fix` (sensor_msgs/NavSatFix)
- Output: `/odometry/gps` (nav_msgs/Odometry)

### Encoder Odometry

**Purpose**: Wheel-based odometry using quadrature encoders and Jetson GPIO for precise dead reckoning.

**Key Features**:
- Interrupt-based quadrature decoding
- 4WD/2WD configurable kinematics  
- Per-wheel direction inversion
- Raw encoder tick debugging output

**Topics**:
- Input: GPIO pins (quadrature A/B signals)
- Output: `/odometry/encoders` (nav_msgs/Odometry), `/encoders/ticks` (custom debug)

### Adaptive Covariance

**Purpose**: Intelligent sensor health monitoring and dynamic EKF covariance adjustment for robust multi-sensor fusion.

**Key Features**:
- Real-time sensor health assessment using configurable heuristics
- Dynamic EKF parameter adjustment based on sensor performance
- Safety-first design with conservative limits and rollback capabilities
- Comprehensive diagnostics and performance logging

**Topics**:
- Input: All sensor topics (monitoring)
- Output: `/adaptive_covariance/sensor_health`, `/ekf_adapter/parameter_changes`
- Interface: `robot_localization` parameter services

### Sync Monitor

**Purpose**: Multi-sensor temporal synchronization for accurate sensor fusion analysis and adaptive covariance adjustment.

**Key Features**:
- `message_filters` based temporal alignment (±50ms configurable)
- Robust fallback operation with missing sensors
- Real-time delay monitoring and quality assessment
- Scenario-specific configurations (indoor/outdoor/high-speed)

**Topics**:
- Input: All odometry and sensor topics
- Output: `/adaptive_covariance/synchronized_data`, `/adaptive_covariance/sync_status`
- Service: `/adaptive_covariance/get_sync_status`

## Configuration

### Global Parameters

Each package can be configured independently. Common parameters:

```yaml
# Frame configuration
base_frame: "base_link"
odom_frame: "odom_[sensor]"

# Publishing settings
publish_tf: true
publish_rate: 30.0

# Validation settings
enable_validation: true
```

### LIDAR Configuration

```yaml
# FastGICP parameters
max_correspondence_distance: 1.0
max_iterations: 64
transformation_epsilon: 0.01
euclidean_fitness_epsilon: 0.01

# Filtering parameters  
voxel_leaf_size: 0.1
max_range: 100.0
min_range: 0.5
```

### IMU/MAG Configuration

```yaml
# Madgwick filter
beta: 0.1
magnetic_declination: 0.0
use_mag: true

# Integration
integrate_position: false
reset_on_large_velocity: true
```

### GPS Configuration

```yaml
# Coordinate system
coordinate_system: "enu"  # or "utm"

# Validation
min_fix_quality: 1
max_position_covariance: 25.0

# Origin
fixed_origin_lat: null  # Auto-set from first fix
```

## Integration with Navigation Stack

### robot_localization Example

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    
    # Enable adaptive covariance parameter services
    enable_parameter_services: true
    
    # Odometry sources
    odom0: /odometry/lidar
    odom0_config: [true,  true,  true,   # x, y, z
                   false, false, true,   # roll, pitch, yaw
                   false, false, false,  # vx, vy, vz
                   false, false, true,   # vroll, vpitch, vyaw
                   false, false, false]  # ax, ay, az
                   
    odom1: /odometry/imu_mag
    odom1_config: [false, false, false,  # x, y, z
                   true,  true,  true,   # roll, pitch, yaw
                   false, false, false,  # vx, vy, vz
                   true,  true,  true,   # vroll, vpitch, vyaw
                   false, false, false]  # ax, ay, az
                   
    odom2: /odometry/gps
    odom2_config: [true,  true,  true,   # x, y, z (absolute position)
                   false, false, false,  # roll, pitch, yaw
                   false, false, false,  # vx, vy, vz
                   false, false, false,  # vroll, vpitch, vyaw
                   false, false, false]  # ax, ay, az
                   
    # Process noise covariance (adjusted dynamically by adaptive_covariance)
    process_noise_covariance: [0.05, 0, 0, ...]  # Will be modified in real-time
```

## Monitoring and Debugging

```bash
# Monitor all odometry topics
ros2 topic list | grep odometry
ros2 topic hz /odometry/lidar
ros2 topic hz /odometry/imu_mag  
ros2 topic hz /odometry/gps

# Check TF tree
ros2 run tf2_tools view_frames

# Performance analysis
ros2 run plotjuggler plotjuggler
```

## Troubleshooting

### Common Issues

1. **FastGICP Compilation Errors**
   ```bash
   # Install CUDA development packages
   sudo apt install nvidia-cuda-dev nvidia-cuda-toolkit
   
   # Rebuild with verbose output
   colcon build --packages-select lidar_odometry_fastgicp --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON
   ```

2. **GPS Origin Not Set**
   ```bash
   # Check GPS fix quality
   ros2 topic echo /mavros/global_position/raw/fix --field status.status
   
   # Verify GPS messages
   ros2 topic hz /mavros/global_position/raw/fix
   ```

3. **IMU Calibration Issues**
   ```bash
   # Check IMU data
   ros2 topic echo /mavros/imu/data
   
   # Verify magnetometer
   ros2 topic echo /mavros/imu/mag
   ```

## Package Status

✅ **LIDAR Odometry (FastGICP)**: Complete implementation with CUDA support  
✅ **IMU/Magnetometer Odometry**: Complete with Madgwick filter  
✅ **GPS Odometry**: Complete with geodetic conversion (ENU/UTM)  
✅ **Encoder Odometry**: Complete with 4WD quadrature decoding and Jetson GPIO  
✅ **Adaptive Covariance**: Complete with intelligent sensor health monitoring and EKF tuning  
✅ **Sync Monitor**: Complete with multi-sensor temporal synchronization and quality assessment  

## License

This workspace is licensed under the MIT License. Individual packages may have different licenses - see package-specific LICENSE files.

## Support

For issues and questions:
1. Check package-specific READMEs
2. Review troubleshooting sections
3. Create GitHub issues with detailed descriptions
4. Include relevant log outputs and configuration files

## Acknowledgments

- **FastGICP**: SMRT-AIST research group
- **Madgwick Filter**: Sebastian Madgwick's algorithm implementation
- **ROS 2 Community**: Navigation and localization stack contributors

# GPS Odometry Package

ROS 2 package for estimating absolute position from GNSS/GPS data, specifically designed for RTK (Real-Time Kinematic) applications with high precision requirements.

## Overview

This package provides GPS-based odometry estimation by converting GPS coordinates (latitude, longitude, altitude) to local coordinate systems (ENU or UTM) and publishing standard ROS 2 odometry messages. It's designed to work with MAVROS for PX4/ArduPilot integration and supports both standard GPS and high-precision RTK modes.

## Features

- **Multiple Coordinate Systems**: Support for ENU (East-North-Up) and UTM coordinate systems
- **Flexible Origin Modes**: Automatic origin from first valid fix or fixed survey origin
- **RTK Integration**: Optimized for high-precision RTK GNSS applications
- **Robust Validation**: Comprehensive GPS fix validation with configurable thresholds
- **Standard Interfaces**: Compatible with ROS 2 navigation stack and tf2
- **MAVROS Integration**: Direct compatibility with MAVROS GPS topics
- **Diagnostic Information**: Real-time statistics and validation feedback

## Installation

### Dependencies

```bash
# Install required Python packages
sudo apt install python3-pip
pip3 install pyproj

# ROS 2 dependencies (usually already installed)
sudo apt install ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-geometry-msgs
sudo apt install ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
```

### Build

```bash
cd ~/odometry_ws
colcon build --packages-select gps_odometry
source install/setup.bash
```

## Usage

### Basic GPS Odometry

```bash
# Launch with default parameters
ros2 launch gps_odometry gps_odometry.launch.py

# Launch with custom GPS topic
ros2 launch gps_odometry gps_odometry.launch.py gps_topic:=/your/gps/topic

# Launch with UTM coordinate system
ros2 launch gps_odometry gps_odometry.launch.py coordinate_system:=utm
```

### RTK High-Precision Mode

```bash
# Launch with RTK configuration
ros2 launch gps_odometry gps_odometry.launch.py config_file:=$(ros2 pkg prefix gps_odometry)/share/gps_odometry/config/gps_odometry_rtk.yaml
```

### Manual Node Execution

```bash
# Run the node directly
ros2 run gps_odometry gps_odometry_node

# With custom parameters
ros2 run gps_odometry gps_odometry_node --ros-args \
  -p gps_topic:=/mavros/global_position/raw/fix \
  -p output_topic:=/odometry/gps \
  -p coordinate_system:=enu \
  -p min_fix_quality:=2
```

## Configuration

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `gps_topic` | string | `/mavros/global_position/raw/fix` | GPS NavSatFix input topic |
| `output_topic` | string | `/odometry/gps` | Odometry output topic |
| `base_frame` | string | `base_link` | Robot base frame |
| `odom_frame` | string | `odom_gps` | GPS odometry frame |
| `coordinate_system` | string | `enu` | Local coordinate system (`enu` or `utm`) |
| `fixed_origin_lat` | double | `null` | Fixed origin latitude (optional) |
| `fixed_origin_lon` | double | `null` | Fixed origin longitude (optional) |
| `fixed_origin_alt` | double | `0.0` | Fixed origin altitude |
| `min_fix_quality` | int | `1` | Minimum NavSatStatus.status value |
| `max_position_covariance` | double | `25.0` | Maximum position covariance (m²) |
| `validate_coordinates` | bool | `true` | Validate coordinate sanity |
| `publish_tf` | bool | `true` | Publish TF transforms |
| `publish_stats` | bool | `true` | Publish periodic statistics |
| `default_position_covariance` | double | `1.0` | Default position uncertainty |
| `orientation_covariance` | double | `999999.0` | Orientation uncertainty (GPS doesn't provide) |
| `velocity_covariance` | double | `999999.0` | Velocity uncertainty (not estimated) |

### Coordinate Systems

#### ENU (East-North-Up)
- **Best for**: Local applications, robot navigation, small areas
- **Characteristics**: Simple, intuitive, minimal distortion for local use
- **Range**: Suitable for areas up to ~100km from origin

#### UTM (Universal Transverse Mercator)
- **Best for**: Large-scale applications, surveying, global mapping
- **Characteristics**: Metric coordinates, global consistency
- **Range**: Suitable for global applications, automatic zone handling

### Fix Quality Levels

| Status | Description | Typical Accuracy |
|--------|-------------|------------------|
| 0 | No fix | N/A |
| 1 | Standard GPS | 3-5 meters |
| 2 | DGPS | 1-3 meters |
| 3 | PPS | Sub-meter |
| 4 | RTK Fixed | 1-2 cm |
| 5 | RTK Float | 10-50 cm |

## Topics

### Subscribed Topics

- `gps_topic` (`sensor_msgs/NavSatFix`): GPS position data

### Published Topics

- `output_topic` (`nav_msgs/Odometry`): GPS-based odometry
- `/tf` (`tf2_msgs/TFMessage`): Transform from odom_frame to base_frame (if enabled)

## Example Applications

### Survey Mapping with Fixed Origin

```yaml
/**:
  ros__parameters:
    coordinate_system: "utm"
    fixed_origin_lat: 40.123456789
    fixed_origin_lon: -74.987654321
    fixed_origin_alt: 100.0
    min_fix_quality: 4  # RTK fixed only
    max_position_covariance: 0.01
```

### Mobile Robot Navigation

```yaml
/**:
  ros__parameters:
    coordinate_system: "enu"
    min_fix_quality: 1
    max_position_covariance: 10.0
    publish_tf: true
```

### Integration with robot_localization

```yaml
ekf_filter_node:
  ros__parameters:
    odom0: /odometry/gps
    odom0_config: [true,  true,  true,   # x, y, z
                   false, false, false,  # roll, pitch, yaw
                   false, false, false,  # vx, vy, vz
                   false, false, false,  # vroll, vpitch, vyaw
                   false, false, false]  # ax, ay, az
```

## Coordinate Conversion Details

### ENU Conversion Process
1. Set origin from first valid fix or fixed coordinates
2. Convert origin to ECEF (Earth-Centered-Earth-Fixed)
3. For each GPS point:
   - Convert to ECEF
   - Subtract origin ECEF
   - Rotate to ENU frame

### UTM Conversion Process
1. Determine UTM zone from GPS coordinates
2. Convert GPS coordinates to UTM easting/northing
3. Set origin in UTM coordinates
4. Convert subsequent points relative to origin

## Troubleshooting

### Common Issues

1. **No Origin Set Warning**
   ```
   Solution: Ensure GPS is providing valid fixes, or set fixed_origin parameters
   ```

2. **High Invalid Fix Rate**
   ```
   Check: GPS antenna placement, sky visibility, fix quality settings
   Consider: Lowering min_fix_quality for testing
   ```

3. **Large Position Jumps**
   ```
   Check: max_position_covariance setting, GPS multipath interference
   Consider: Using RTK mode for better precision
   ```

4. **Coordinate System Issues**
   ```
   Verify: Coordinate system choice appropriate for application
   ENU: Better for local navigation
   UTM: Better for large-scale mapping
   ```

### Diagnostic Commands

```bash
# Check GPS message rate
ros2 topic hz /mavros/global_position/raw/fix

# Monitor GPS fix quality
ros2 topic echo /mavros/global_position/raw/fix --field status.status

# Check odometry output
ros2 topic echo /odometry/gps

# Monitor coordinate conversion
ros2 topic echo /odometry/gps --field pose.pose.position

# Check TF tree
ros2 run tf2_tools view_frames
```

## Integration Examples

### With Navigation2

```xml
<node pkg="nav2_map_server" exec="map_server" name="map_server">
  <param name="yaml_filename" value="your_map.yaml"/>
  <param name="frame_id" value="odom_gps"/>
</node>
```

### With SLAM Toolbox

```yaml
slam_toolbox:
  ros__parameters:
    odom_frame: odom_gps
    map_frame: map
    base_frame: base_link
```

## Performance Considerations

- **Update Rate**: Typically matches GPS rate (1-10 Hz)
- **Latency**: Minimal processing latency (~1ms coordinate conversion)
- **Memory**: Low memory footprint
- **CPU**: Negligible CPU usage for coordinate conversion

## License

This package is licensed under the MIT License. See LICENSE file for details.

## Contributing

Contributions are welcome! Please see CONTRIBUTING.md for guidelines.

## Authors

- GPS Odometry Package Development Team
- ROS 2 Humble Integration

## Changelog

### Version 1.0.0
- Initial release with ENU and UTM coordinate systems
- RTK support and validation
- MAVROS integration
- Comprehensive parameter configuration

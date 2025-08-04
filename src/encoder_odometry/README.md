# Encoder Odometry Package

ROS 2 package for 4WD encoder-based odometry using Jetson GPIO and quadrature encoders. Provides accurate wheel-based odometry for differential drive robots with Hall effect magnetic encoders.

## Overview

This package reads quadrature encoder signals from 4 wheels using Jetson GPIO interrupts, calculates robot odometry using differential kinematics, and publishes standard ROS 2 odometry messages. It's designed for robots with magnetic Hall effect encoders providing A/B quadrature signals.

## Features

- **4WD/2WD Support**: Configurable drive modes with proper kinematics
- **Interrupt-Based Reading**: Uses GPIO interrupts for accurate, real-time encoder reading
- **Robust Quadrature Decoding**: Handles direction detection and noise filtering
- **Standard ROS 2 Integration**: Publishes nav_msgs/Odometry and TF transforms
- **Raw Data Publishing**: Optional encoder tick debugging output
- **Configurable Parameters**: All robot dimensions and GPIO pins configurable
- **Direction Inversion**: Per-wheel direction inversion for motor mounting
- **Velocity Filtering**: Low-pass filtering for smooth velocity estimation
- **Reset Service**: Runtime odometry reset capability

## Hardware Requirements

### Robot Specifications
- **4-wheel drive robot** with magnetic Hall effect encoders
- **Encoder Resolution**: 537.7 PPR (Pulses Per Revolution) at wheel output
- **Wheel Diameter**: 215mm (configurable)
- **Track Width**: 380mm (distance between left/right wheels)
- **Wheelbase**: 364.5mm (distance between front/rear axles)

### GPIO Pin Assignment (Jetson BCM)

| Encoder Position | Channel A | Channel B |
|------------------|-----------|-----------|
| Front Left (FL)  | GPIO 17   | GPIO 18   |
| Front Right (FR) | GPIO 27   | GPIO 22   |
| Rear Left (RL)   | GPIO 23   | GPIO 24   |
| Rear Right (RR)  | GPIO 25   | GPIO 4    |

### Wiring Diagram

```
Encoder Connector    Jetson GPIO
┌─────────────────┐  ┌────────────┐
│ Channel A  ────►│──│ GPIO Pin   │
│ Channel B  ────►│──│ GPIO Pin   │
│ VCC (5V)   ────►│  │ DON'T      │ ⚠️  DO NOT CONNECT
│ GND        ────►│  │ CONNECT    │ ⚠️  TO JETSON!
└─────────────────┘  └────────────┘
```

## ⚠️ CRITICAL SAFETY WARNINGS

### 1. **DO NOT CONNECT VCC OR GND**
- **NEVER** connect encoder VCC (5V) or GND to Jetson GPIO pins
- **ONLY** connect signal wires (Channel A and B)
- Jetson GPIO pins are **3.3V logic only**

### 2. **Voltage Level Conversion**
- If encoders output **5V logic levels**, use a **logic level converter**
- Convert 5V signals to 3.3V before connecting to Jetson
- Recommended: Bi-directional logic level converter (e.g., SparkFun BOB-12009)

### 3. **Power Supply**
- Encoders should be powered from **separate 5V supply**
- Use common ground between encoder power supply and Jetson (through level converter)
- Ensure proper grounding to avoid signal noise

### Connection Example with Level Converter

```
Encoder (5V)     Level Converter    Jetson (3.3V)
┌──────────┐     ┌─────────────┐    ┌─────────────┐
│ Ch A (5V)│────►│ HV1 → LV1   │───►│ GPIO 17     │
│ Ch B (5V)│────►│ HV2 → LV2   │───►│ GPIO 18     │
│ VCC (5V) │────►│ HV Power    │    │             │
│ GND      │────►│ GND         │    │ GND         │◄──┤
└──────────┘     │ LV Power    │◄───│ 3.3V        │
                 └─────────────┘    └─────────────┘
```

## Installation

### Dependencies

```bash
# Install Jetson.GPIO for hardware access
sudo apt update
sudo apt install -y python3-pip
sudo pip3 install Jetson.GPIO

# Grant GPIO access permissions (add your user to gpio group)
sudo usermod -a -G gpio $USER
# Log out and log back in for group changes to take effect

# ROS 2 dependencies (usually already installed)
sudo apt install -y \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-tf2-ros \
    ros-humble-std-srvs
```

### Build Package

```bash
cd ~/odometry_ws
colcon build --packages-select encoder_odometry
source install/setup.bash
```

## Usage

### Basic Launch

```bash
# Launch with default parameters
ros2 launch encoder_odometry encoder_odometry.launch.py

# Launch with custom configuration
ros2 launch encoder_odometry encoder_odometry.launch.py \
    config_file:=/path/to/custom_config.yaml

# Launch with custom output topic
ros2 launch encoder_odometry encoder_odometry.launch.py \
    output_topic:=/robot/odometry
```

### Manual Node Execution

```bash
# Run node directly
ros2 run encoder_odometry encoder_odometry_node

# With custom parameters
ros2 run encoder_odometry encoder_odometry_node --ros-args \
    -p robot.wheel_diameter:=0.200 \
    -p robot.track_width:=0.350 \
    -p publishing.publish_rate:=100.0
```

### High-Speed Configuration

```bash
# Use high-performance config for fast robots
ros2 launch encoder_odometry encoder_odometry.launch.py \
    config_file:=$(ros2 pkg prefix encoder_odometry)/share/encoder_odometry/config/encoder_params_highspeed.yaml
```

## Configuration

### Robot Parameters

```yaml
robot:
  wheel_diameter: 0.215      # Wheel diameter in meters
  track_width: 0.380         # Distance between left/right wheels
  wheelbase: 0.3645          # Distance between front/rear axles
  encoder_ppr: 537.7         # Pulses per revolution
  drive_mode: "4wd"          # "4wd" or "2wd" kinematics
```

### GPIO Pin Configuration

```yaml
pins:
  front_left:
    a: 17    # Channel A GPIO pin (BCM numbering)
    b: 18    # Channel B GPIO pin
  front_right:
    a: 27
    b: 22
  # ... (rear wheels)
```

### Direction Inversion

```yaml
invert:
  front_left: false          # Normal direction
  front_right: true          # Inverted (typical for right wheels)
  rear_left: false           # Normal direction  
  rear_right: true           # Inverted (typical for right wheels)
```

### Publishing Settings

```yaml
publishing:
  publish_rate: 50.0         # Odometry update rate (Hz)
  publish_tf: true           # Publish TF transforms
  publish_raw_ticks: true    # Publish raw encoder data
  publish_diagnostics: true  # Periodic diagnostic output
```

## Topics

### Published Topics

- **`/odometry/encoders`** (`nav_msgs/Odometry`): Main odometry output
  - Position (x, y, z=0)
  - Orientation (quaternion from yaw angle)
  - Linear and angular velocities
  - Covariance estimates

- **`/encoders/ticks`** (`encoder_odometry/EncoderTicks`): Raw encoder data (optional)
  - Accumulated tick counts for each wheel
  - Tick deltas since last update
  - Calculated distances for each wheel

- **`/tf`** (`tf2_msgs/TFMessage`): Transform from odom_frame to base_frame (optional)

### Services

- **`~/reset_odometry`** (`std_srvs/Empty`): Reset odometry to origin

## Coordinate System

### Drive Modes

#### 4WD Mode (Default)
- Uses all 4 wheels for odometry calculation
- Averages left/right sides independently
- More robust to individual wheel slip
- Better accuracy for skid-steer robots

#### 2WD Mode
- Uses only rear wheels (typical for car-like robots)
- Front wheels assumed to be passive/steering
- Simpler calculation, lower computational load

### Coordinate Conventions
- **X-axis**: Forward (positive = robot moves forward)
- **Y-axis**: Left (positive = robot moves left)
- **Z-axis**: Up (always 0 for ground robots)
- **Yaw**: Counter-clockwise rotation (right-hand rule)

## Calibration and Testing

### Direction Calibration

1. **Test Individual Wheels**:
   ```bash
   # Monitor raw encoder ticks
   ros2 topic echo /encoders/ticks
   
   # Manually rotate each wheel forward
   # Verify tick counts increase for forward motion
   ```

2. **Adjust Direction Inversion**:
   ```yaml
   # If wheel rotates forward but ticks decrease, set invert: true
   invert:
     front_left: true   # Invert if needed
   ```

3. **Test Robot Motion**:
   ```bash
   # Drive robot forward in straight line
   ros2 topic echo /odometry/encoders --field pose.pose.position
   
   # X should increase, Y should stay near 0
   ```

### Encoder Resolution Verification

```bash
# Measure wheel circumference: C = π × diameter
# For 215mm wheel: C = 3.14159 × 0.215 = 0.6755m

# Calculate expected ticks per meter:
# Ticks/meter = PPR / circumference = 537.7 / 0.6755 = 795.7 ticks/meter

# Push robot exactly 1 meter, verify tick count matches
```

### Wheel Diameter Calibration

```bash
# Mark a position, drive robot forward 10 meters
# Measure actual distance traveled
# Adjust wheel_diameter parameter proportionally

# If robot travels 9.8m when commanded 10m:
# new_diameter = old_diameter × (9.8 / 10.0)
```

## Troubleshooting

### Common Issues

1. **No Encoder Signals**
   ```bash
   # Check GPIO permissions
   groups $USER  # Should include 'gpio'
   
   # Test GPIO directly
   sudo cat /sys/kernel/debug/gpio
   
   # Verify wiring - use multimeter to check signal levels
   ```

2. **Incorrect Direction**
   ```bash
   # Check encoder wiring (A/B channels might be swapped)
   # Adjust invert parameters in config
   # Verify motor rotation direction
   ```

3. **Noisy/Erratic Readings**
   ```bash
   # Check signal quality with oscilloscope
   # Increase debounce_time parameter
   # Add pull-up/pull-down resistors
   # Check for electromagnetic interference
   ```

4. **High-Speed Pulse Loss**
   ```bash
   # Reduce publish_rate to lower CPU load
   # Use pull-up resistors for better signal quality
   # Consider hardware-based encoder counting
   # Monitor CPU usage: htop
   ```

5. **Simulation Mode Warning**
   ```bash
   # If seeing "Running in simulation mode":
   sudo pip3 install Jetson.GPIO
   # Ensure running on actual Jetson hardware
   ```

### Performance Optimization

1. **High-Speed Operation**
   ```yaml
   # Use high-speed configuration
   publishing:
     publish_rate: 100.0
     publish_raw_ticks: false    # Reduce overhead
     publish_diagnostics: false
   
   advanced:
     debounce_time: 0.0005      # Faster response
     gpio_pull_mode: "up"       # Better signal integrity
   ```

2. **CPU Usage Optimization**
   ```bash
   # Set CPU governor to performance
   sudo cpufreq-set -g performance
   
   # Set process priority
   sudo chrt -f 50 ros2 run encoder_odometry encoder_odometry_node
   ```

3. **Real-Time Performance**
   ```bash
   # Install RT kernel (if available)
   # Use DDS settings for real-time
   export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
   ```

### Diagnostic Commands

```bash
# Monitor encoder status
ros2 topic echo /encoders/ticks

# Check odometry output
ros2 topic echo /odometry/encoders

# Monitor update rates
ros2 topic hz /odometry/encoders

# View TF tree
ros2 run tf2_tools view_frames

# Reset odometry
ros2 service call /encoder_odometry/reset_odometry std_srvs/srv/Empty

# Check node parameters
ros2 param list /encoder_odometry
ros2 param get /encoder_odometry robot.wheel_diameter
```

## Integration with Navigation Stack

### robot_localization Integration

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    
    odom0: /odometry/encoders
    odom0_config: [true,  true,  false,   # x, y, z
                   false, false, true,    # roll, pitch, yaw
                   true,  false, false,   # vx, vy, vz
                   false, false, true,    # vroll, vpitch, vyaw
                   false, false, false]   # ax, ay, az
```

### Navigation2 Integration

```yaml
# In nav2_params.yaml
controller_server:
  ros__parameters:
    odom_topic: "/odometry/encoders"

local_costmap:
  ros__parameters:
    robot_base_frame: "base_link"
    global_frame: "odom_encoders"
```

## Limitations and Considerations

### Accuracy Limitations

1. **Wheel Slip**: Encoder odometry assumes no wheel slippage
2. **Uneven Terrain**: Performance degrades on slopes or rough surfaces
3. **Mechanical Play**: Backlash in drivetrain affects accuracy
4. **Tire Pressure**: Under-inflated tires change effective wheel diameter

### Performance Limitations

1. **Python Overhead**: May lose pulses at very high speeds (>5000 RPM)
2. **Interrupt Latency**: GPIO interrupt response time varies
3. **CPU Load**: High encoder rates can impact system performance

### Alternative Solutions

For applications requiring higher performance:

1. **Hardware Counters**: Use dedicated encoder interface boards
2. **C++ Implementation**: Rewrite critical sections in C++
3. **FPGA Solutions**: Hardware-based quadrature decoding
4. **Real-Time OS**: Use RT kernel for deterministic performance

## Example Applications

### Warehouse Robot

```yaml
# Low-speed, high-accuracy configuration
robot:
  wheel_diameter: 0.200
  track_width: 0.500
  drive_mode: "4wd"

publishing:
  publish_rate: 30.0

advanced:
  velocity_filter_alpha: 0.05  # Heavy filtering for smooth motion
```

### Racing Robot

```yaml
# High-speed configuration
publishing:
  publish_rate: 100.0
  publish_raw_ticks: false

advanced:
  debounce_time: 0.0005
  velocity_filter_alpha: 0.5   # Responsive filtering
```

### Outdoor Robot

```yaml
# Robust configuration for rough terrain
advanced:
  gpio_pull_mode: "up"
  debounce_time: 0.002         # Longer debounce for noise
  velocity_filter_alpha: 0.2   # Moderate filtering
```

## License

This package is licensed under the MIT License. See LICENSE file for details.

## Contributing

Contributions are welcome! Please see CONTRIBUTING.md for guidelines.

## Support

For issues and questions:
1. Check this README thoroughly
2. Verify hardware connections and signal levels
3. Test with provided diagnostic commands
4. Create GitHub issues with detailed hardware info and log outputs

## Changelog

### Version 1.0.0
- Initial release with 4WD/2WD support
- Interrupt-based quadrature decoding
- Standard ROS 2 integration
- Comprehensive configuration options
- Detailed hardware documentation

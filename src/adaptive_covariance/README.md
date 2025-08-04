# Adaptive Covariance Package

**Intelligent sensor health monitoring and dynamic EKF covariance adjustment for robust multi-sensor fusion in ROS 2**

## Overview

The Adaptive Covariance package provides a comprehensive solution for monitoring sensor health and dynamically adjusting Extended Kalman Filter (EKF) parameters in real-time. This system significantly improves the robustness and reliability of multi-sensor odometry fusion by automatically adapting to changing sensor conditions.

### Key Features

- **Real-time Sensor Health Monitoring**: Continuous assessment of GPS, IMU, LIDAR, and encoder sensors
- **Intelligent Covariance Adjustment**: Dynamic EKF parameter tuning based on sensor health heuristics
- **Professional-grade Diagnostics**: Comprehensive logging and monitoring capabilities
- **Safety-first Design**: Conservative parameter limits and emergency fallback mechanisms
- **Research-ready Logging**: Detailed performance metrics for analysis and comparison

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                           ADAPTIVE COVARIANCE SYSTEM                            │
├─────────────────────┬─────────────────────┬─────────────────────────────────────┤
│  Sensor Health      │  Adaptive           │  EKF Covariance                     │
│  Monitor            │  Covariance Node    │  Adapter                            │
│                     │                     │                                     │
│  • Individual       │  • Health           │  • robot_localization              │
│    sensor analysis  │    coordination     │    parameter interface              │
│  • Trend detection  │  • Heuristic        │  • Safe parameter                   │
│  • Alert generation │    evaluation       │    adjustment                       │
│  • Performance      │  • Covariance       │  • Rollback capability             │
│    baselines        │    factor calc      │  • Change rate limiting            │
└─────────────────────┴─────────────────────┴─────────────────────────────────────┘
            │                       │                           │
            ▼                       ▼                           ▼
┌─────────────────────┬─────────────────────┬─────────────────────────────────────┐
│   Sensor Inputs     │    Health Topics    │     EKF Parameters                  │
├─────────────────────┼─────────────────────┼─────────────────────────────────────┤
│ /mavros/gps/*       │ /sensor_health/*    │ /robot_localization/               │
│ /mavros/imu/*       │ /adaptive_cov/*     │ ekf_filter_node/set_parameters     │
│ /odometry/lidar     │ /diagnostics        │                                     │
│ /odometry/encoders  │                     │                                     │
│ /encoders/ticks     │                     │                                     │
└─────────────────────┴─────────────────────┴─────────────────────────────────────┘
```

## Components

### 1. Sensor Health Monitor (`sensor_health_monitor.py`)

Provides detailed, sensor-specific health analysis:

**GPS Health Monitoring:**
- Fix quality assessment (NO_FIX, FIX, RTK_FIX)
- HDOP (Horizontal Dilution of Precision) evaluation
- Position covariance analysis
- Satellite count validation

**IMU Health Monitoring:**
- Acceleration magnitude validation (~9.8 m/s² when stationary)
- Gyroscope stability assessment
- Orientation covariance tracking
- Bias drift detection

**LIDAR Health Monitoring:**
- Point cloud density analysis
- Registration quality assessment
- Position covariance evaluation
- Feature tracking performance

**Encoder Health Monitoring:**
- Wheel slip detection and quantification
- Tick rate consistency analysis
- Cross-wheel synchronization validation
- Velocity reasonableness checks

### 2. Adaptive Covariance Node (`adaptive_covariance_node.py`)

Central coordinator that implements the core heuristic logic:

**Health Assessment Heuristics:**
```python
if fix_status == RTK_FIX and hdop < 0.8:
    return SensorHealth.EXCELLENT  # σ factor: 0.5
elif fix_status == FIX and hdop < 1.5:
    return SensorHealth.GOOD       # σ factor: 1.0
elif position_covariance > 100.0:
    return SensorHealth.FAILED     # σ factor: 100.0
```

**Dynamic Covariance Factors:**
- **EXCELLENT**: 0.5-0.8× baseline covariance (high confidence)
- **GOOD**: 1.0× baseline covariance (normal operation)
- **DEGRADED**: 1.5-3.0× baseline covariance (reduced confidence)
- **POOR**: 3.0-10.0× baseline covariance (low confidence)
- **FAILED**: 100.0× baseline covariance (effectively disabled)

### 3. EKF Covariance Adapter (`ekf_covariance_adapter.py`)

Safe interface to `robot_localization` EKF parameters:

**Safety Features:**
- Parameter change rate limiting (max 2× change per second)
- Value bounds enforcement (0.1× to 10× baseline)
- Automatic rollback on EKF divergence
- Emergency baseline reset capability

**Parameter Mapping Examples:**
```yaml
# GPS affects position process noise
process_noise_covariance.0: 0.05  # x position
process_noise_covariance.7: 0.05  # y position

# IMU affects orientation process noise  
process_noise_covariance.21: 0.03 # roll
process_noise_covariance.35: 0.06 # yaw

# Encoders affect velocity process noise
process_noise_covariance.42: 0.1  # x velocity
process_noise_covariance.77: 0.2  # yaw velocity
```

## Configuration

### Sensor Health Thresholds

Configure health assessment criteria in `config/sensor_health_thresholds.yaml`:

```yaml
gps:
  hdop_excellent: 0.8      # RTK-level accuracy
  hdop_good: 1.5           # Standard GPS accuracy
  hdop_poor: 5.0           # Poor GPS geometry
  fix_status_min: 1        # Minimum required fix
  
imu:
  accel_excellent_min: 8.0 # Stationary acceleration bounds
  accel_excellent_max: 11.0
  gyro_excellent_max: 0.5  # Stable gyroscope readings
  
encoders:
  slip_excellent: 0.02     # 2% wheel slip threshold
  slip_poor: 0.2           # 20% slip threshold
```

### EKF Parameter Mapping

Define sensor-to-EKF parameter relationships in `config/ekf_parameter_mapping.yaml`:

```yaml
ekf_parameter_mappings:
  - sensor_name: "gps"
    ekf_parameter: "process_noise_covariance.0"
    baseline_value: 0.05
    min_value: 0.005
    max_value: 5.0
```

## Usage

### Basic Launch

Launch the complete adaptive covariance system:

```bash
# Launch with default settings
ros2 launch adaptive_covariance adaptive_covariance.launch.py

# Launch with custom EKF namespace
ros2 launch adaptive_covariance adaptive_covariance.launch.py \
    ekf_namespace:=my_robot_localization

# Launch in monitoring-only mode (no EKF adjustment)
ros2 launch adaptive_covariance adaptive_covariance.launch.py \
    enable_ekf_adjustment:=false
```

### Integration with Existing Odometry System

Add to your existing launch file:

```python
# In your main launch file
adaptive_covariance_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        get_package_share_directory('adaptive_covariance'),
        '/launch/adaptive_covariance.launch.py'
    ]),
    launch_arguments={
        'ekf_namespace': 'robot_localization',
        'enable_ekf_adjustment': 'true'
    }.items()
)
```

### Individual Node Launch

Launch components separately for testing:

```bash
# Sensor health monitoring only
ros2 run adaptive_covariance sensor_health_monitor

# EKF adapter only
ros2 run adaptive_covariance ekf_covariance_adapter

# Main coordination node only  
ros2 run adaptive_covariance adaptive_covariance_node
```

## Monitoring and Diagnostics

### Real-time Health Status

Monitor sensor health in real-time:

```bash
# Overall sensor health summary
ros2 topic echo /adaptive_covariance/sensor_health

# Detailed diagnostic information
ros2 topic echo /diagnostics

# Individual sensor status
ros2 topic echo /sensor_health/gps/ok
ros2 topic echo /sensor_health/imu/ok
ros2 topic echo /sensor_health/lidar/ok
ros2 topic echo /sensor_health/encoders/ok
```

### Parameter Adjustment Logging

Track EKF parameter changes:

```bash
# Parameter change log
ros2 topic echo /ekf_adapter/parameter_changes

# Current covariance values
ros2 topic echo /ekf_adapter/current_covariances

# Parameter status
ros2 topic echo /ekf_adapter/parameter_status
```

### Health Alerts

Monitor sensor health alerts:

```bash
# Real-time health alerts
ros2 topic echo /sensor_health/alerts

# Performance metrics
ros2 topic echo /sensor_health/performance
```

## Heuristic Rules Implementation

### GPS Heuristics

```python
def evaluate_gps_health(self, metrics, thresholds):
    fix_status = metrics.get('fix_status', -1)
    hdop = metrics.get('hdop', float('inf'))
    
    # RTK fix with excellent geometry
    if fix_status == 2 and hdop < 0.8:
        return SensorHealth.EXCELLENT
    
    # Standard fix with good geometry
    if fix_status == 1 and hdop < 1.5:
        return SensorHealth.GOOD
    
    # Poor geometry or no fix
    if fix_status <= 0 or hdop > 5.0:
        return SensorHealth.FAILED
    
    return SensorHealth.DEGRADED
```

### IMU Heuristics

```python
def evaluate_imu_health(self, metrics, thresholds):
    accel_mag = metrics.get('accel_magnitude', 0.0)
    gyro_mag = metrics.get('gyro_magnitude', 0.0)
    
    # Reasonable acceleration + stable gyroscope
    if 8.0 < accel_mag < 11.0 and gyro_mag < 0.5:
        return SensorHealth.EXCELLENT
    
    # Excessive rotation indicates poor conditions
    if gyro_mag > 10.0:
        return SensorHealth.POOR
    
    return SensorHealth.GOOD
```

### Encoder Heuristics

```python
def evaluate_encoder_health(self, metrics, thresholds):
    slip_ratio = metrics.get('slip_ratio', 0.0)
    
    # Excellent grip conditions
    if slip_ratio < 0.02:
        return SensorHealth.EXCELLENT
    
    # Significant wheel slip
    if slip_ratio > 0.2:
        return SensorHealth.POOR
    
    return SensorHealth.GOOD
```

## Performance Impact Analysis

### Comparative Testing Framework

The system provides built-in comparison capabilities:

```bash
# Log static vs adaptive performance
ros2 topic echo /adaptive_covariance/performance_comparison

# Export data for analysis
ros2 bag record /adaptive_covariance/adjustments \
                /ekf_adapter/parameter_changes \
                /robot_localization/odometry/filtered
```

### Metrics for Evaluation

**Quantitative Metrics:**
- Position estimation accuracy vs ground truth
- Velocity estimation consistency
- Heading estimation stability
- Covariance prediction accuracy

**Qualitative Metrics:**  
- System responsiveness to sensor degradation
- Recovery time from sensor failures
- False positive rate for health assessments

## Safety and Fault Tolerance

### Conservative Parameter Limits

- **Minimum adjustment factor**: 0.1× (never trust sensor completely)
- **Maximum adjustment factor**: 10.0× (never completely ignore sensor)
- **Rate limiting**: Maximum 2× change per second
- **Validation**: Parameter bounds checking before application

### Emergency Procedures

```python
# Automatic emergency reset triggers
if sensor_failure_rate > 0.5:
    self.emergency_reset_to_baseline()

if ekf_position_uncertainty > 100.0:
    self.disable_failed_sensors()
    self.increase_process_noise_globally()
```

### Rollback Capability

- Automatic parameter rollback on EKF divergence detection
- Manual reset to baseline values via service calls
- Parameter change history for forensic analysis

## Integration with robot_localization

### Required EKF Configuration

Your `robot_localization` configuration should include:

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    
    # Enable parameter services
    enable_parameter_services: true
    
    # Sensor inputs (example)
    odom0: /odometry/lidar
    odom0_config: [true, true, false, false, false, true, ...]
    
    imu0: /mavros/imu/data  
    imu0_config: [false, false, false, true, true, true, ...]
    
    # Process noise covariance (will be adjusted dynamically)
    process_noise_covariance: [0.05, 0, 0, ...]
```

### Service Interface

The system uses standard ROS 2 parameter services:

```bash
# Check available EKF parameters
ros2 service call /robot_localization/ekf_filter_node/list_parameters \
    rcl_interfaces/srv/ListParameters

# Manual parameter adjustment (for testing)
ros2 service call /robot_localization/ekf_filter_node/set_parameters \
    rcl_interfaces/srv/SetParameters \
    "{parameters: [{name: 'process_noise_covariance.0', value: {type: 3, double_value: 0.1}}]}"
```

## Troubleshooting

### Common Issues

**1. EKF Service Connection Failure**
```bash
# Check if robot_localization is running
ros2 node list | grep ekf

# Verify service availability
ros2 service list | grep robot_localization

# Check parameter service specifically
ros2 service info /robot_localization/ekf_filter_node/set_parameters
```

**2. Sensor Topic Connection Issues**
```bash
# Verify sensor topics are publishing
ros2 topic hz /mavros/global_position/raw/fix
ros2 topic hz /mavros/imu/data
ros2 topic hz /odometry/lidar
ros2 topic hz /odometry/encoders

# Check topic data quality
ros2 topic echo /mavros/global_position/raw/fix --field status.status
```

**3. Parameter Adjustment Not Working**
```bash
# Check if EKF adjustment is enabled
ros2 param get /adaptive_covariance_node enable_ekf_adjustment

# Monitor parameter changes
ros2 topic echo /ekf_adapter/parameter_changes

# Check for error messages
ros2 log view | grep adaptive_covariance
```

### Debug Mode

Enable detailed logging for troubleshooting:

```bash
ros2 launch adaptive_covariance adaptive_covariance.launch.py \
    log_level:=debug
```

### Simulation Mode

Test without hardware sensors:

```bash
# Use simulation time
ros2 launch adaptive_covariance adaptive_covariance.launch.py \
    use_sim_time:=true \
    enable_ekf_adjustment:=false  # Monitor only in simulation
```

## Research and Development

### Data Collection for Analysis

```bash
# Record comprehensive dataset
ros2 bag record \
    /adaptive_covariance/sensor_health \
    /adaptive_covariance/adjustments \
    /ekf_adapter/parameter_changes \
    /robot_localization/odometry/filtered \
    /mavros/global_position/raw/fix \
    /mavros/imu/data \
    /odometry/lidar \
    /odometry/encoders \
    /diagnostics
```

### Performance Metrics Extraction

```python
# Example analysis script
import rosbag2_py
import yaml

def analyze_adaptive_performance(bag_path):
    # Extract parameter changes and odometry accuracy
    # Compare static vs adaptive covariance performance
    # Generate performance reports
    pass
```

### Custom Heuristic Development

Extend the system with custom heuristics:

```python
# In adaptive_covariance_node.py
def _evaluate_custom_sensor_health(self, sensor_name, metrics, thresholds):
    """Implement custom health evaluation logic"""
    
    # Your custom heuristic implementation
    if custom_condition_excellent(metrics):
        return SensorHealth.EXCELLENT
    elif custom_condition_degraded(metrics):
        return SensorHealth.DEGRADED
    
    return SensorHealth.GOOD
```

## Contributing

This package is designed for research and industrial applications. Contributions are welcome:

1. **Sensor Support**: Add support for additional sensor types
2. **Heuristics**: Implement improved health assessment algorithms  
3. **EKF Integration**: Extend support for other localization frameworks
4. **Performance**: Optimize real-time performance for high-frequency applications

## License

MIT License - See LICENSE file for details.

## Acknowledgments

- **robot_localization**: Tom Moore and the ROS Navigation Stack team
- **Research Foundation**: Based on adaptive filtering and sensor fusion research
- **Industrial Validation**: Tested in real-world robotic applications

## Citation

If you use this package in research, please cite:

```bibtex
@software{adaptive_covariance_ros2,
  title={Adaptive Covariance for Robust Multi-Sensor Fusion in ROS 2},
  author={fjgarco},
  year={2025},
  url={https://github.com/fjgarco/odometry_ws}
}
```

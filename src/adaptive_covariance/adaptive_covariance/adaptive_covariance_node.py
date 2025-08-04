#!/usr/bin/env python3
"""
Adaptive Covariance Node - Main coordinator for sensor health monitoring and EKF adaptation

This node orchestrates the entire adaptive covariance system by:
1. Coordinating sensor health monitoring
2. Computing adaptive covariance adjustments
3. Interfacing with robot_localization EKF
4. Logging performance metrics and comparisons

Author: fjgarco
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import yaml
from typing import Dict, List, Any, Optional
from dataclasses import dataclass, field
from enum import Enum
import threading
import time

# ROS 2 messages
from std_msgs.msg import Header, Float64MultiArray, String
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from sensor_msgs.msg import NavSatFix, Imu, MagneticField, PointCloud2
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.srv import SetParameters, GetParameters

# Custom messages
from encoder_odometry.msg import EncoderTicks


class SensorHealth(Enum):
    """Sensor health status enumeration"""
    EXCELLENT = 0    # σ factor: 0.5-0.8
    GOOD = 1        # σ factor: 0.8-1.0  
    DEGRADED = 2    # σ factor: 1.5-3.0
    POOR = 3        # σ factor: 3.0-10.0
    FAILED = 4      # σ factor: 100.0 (effectively disable)


@dataclass
class SensorMetrics:
    """Container for sensor health metrics"""
    health_status: SensorHealth = SensorHealth.GOOD
    data_rate: float = 0.0  # Hz
    expected_rate: float = 30.0  # Expected Hz
    last_update: float = 0.0  # timestamp
    message_count: int = 0
    error_count: int = 0
    quality_indicators: Dict[str, float] = field(default_factory=dict)
    covariance_factor: float = 1.0  # Multiplier for baseline covariance


@dataclass
class CovarianceAdjustment:
    """Container for EKF covariance adjustments"""
    sensor_name: str
    original_covariance: List[float]
    adjusted_covariance: List[float]
    adjustment_factor: float
    timestamp: float
    reason: str


class AdaptiveCovarianceNode(Node):
    """
    Main adaptive covariance coordination node
    
    Responsibilities:
    - Coordinate sensor health monitoring
    - Compute covariance adjustments based on heuristics
    - Interface with robot_localization EKF
    - Log performance and provide diagnostics
    """
    
    def __init__(self):
        super().__init__('adaptive_covariance_node')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize parameters
        self._declare_parameters()
        
        # Sensor metrics storage
        self.sensor_metrics: Dict[str, SensorMetrics] = {}
        self.covariance_adjustments: List[CovarianceAdjustment] = []
        
        # Thread safety
        self.metrics_lock = threading.RLock()
        
        # Load configuration
        self._load_configuration()
        
        # Initialize subscribers for sensor monitoring
        self._setup_sensor_subscribers()
        
        # Initialize publishers for diagnostics and EKF communication
        self._setup_publishers()
        
        # Initialize services for EKF parameter adjustment
        self._setup_services()
        
        # Start monitoring timer
        self.monitoring_timer = self.create_timer(
            self.monitoring_period,
            self._monitor_sensor_health,
            callback_group=self.callback_group
        )
        
        # Start EKF adjustment timer
        self.adjustment_timer = self.create_timer(
            self.adjustment_period,
            self._adjust_ekf_covariances,
            callback_group=self.callback_group
        )
        
        self.get_logger().info("Adaptive Covariance Node initialized successfully")
        self.get_logger().info(f"Monitoring {len(self.sensor_configs)} sensors")
        
    def _declare_parameters(self):
        """Declare ROS 2 parameters with defaults"""
        
        # Core timing parameters
        self.declare_parameter('monitoring_period', 0.1,  # 10 Hz monitoring
                              ParameterDescriptor(description="Sensor health monitoring period (seconds)"))
        self.declare_parameter('adjustment_period', 0.2,  # 5 Hz EKF adjustment
                              ParameterDescriptor(description="EKF covariance adjustment period (seconds)"))
        
        # Configuration files
        self.declare_parameter('config_file', 'config/adaptive_covariance_params.yaml',
                              ParameterDescriptor(description="Main configuration file"))
        self.declare_parameter('thresholds_file', 'config/sensor_health_thresholds.yaml',
                              ParameterDescriptor(description="Sensor health thresholds file"))
        
        # EKF interface
        self.declare_parameter('ekf_namespace', 'robot_localization',
                              ParameterDescriptor(description="robot_localization namespace"))
        self.declare_parameter('enable_ekf_adjustment', True,
                              ParameterDescriptor(description="Enable real EKF parameter adjustment"))
        
        # Diagnostic settings
        self.declare_parameter('enable_diagnostics', True,
                              ParameterDescriptor(description="Enable diagnostic publishing"))
        self.declare_parameter('log_adjustments', True,
                              ParameterDescriptor(description="Log covariance adjustments"))
        
        # Get parameter values
        self.monitoring_period = self.get_parameter('monitoring_period').value
        self.adjustment_period = self.get_parameter('adjustment_period').value
        self.config_file = self.get_parameter('config_file').value
        self.thresholds_file = self.get_parameter('thresholds_file').value
        self.ekf_namespace = self.get_parameter('ekf_namespace').value
        self.enable_ekf_adjustment = self.get_parameter('enable_ekf_adjustment').value
        self.enable_diagnostics = self.get_parameter('enable_diagnostics').value
        self.log_adjustments = self.get_parameter('log_adjustments').value
        
    def _load_configuration(self):
        """Load configuration from YAML files"""
        try:
            # Load main configuration
            config_path = self.get_package_share_directory('adaptive_covariance') + '/' + self.config_file
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
            
            # Load sensor thresholds
            thresholds_path = self.get_package_share_directory('adaptive_covariance') + '/' + self.thresholds_file
            with open(thresholds_path, 'r') as f:
                self.thresholds = yaml.safe_load(f)
                
            # Extract sensor configurations
            self.sensor_configs = self.config.get('sensors', {})
            
            # Initialize sensor metrics
            for sensor_name in self.sensor_configs.keys():
                self.sensor_metrics[sensor_name] = SensorMetrics()
                
            self.get_logger().info(f"Loaded configuration for {len(self.sensor_configs)} sensors")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load configuration: {str(e)}")
            # Use default minimal configuration
            self._setup_default_config()
    
    def _setup_default_config(self):
        """Setup minimal default configuration if files are missing"""
        self.config = {
            'sensors': {
                'gps': {'topic': '/mavros/global_position/raw/fix', 'type': 'nav_sat_fix'},
                'imu': {'topic': '/mavros/imu/data', 'type': 'imu'},
                'lidar': {'topic': '/odometry/lidar', 'type': 'odometry'},
                'encoders': {'topic': '/odometry/encoders', 'type': 'odometry'}
            }
        }
        
        self.thresholds = {
            'gps': {'rate_min': 5.0, 'covariance_max': 25.0},
            'imu': {'rate_min': 50.0, 'accel_max': 50.0},
            'lidar': {'rate_min': 10.0, 'features_min': 100},
            'encoders': {'rate_min': 20.0, 'slip_threshold': 0.1}
        }
        
        self.sensor_configs = self.config['sensors']
        for sensor_name in self.sensor_configs.keys():
            self.sensor_metrics[sensor_name] = SensorMetrics()
    
    def _setup_sensor_subscribers(self):
        """Setup subscribers for each configured sensor"""
        self.subscribers = {}
        
        for sensor_name, config in self.sensor_configs.items():
            topic = config['topic']
            msg_type = config['type']
            
            if msg_type == 'nav_sat_fix':
                self.subscribers[sensor_name] = self.create_subscription(
                    NavSatFix, topic, 
                    lambda msg, name=sensor_name: self._gps_callback(msg, name),
                    10, callback_group=self.callback_group
                )
            elif msg_type == 'imu':
                self.subscribers[sensor_name] = self.create_subscription(
                    Imu, topic,
                    lambda msg, name=sensor_name: self._imu_callback(msg, name),
                    10, callback_group=self.callback_group
                )
            elif msg_type == 'odometry':
                self.subscribers[sensor_name] = self.create_subscription(
                    Odometry, topic,
                    lambda msg, name=sensor_name: self._odometry_callback(msg, name),
                    10, callback_group=self.callback_group
                )
            elif msg_type == 'encoder_ticks':
                self.subscribers[sensor_name] = self.create_subscription(
                    EncoderTicks, topic,
                    lambda msg, name=sensor_name: self._encoder_ticks_callback(msg, name),
                    10, callback_group=self.callback_group
                )
            elif msg_type == 'point_cloud':
                self.subscribers[sensor_name] = self.create_subscription(
                    PointCloud2, topic,
                    lambda msg, name=sensor_name: self._lidar_callback(msg, name),
                    10, callback_group=self.callback_group
                )
            
            self.get_logger().info(f"Subscribed to {sensor_name}: {topic} ({msg_type})")
    
    def _setup_publishers(self):
        """Setup publishers for diagnostics and EKF communication"""
        
        # Diagnostic publisher
        if self.enable_diagnostics:
            self.diagnostics_pub = self.create_publisher(
                DiagnosticArray, '/diagnostics', 10
            )
        
        # Sensor health status publisher
        self.sensor_health_pub = self.create_publisher(
            String, '/adaptive_covariance/sensor_health', 10
        )
        
        # Covariance adjustment log publisher
        self.covariance_log_pub = self.create_publisher(
            String, '/adaptive_covariance/adjustments', 10
        )
        
        # EKF configuration publisher (for external monitoring)
        self.ekf_config_pub = self.create_publisher(
            Float64MultiArray, '/adaptive_covariance/ekf_covariances', 10
        )
        
    def _setup_services(self):
        """Setup service clients for EKF parameter adjustment"""
        if self.enable_ekf_adjustment:
            # Service client for robot_localization parameter setting
            self.ekf_param_client = self.create_client(
                SetParameters, 
                f'/{self.ekf_namespace}/ekf_filter_node/set_parameters'
            )
            
            # Wait for service availability
            if not self.ekf_param_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn("EKF parameter service not available - running in monitoring mode only")
                self.enable_ekf_adjustment = False
    
    def get_package_share_directory(self, package_name: str) -> str:
        """Get the share directory for a package"""
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(package_name)
    
    # Sensor callback methods
    def _gps_callback(self, msg: NavSatFix, sensor_name: str):
        """Handle GPS sensor messages"""
        with self.metrics_lock:
            metrics = self.sensor_metrics[sensor_name]
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            # Update basic metrics
            metrics.last_update = current_time
            metrics.message_count += 1
            
            # GPS-specific health indicators
            metrics.quality_indicators['fix_status'] = float(msg.status.status)
            metrics.quality_indicators['position_covariance'] = np.trace(np.array(msg.position_covariance).reshape(3, 3))
            
            # HDOP calculation (if available in covariance)
            if len(msg.position_covariance) >= 9:
                covar_matrix = np.array(msg.position_covariance).reshape(3, 3)
                hdop = np.sqrt(covar_matrix[0, 0] + covar_matrix[1, 1])
                metrics.quality_indicators['hdop'] = hdop
            
            # Update data rate
            self._update_data_rate(sensor_name, current_time)
    
    def _imu_callback(self, msg: Imu, sensor_name: str):
        """Handle IMU sensor messages"""
        with self.metrics_lock:
            metrics = self.sensor_metrics[sensor_name]
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            # Update basic metrics
            metrics.last_update = current_time
            metrics.message_count += 1
            
            # IMU-specific health indicators
            accel_magnitude = np.sqrt(
                msg.linear_acceleration.x**2 + 
                msg.linear_acceleration.y**2 + 
                msg.linear_acceleration.z**2
            )
            gyro_magnitude = np.sqrt(
                msg.angular_velocity.x**2 + 
                msg.angular_velocity.y**2 + 
                msg.angular_velocity.z**2
            )
            
            metrics.quality_indicators['accel_magnitude'] = accel_magnitude
            metrics.quality_indicators['gyro_magnitude'] = gyro_magnitude
            
            # Orientation covariance
            if len(msg.orientation_covariance) >= 9:
                orientation_cov = np.trace(np.array(msg.orientation_covariance).reshape(3, 3))
                metrics.quality_indicators['orientation_covariance'] = orientation_cov
            
            # Update data rate
            self._update_data_rate(sensor_name, current_time)
    
    def _odometry_callback(self, msg: Odometry, sensor_name: str):
        """Handle odometry messages (LIDAR, visual, encoder)"""
        with self.metrics_lock:
            metrics = self.sensor_metrics[sensor_name]
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            # Update basic metrics
            metrics.last_update = current_time
            metrics.message_count += 1
            
            # Odometry-specific health indicators
            position_cov = np.trace(np.array(msg.pose.covariance).reshape(6, 6)[:3, :3])
            velocity_cov = np.trace(np.array(msg.twist.covariance).reshape(6, 6)[:3, :3])
            
            metrics.quality_indicators['position_covariance'] = position_cov
            metrics.quality_indicators['velocity_covariance'] = velocity_cov
            
            # Velocity magnitude
            velocity_mag = np.sqrt(
                msg.twist.twist.linear.x**2 + 
                msg.twist.twist.linear.y**2 + 
                msg.twist.twist.linear.z**2
            )
            metrics.quality_indicators['velocity_magnitude'] = velocity_mag
            
            # Update data rate
            self._update_data_rate(sensor_name, current_time)
    
    def _encoder_ticks_callback(self, msg: EncoderTicks, sensor_name: str):
        """Handle encoder tick messages"""
        with self.metrics_lock:
            metrics = self.sensor_metrics[sensor_name]
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            # Update basic metrics
            metrics.last_update = current_time
            metrics.message_count += 1
            
            # Encoder-specific health indicators
            total_ticks = abs(msg.front_left) + abs(msg.front_right) + abs(msg.rear_left) + abs(msg.rear_right)
            metrics.quality_indicators['total_ticks'] = float(total_ticks)
            
            # Slip detection (differential between wheels)
            left_avg = (msg.front_left + msg.rear_left) / 2.0
            right_avg = (msg.front_right + msg.rear_right) / 2.0
            slip_indicator = abs(left_avg - right_avg) / max(abs(left_avg) + abs(right_avg), 1.0)
            metrics.quality_indicators['slip_indicator'] = slip_indicator
            
            # Update data rate
            self._update_data_rate(sensor_name, current_time)
    
    def _lidar_callback(self, msg: PointCloud2, sensor_name: str):
        """Handle LIDAR point cloud messages"""
        with self.metrics_lock:
            metrics = self.sensor_metrics[sensor_name]
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            # Update basic metrics
            metrics.last_update = current_time
            metrics.message_count += 1
            
            # LIDAR-specific health indicators
            point_count = msg.width * msg.height
            metrics.quality_indicators['point_count'] = float(point_count)
            metrics.quality_indicators['data_size'] = float(len(msg.data))
            
            # Update data rate
            self._update_data_rate(sensor_name, current_time)
    
    def _update_data_rate(self, sensor_name: str, current_time: float):
        """Update data rate calculation for a sensor"""
        metrics = self.sensor_metrics[sensor_name]
        
        # Simple moving average for data rate
        if hasattr(metrics, '_last_rate_update'):
            dt = current_time - metrics._last_rate_update
            if dt > 0:
                instantaneous_rate = 1.0 / dt
                # Exponential moving average with alpha = 0.1
                metrics.data_rate = 0.9 * metrics.data_rate + 0.1 * instantaneous_rate
        
        metrics._last_rate_update = current_time
    
    def _monitor_sensor_health(self):
        """Main sensor health monitoring loop"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        with self.metrics_lock:
            for sensor_name, metrics in self.sensor_metrics.items():
                # Evaluate sensor health based on heuristics
                health_status = self._evaluate_sensor_health(sensor_name, metrics, current_time)
                
                # Update health status
                metrics.health_status = health_status
                
                # Calculate covariance adjustment factor
                metrics.covariance_factor = self._calculate_covariance_factor(health_status, sensor_name, metrics)
        
        # Publish diagnostics
        if self.enable_diagnostics:
            self._publish_diagnostics()
        
        # Publish sensor health summary
        self._publish_sensor_health()
    
    def _evaluate_sensor_health(self, sensor_name: str, metrics: SensorMetrics, current_time: float) -> SensorHealth:
        """Evaluate sensor health based on configured heuristics"""
        
        # Check if sensor is receiving data
        time_since_update = current_time - metrics.last_update
        if time_since_update > 2.0:  # No data for 2 seconds
            return SensorHealth.FAILED
        
        # Get sensor-specific thresholds
        thresholds = self.thresholds.get(sensor_name, {})
        
        # Data rate check
        min_rate = thresholds.get('rate_min', 1.0)
        if metrics.data_rate < min_rate:
            return SensorHealth.POOR
        
        # Sensor-specific health evaluation
        if sensor_name in ['gps']:
            return self._evaluate_gps_health(metrics, thresholds)
        elif sensor_name in ['imu']:
            return self._evaluate_imu_health(metrics, thresholds)
        elif sensor_name in ['lidar']:
            return self._evaluate_lidar_health(metrics, thresholds)
        elif sensor_name in ['encoders']:
            return self._evaluate_encoder_health(metrics, thresholds)
        else:
            # Generic odometry evaluation
            return self._evaluate_odometry_health(metrics, thresholds)
    
    def _evaluate_gps_health(self, metrics: SensorMetrics, thresholds: Dict) -> SensorHealth:
        """GPS-specific health evaluation"""
        fix_status = metrics.quality_indicators.get('fix_status', -1)
        position_cov = metrics.quality_indicators.get('position_covariance', float('inf'))
        hdop = metrics.quality_indicators.get('hdop', float('inf'))
        
        # RTK fix is excellent
        if fix_status == 2 and hdop < 0.8:  # RTK_FIX
            return SensorHealth.EXCELLENT
        
        # Regular fix with good HDOP
        if fix_status == 1 and hdop < 1.5:  # FIX
            return SensorHealth.GOOD
        
        # Fix but degraded accuracy
        if fix_status == 1 and position_cov < thresholds.get('covariance_max', 25.0):
            return SensorHealth.DEGRADED
        
        # No fix or very poor accuracy
        if fix_status <= 0 or position_cov > 100.0:
            return SensorHealth.FAILED
        
        return SensorHealth.POOR
    
    def _evaluate_imu_health(self, metrics: SensorMetrics, thresholds: Dict) -> SensorHealth:
        """IMU-specific health evaluation"""
        accel_mag = metrics.quality_indicators.get('accel_magnitude', 0.0)
        gyro_mag = metrics.quality_indicators.get('gyro_magnitude', 0.0)
        orientation_cov = metrics.quality_indicators.get('orientation_covariance', float('inf'))
        
        # Check for reasonable acceleration (around 9.8 m/s² when stationary)
        accel_reasonable = 5.0 < accel_mag < 20.0
        
        # Check for excessive rotation
        gyro_excessive = gyro_mag > thresholds.get('gyro_max', 10.0)
        
        # High frequency and reasonable values
        if accel_reasonable and not gyro_excessive and metrics.data_rate > 100.0:
            return SensorHealth.EXCELLENT
        
        # Reasonable values but lower frequency
        if accel_reasonable and not gyro_excessive:
            return SensorHealth.GOOD
        
        # Some issues but still usable
        if accel_mag > 0.1 and gyro_mag < 50.0:
            return SensorHealth.DEGRADED
        
        # Very poor or no data
        if accel_mag < 0.1 or gyro_mag > 100.0:
            return SensorHealth.FAILED
        
        return SensorHealth.POOR
    
    def _evaluate_lidar_health(self, metrics: SensorMetrics, thresholds: Dict) -> SensorHealth:
        """LIDAR-specific health evaluation"""
        point_count = metrics.quality_indicators.get('point_count', 0)
        
        # High point density
        if point_count > thresholds.get('points_excellent', 10000):
            return SensorHealth.EXCELLENT
        
        # Good point density
        if point_count > thresholds.get('points_good', 5000):
            return SensorHealth.GOOD
        
        # Minimum usable points
        if point_count > thresholds.get('points_min', 1000):
            return SensorHealth.DEGRADED
        
        # Very few points
        if point_count > 100:
            return SensorHealth.POOR
        
        return SensorHealth.FAILED
    
    def _evaluate_encoder_health(self, metrics: SensorMetrics, thresholds: Dict) -> SensorHealth:
        """Encoder-specific health evaluation"""
        slip_indicator = metrics.quality_indicators.get('slip_indicator', 0.0)
        total_ticks = metrics.quality_indicators.get('total_ticks', 0.0)
        
        # Low slip and good data rate
        if slip_indicator < 0.05 and metrics.data_rate > 50.0:
            return SensorHealth.EXCELLENT
        
        # Acceptable slip
        if slip_indicator < thresholds.get('slip_threshold', 0.1):
            return SensorHealth.GOOD
        
        # High slip but still getting data
        if slip_indicator < 0.3 and total_ticks > 0:
            return SensorHealth.DEGRADED
        
        # Very high slip
        if slip_indicator < 0.5:
            return SensorHealth.POOR
        
        return SensorHealth.FAILED
    
    def _evaluate_odometry_health(self, metrics: SensorMetrics, thresholds: Dict) -> SensorHealth:
        """Generic odometry health evaluation"""
        position_cov = metrics.quality_indicators.get('position_covariance', float('inf'))
        velocity_cov = metrics.quality_indicators.get('velocity_covariance', float('inf'))
        
        # Low covariance and good rate
        if position_cov < 0.1 and velocity_cov < 0.01 and metrics.data_rate > 20.0:
            return SensorHealth.EXCELLENT
        
        # Reasonable covariance
        if position_cov < 1.0 and velocity_cov < 0.1:
            return SensorHealth.GOOD
        
        # Higher covariance but usable
        if position_cov < 10.0 and velocity_cov < 1.0:
            return SensorHealth.DEGRADED
        
        # Very high covariance
        if position_cov < 100.0:
            return SensorHealth.POOR
        
        return SensorHealth.FAILED
    
    def _calculate_covariance_factor(self, health_status: SensorHealth, sensor_name: str, metrics: SensorMetrics) -> float:
        """Calculate covariance adjustment factor based on health status"""
        
        # Base factors for each health level
        base_factors = {
            SensorHealth.EXCELLENT: 0.5,
            SensorHealth.GOOD: 1.0,
            SensorHealth.DEGRADED: 2.0,
            SensorHealth.POOR: 5.0,
            SensorHealth.FAILED: 100.0
        }
        
        base_factor = base_factors[health_status]
        
        # Additional adjustments based on specific metrics
        sensor_config = self.sensor_configs.get(sensor_name, {})
        adjustment_rules = sensor_config.get('adjustment_rules', {})
        
        # Apply sensor-specific rules
        if sensor_name == 'gps':
            hdop = metrics.quality_indicators.get('hdop', 1.0)
            if hdop > 2.0:
                base_factor *= min(hdop, 10.0)
        
        elif sensor_name == 'imu':
            gyro_mag = metrics.quality_indicators.get('gyro_magnitude', 0.0)
            if gyro_mag > 5.0:  # High rotation
                base_factor *= min(1.0 + gyro_mag / 10.0, 5.0)
        
        elif sensor_name == 'encoders':
            slip = metrics.quality_indicators.get('slip_indicator', 0.0)
            if slip > 0.1:
                base_factor *= min(1.0 + slip * 10.0, 10.0)
        
        # Data rate adjustment
        expected_rate = metrics.expected_rate
        if metrics.data_rate < expected_rate * 0.5:
            rate_factor = expected_rate / max(metrics.data_rate, 0.1)
            base_factor *= min(rate_factor, 5.0)
        
        return base_factor
    
    def _adjust_ekf_covariances(self):
        """Adjust EKF covariances based on current sensor health"""
        if not self.enable_ekf_adjustment:
            return
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        adjustments_made = []
        
        with self.metrics_lock:
            for sensor_name, metrics in self.sensor_metrics.items():
                # Get EKF parameter names for this sensor
                ekf_params = self._get_ekf_parameters_for_sensor(sensor_name)
                
                for param_name, base_value in ekf_params.items():
                    # Calculate adjusted value
                    adjusted_value = base_value * metrics.covariance_factor
                    
                    # Create adjustment record
                    adjustment = CovarianceAdjustment(
                        sensor_name=sensor_name,
                        original_covariance=[base_value],
                        adjusted_covariance=[adjusted_value],
                        adjustment_factor=metrics.covariance_factor,
                        timestamp=current_time,
                        reason=f"Health: {metrics.health_status.name}, Factor: {metrics.covariance_factor:.2f}"
                    )
                    
                    adjustments_made.append(adjustment)
                    
                    # Apply to EKF (if service is available)
                    if self.ekf_param_client.service_is_ready():
                        self._set_ekf_parameter(param_name, adjusted_value)
        
        # Store adjustments
        self.covariance_adjustments.extend(adjustments_made)
        
        # Publish adjustment logs
        if self.log_adjustments and adjustments_made:
            self._publish_adjustment_log(adjustments_made)
    
    def _get_ekf_parameters_for_sensor(self, sensor_name: str) -> Dict[str, float]:
        """Get EKF parameter names and base values for a sensor"""
        
        # This mapping depends on your robot_localization configuration
        # These are example parameter names - adjust for your setup
        param_mappings = {
            'gps': {
                'process_noise_covariance.0': 0.05,   # x position
                'process_noise_covariance.7': 0.05,   # y position
                'process_noise_covariance.14': 0.06,  # z position
            },
            'imu': {
                'process_noise_covariance.21': 0.03,  # roll
                'process_noise_covariance.28': 0.03,  # pitch
                'process_noise_covariance.35': 0.06,  # yaw
            },
            'lidar': {
                'process_noise_covariance.0': 0.1,    # x position
                'process_noise_covariance.7': 0.1,    # y position
                'process_noise_covariance.35': 0.2,   # yaw
            },
            'encoders': {
                'process_noise_covariance.42': 0.1,   # x velocity
                'process_noise_covariance.49': 0.1,   # y velocity
                'process_noise_covariance.77': 0.2,   # yaw velocity
            }
        }
        
        return param_mappings.get(sensor_name, {})
    
    def _set_ekf_parameter(self, param_name: str, value: float):
        """Set an EKF parameter via service call"""
        try:
            from rcl_interfaces.msg import Parameter, ParameterValue
            from rcl_interfaces.srv import SetParameters
            
            request = SetParameters.Request()
            param = Parameter()
            param.name = param_name
            param.value = ParameterValue()
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = value
            
            request.parameters = [param]
            
            # Async service call
            future = self.ekf_param_client.call_async(request)
            
        except Exception as e:
            self.get_logger().error(f"Failed to set EKF parameter {param_name}: {str(e)}")
    
    def _publish_diagnostics(self):
        """Publish diagnostic information"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        for sensor_name, metrics in self.sensor_metrics.items():
            diag_status = DiagnosticStatus()
            diag_status.name = f"adaptive_covariance/{sensor_name}"
            diag_status.hardware_id = sensor_name
            
            # Set status level based on health
            if metrics.health_status == SensorHealth.EXCELLENT:
                diag_status.level = DiagnosticStatus.OK
                diag_status.message = "Sensor operating excellently"
            elif metrics.health_status == SensorHealth.GOOD:
                diag_status.level = DiagnosticStatus.OK
                diag_status.message = "Sensor operating normally"
            elif metrics.health_status == SensorHealth.DEGRADED:
                diag_status.level = DiagnosticStatus.WARN
                diag_status.message = "Sensor performance degraded"
            elif metrics.health_status == SensorHealth.POOR:
                diag_status.level = DiagnosticStatus.WARN
                diag_status.message = "Sensor performance poor"
            else:  # FAILED
                diag_status.level = DiagnosticStatus.ERROR
                diag_status.message = "Sensor failed or no data"
            
            # Add key-value pairs
            diag_status.values = [
                KeyValue(key="health_status", value=metrics.health_status.name),
                KeyValue(key="data_rate", value=f"{metrics.data_rate:.1f}"),
                KeyValue(key="message_count", value=str(metrics.message_count)),
                KeyValue(key="covariance_factor", value=f"{metrics.covariance_factor:.2f}"),
                KeyValue(key="last_update", value=f"{metrics.last_update:.3f}"),
            ]
            
            # Add sensor-specific indicators
            for key, value in metrics.quality_indicators.items():
                diag_status.values.append(KeyValue(key=key, value=f"{value:.3f}"))
            
            diag_array.status.append(diag_status)
        
        self.diagnostics_pub.publish(diag_array)
    
    def _publish_sensor_health(self):
        """Publish sensor health summary"""
        health_summary = {}
        
        with self.metrics_lock:
            for sensor_name, metrics in self.sensor_metrics.items():
                health_summary[sensor_name] = {
                    'status': metrics.health_status.name,
                    'rate': metrics.data_rate,
                    'factor': metrics.covariance_factor
                }
        
        health_msg = String()
        health_msg.data = yaml.dump(health_summary)
        self.sensor_health_pub.publish(health_msg)
    
    def _publish_adjustment_log(self, adjustments: List[CovarianceAdjustment]):
        """Publish covariance adjustment log"""
        log_data = []
        for adj in adjustments:
            log_data.append({
                'sensor': adj.sensor_name,
                'factor': adj.adjustment_factor,
                'reason': adj.reason,
                'timestamp': adj.timestamp
            })
        
        log_msg = String()
        log_msg.data = yaml.dump(log_data)
        self.covariance_log_pub.publish(log_msg)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    node = AdaptiveCovarianceNode()
    
    # Use multi-threaded executor for concurrent callbacks
    executor = MultiThreadedExecutor(num_threads=4)
    
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Adaptive Covariance Node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Sensor Health Monitor - Dedicated monitoring node for individual sensors

This node provides detailed monitoring of individual sensors with specialized
health metrics and alerting capabilities.

Author: fjgarco
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
from typing import Dict, List, Any, Optional
from dataclasses import dataclass
import threading
import time
from collections import deque

# ROS 2 messages
from std_msgs.msg import Header, String, Bool
from sensor_msgs.msg import NavSatFix, Imu, MagneticField, PointCloud2
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Vector3

# Custom messages
from encoder_odometry.msg import EncoderTicks


@dataclass
class HealthAlert:
    """Health alert message"""
    sensor_name: str
    alert_type: str
    severity: str  # INFO, WARN, ERROR, CRITICAL
    message: str
    timestamp: float
    value: float
    threshold: float


class SensorHealthMonitor(Node):
    """
    Specialized sensor health monitoring node
    
    Provides detailed health analysis for each sensor type with:
    - Historical trend analysis
    - Alert generation and escalation
    - Performance benchmarking
    - Predictive health assessment
    """
    
    def __init__(self):
        super().__init__('sensor_health_monitor')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # Parameters
        self._declare_parameters()
        
        # Health history storage (circular buffers)
        self.health_history: Dict[str, deque] = {}
        self.alert_history: List[HealthAlert] = []
        
        # Threading
        self.health_lock = threading.RLock()
        
        # Performance baselines
        self.performance_baselines: Dict[str, Dict[str, float]] = {}
        
        # Initialize monitoring
        self._setup_subscribers()
        self._setup_publishers()
        
        # Start health analysis timer
        self.analysis_timer = self.create_timer(
            self.analysis_period,
            self._analyze_sensor_health,
            callback_group=self.callback_group
        )
        
        # Start trend analysis timer
        self.trend_timer = self.create_timer(
            self.trend_period,
            self._analyze_trends,
            callback_group=self.callback_group
        )
        
        self.get_logger().info("Sensor Health Monitor initialized")
    
    def _declare_parameters(self):
        """Declare node parameters"""
        
        # Timing parameters
        self.declare_parameter('analysis_period', 1.0)  # 1 Hz detailed analysis
        self.declare_parameter('trend_period', 10.0)    # 0.1 Hz trend analysis
        self.declare_parameter('history_length', 1000)  # Keep 1000 samples
        
        # Alert thresholds
        self.declare_parameter('rate_drop_threshold', 0.5)      # 50% rate drop = alert
        self.declare_parameter('covariance_spike_threshold', 5.0)  # 5x covariance = alert
        self.declare_parameter('consecutive_alerts_limit', 5)    # Escalate after 5 consecutive
        
        # Get values
        self.analysis_period = self.get_parameter('analysis_period').value
        self.trend_period = self.get_parameter('trend_period').value
        self.history_length = self.get_parameter('history_length').value
        self.rate_drop_threshold = self.get_parameter('rate_drop_threshold').value
        self.covariance_spike_threshold = self.get_parameter('covariance_spike_threshold').value
        self.consecutive_alerts_limit = self.get_parameter('consecutive_alerts_limit').value
    
    def _setup_subscribers(self):
        """Setup sensor subscribers for health monitoring"""
        
        # GPS Health Monitor
        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/raw/fix',
            self._monitor_gps_health, 10, callback_group=self.callback_group
        )
        
        # IMU Health Monitor
        self.imu_sub = self.create_subscription(
            Imu, '/mavros/imu/data',
            self._monitor_imu_health, 10, callback_group=self.callback_group
        )
        
        # Magnetometer Health Monitor
        self.mag_sub = self.create_subscription(
            MagneticField, '/mavros/imu/mag',
            self._monitor_mag_health, 10, callback_group=self.callback_group
        )
        
        # LIDAR Health Monitor
        self.lidar_sub = self.create_subscription(
            Odometry, '/odometry/lidar',
            self._monitor_lidar_health, 10, callback_group=self.callback_group
        )
        
        # Encoder Health Monitor
        self.encoder_sub = self.create_subscription(
            Odometry, '/odometry/encoders',
            self._monitor_encoder_health, 10, callback_group=self.callback_group
        )
        
        # Encoder raw data monitor
        self.encoder_ticks_sub = self.create_subscription(
            EncoderTicks, '/encoders/ticks',
            self._monitor_encoder_raw_health, 10, callback_group=self.callback_group
        )
    
    def _setup_publishers(self):
        """Setup health monitoring publishers"""
        
        # Health alerts publisher
        self.alerts_pub = self.create_publisher(
            String, '/sensor_health/alerts', 10
        )
        
        # Performance metrics publisher
        self.performance_pub = self.create_publisher(
            String, '/sensor_health/performance', 10
        )
        
        # Health summary publisher
        self.summary_pub = self.create_publisher(
            DiagnosticArray, '/sensor_health/summary', 10
        )
        
        # Individual sensor status publishers
        self.gps_status_pub = self.create_publisher(Bool, '/sensor_health/gps/ok', 10)
        self.imu_status_pub = self.create_publisher(Bool, '/sensor_health/imu/ok', 10)
        self.lidar_status_pub = self.create_publisher(Bool, '/sensor_health/lidar/ok', 10)
        self.encoder_status_pub = self.create_publisher(Bool, '/sensor_health/encoders/ok', 10)
    
    def _init_sensor_history(self, sensor_name: str):
        """Initialize history tracking for a sensor"""
        if sensor_name not in self.health_history:
            self.health_history[sensor_name] = deque(maxlen=self.history_length)
            self.performance_baselines[sensor_name] = {}
    
    def _add_health_sample(self, sensor_name: str, metrics: Dict[str, Any]):
        """Add a health sample to history"""
        with self.health_lock:
            self._init_sensor_history(sensor_name)
            
            sample = {
                'timestamp': self.get_clock().now().nanoseconds / 1e9,
                'metrics': metrics.copy()
            }
            
            self.health_history[sensor_name].append(sample)
    
    def _monitor_gps_health(self, msg: NavSatFix):
        """Monitor GPS health metrics"""
        
        # Extract health metrics
        metrics = {
            'fix_status': msg.status.status,
            'service': msg.status.service,
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
            'position_covariance': list(msg.position_covariance),
            'covariance_type': msg.position_covariance_type
        }
        
        # Calculate HDOP if covariance is available
        if len(msg.position_covariance) >= 9:
            covar_matrix = np.array(msg.position_covariance).reshape(3, 3)
            hdop = np.sqrt(covar_matrix[0, 0] + covar_matrix[1, 1])
            metrics['hdop'] = hdop
        
        # Add to history
        self._add_health_sample('gps', metrics)
        
        # Immediate health checks
        self._check_gps_immediate_health(metrics)
    
    def _monitor_imu_health(self, msg: Imu):
        """Monitor IMU health metrics"""
        
        # Extract health metrics
        metrics = {
            'orientation': [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
            'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
            'orientation_covariance': list(msg.orientation_covariance),
            'angular_velocity_covariance': list(msg.angular_velocity_covariance),
            'linear_acceleration_covariance': list(msg.linear_acceleration_covariance)
        }
        
        # Calculate magnitudes
        accel_mag = np.linalg.norm(metrics['linear_acceleration'])
        gyro_mag = np.linalg.norm(metrics['angular_velocity'])
        
        metrics['accel_magnitude'] = accel_mag
        metrics['gyro_magnitude'] = gyro_mag
        
        # Add to history
        self._add_health_sample('imu', metrics)
        
        # Immediate health checks
        self._check_imu_immediate_health(metrics)
    
    def _monitor_mag_health(self, msg: MagneticField):
        """Monitor magnetometer health metrics"""
        
        metrics = {
            'magnetic_field': [msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z],
            'magnetic_field_covariance': list(msg.magnetic_field_covariance)
        }
        
        # Calculate magnitude and heading
        mag_mag = np.linalg.norm(metrics['magnetic_field'])
        heading = np.arctan2(msg.magnetic_field.y, msg.magnetic_field.x)
        
        metrics['magnetic_magnitude'] = mag_mag
        metrics['magnetic_heading'] = heading
        
        # Add to history
        self._add_health_sample('magnetometer', metrics)
        
        # Immediate health checks
        self._check_mag_immediate_health(metrics)
    
    def _monitor_lidar_health(self, msg: Odometry):
        """Monitor LIDAR odometry health metrics"""
        
        metrics = {
            'position': [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
            'orientation': [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                          msg.pose.pose.orientation.z, msg.pose.pose.orientation.w],
            'velocity': [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z],
            'angular_velocity': [msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z],
            'pose_covariance': list(msg.pose.covariance),
            'twist_covariance': list(msg.twist.covariance)
        }
        
        # Calculate velocity magnitude
        vel_mag = np.linalg.norm(metrics['velocity'])
        metrics['velocity_magnitude'] = vel_mag
        
        # Calculate position covariance trace
        pose_cov_matrix = np.array(metrics['pose_covariance']).reshape(6, 6)
        position_cov_trace = np.trace(pose_cov_matrix[:3, :3])
        metrics['position_covariance_trace'] = position_cov_trace
        
        # Add to history
        self._add_health_sample('lidar', metrics)
        
        # Immediate health checks
        self._check_lidar_immediate_health(metrics)
    
    def _monitor_encoder_health(self, msg: Odometry):
        """Monitor encoder odometry health metrics"""
        
        metrics = {
            'position': [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
            'orientation': [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                          msg.pose.pose.orientation.z, msg.pose.pose.orientation.w],
            'velocity': [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z],
            'angular_velocity': [msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z],
            'pose_covariance': list(msg.pose.covariance),
            'twist_covariance': list(msg.twist.covariance)
        }
        
        # Calculate velocity magnitude
        vel_mag = np.linalg.norm(metrics['velocity'])
        metrics['velocity_magnitude'] = vel_mag
        
        # Add to history
        self._add_health_sample('encoders', metrics)
        
        # Immediate health checks
        self._check_encoder_immediate_health(metrics)
    
    def _monitor_encoder_raw_health(self, msg: EncoderTicks):
        """Monitor raw encoder tick health"""
        
        metrics = {
            'front_left': msg.front_left,
            'front_right': msg.front_right,
            'rear_left': msg.rear_left,
            'rear_right': msg.rear_right,
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        }
        
        # Calculate tick rates and slip indicators
        total_ticks = abs(msg.front_left) + abs(msg.front_right) + abs(msg.rear_left) + abs(msg.rear_right)
        left_avg = (msg.front_left + msg.rear_left) / 2.0
        right_avg = (msg.front_right + msg.rear_right) / 2.0
        
        metrics['total_ticks'] = total_ticks
        metrics['left_average'] = left_avg
        metrics['right_average'] = right_avg
        
        # Slip detection
        if abs(left_avg) + abs(right_avg) > 0:
            slip_ratio = abs(left_avg - right_avg) / (abs(left_avg) + abs(right_avg))
            metrics['slip_ratio'] = slip_ratio
        else:
            metrics['slip_ratio'] = 0.0
        
        # Add to history
        self._add_health_sample('encoder_raw', metrics)
        
        # Immediate health checks
        self._check_encoder_raw_immediate_health(metrics)
    
    def _check_gps_immediate_health(self, metrics: Dict[str, Any]):
        """Immediate GPS health checks"""
        
        # Check fix status
        fix_status = metrics.get('fix_status', -1)
        if fix_status <= 0:  # NO_FIX
            self._generate_alert('gps', 'no_fix', 'ERROR', 
                               f"GPS has no fix (status: {fix_status})", 
                               fix_status, 1)
        
        # Check HDOP if available
        hdop = metrics.get('hdop', 0.0)
        if hdop > 3.0:
            self._generate_alert('gps', 'poor_hdop', 'WARN',
                               f"GPS HDOP too high: {hdop:.2f}",
                               hdop, 3.0)
    
    def _check_imu_immediate_health(self, metrics: Dict[str, Any]):
        """Immediate IMU health checks"""
        
        # Check acceleration magnitude (should be around 9.8 when stationary)
        accel_mag = metrics.get('accel_magnitude', 0.0)
        if accel_mag < 5.0 or accel_mag > 20.0:
            self._generate_alert('imu', 'abnormal_accel', 'WARN',
                               f"IMU acceleration magnitude abnormal: {accel_mag:.2f} m/s²",
                               accel_mag, 9.8)
        
        # Check for excessive gyroscope readings
        gyro_mag = metrics.get('gyro_magnitude', 0.0)
        if gyro_mag > 10.0:  # 10 rad/s is quite high
            self._generate_alert('imu', 'excessive_rotation', 'WARN',
                               f"IMU reporting excessive rotation: {gyro_mag:.2f} rad/s",
                               gyro_mag, 10.0)
    
    def _check_mag_immediate_health(self, metrics: Dict[str, Any]):
        """Immediate magnetometer health checks"""
        
        # Check magnetic field magnitude (Earth's field is ~25-65 µT)
        mag_mag = metrics.get('magnetic_magnitude', 0.0)
        if mag_mag < 20e-6 or mag_mag > 100e-6:  # Convert µT to T
            self._generate_alert('magnetometer', 'abnormal_field', 'WARN',
                               f"Magnetic field magnitude abnormal: {mag_mag*1e6:.1f} µT",
                               mag_mag*1e6, 50.0)
    
    def _check_lidar_immediate_health(self, metrics: Dict[str, Any]):
        """Immediate LIDAR health checks"""
        
        # Check position covariance
        pos_cov = metrics.get('position_covariance_trace', 0.0)
        if pos_cov > 10.0:
            self._generate_alert('lidar', 'high_covariance', 'WARN',
                               f"LIDAR position covariance too high: {pos_cov:.3f}",
                               pos_cov, 10.0)
    
    def _check_encoder_immediate_health(self, metrics: Dict[str, Any]):
        """Immediate encoder odometry health checks"""
        
        # Check for unrealistic velocities
        vel_mag = metrics.get('velocity_magnitude', 0.0)
        if vel_mag > 10.0:  # 10 m/s = 36 km/h (reasonable robot limit)
            self._generate_alert('encoders', 'unrealistic_velocity', 'WARN',
                               f"Encoder reporting unrealistic velocity: {vel_mag:.2f} m/s",
                               vel_mag, 10.0)
    
    def _check_encoder_raw_immediate_health(self, metrics: Dict[str, Any]):
        """Immediate raw encoder health checks"""
        
        # Check slip ratio
        slip_ratio = metrics.get('slip_ratio', 0.0)
        if slip_ratio > 0.2:  # 20% slip is concerning
            self._generate_alert('encoder_raw', 'high_slip', 'WARN',
                               f"High wheel slip detected: {slip_ratio*100:.1f}%",
                               slip_ratio, 0.2)
    
    def _generate_alert(self, sensor_name: str, alert_type: str, severity: str, 
                       message: str, value: float, threshold: float):
        """Generate and publish a health alert"""
        
        alert = HealthAlert(
            sensor_name=sensor_name,
            alert_type=alert_type,
            severity=severity,
            message=message,
            timestamp=self.get_clock().now().nanoseconds / 1e9,
            value=value,
            threshold=threshold
        )
        
        with self.health_lock:
            self.alert_history.append(alert)
        
        # Log the alert
        if severity == 'ERROR':
            self.get_logger().error(f"{sensor_name}: {message}")
        elif severity == 'WARN':
            self.get_logger().warn(f"{sensor_name}: {message}")
        else:
            self.get_logger().info(f"{sensor_name}: {message}")
        
        # Publish alert
        self._publish_alert(alert)
    
    def _publish_alert(self, alert: HealthAlert):
        """Publish a health alert"""
        alert_msg = String()
        alert_msg.data = f"{alert.timestamp:.3f}|{alert.sensor_name}|{alert.alert_type}|{alert.severity}|{alert.message}|{alert.value:.3f}|{alert.threshold:.3f}"
        self.alerts_pub.publish(alert_msg)
    
    def _analyze_sensor_health(self):
        """Perform detailed sensor health analysis"""
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        with self.health_lock:
            for sensor_name, history in self.health_history.items():
                if len(history) < 10:  # Need some history for analysis
                    continue
                
                # Analyze data rate
                self._analyze_data_rate(sensor_name, history, current_time)
                
                # Analyze metric trends
                self._analyze_metric_trends(sensor_name, history)
                
                # Update performance baselines
                self._update_performance_baseline(sensor_name, history)
        
        # Publish health summary
        self._publish_health_summary()
        
        # Publish individual sensor status
        self._publish_sensor_status()
    
    def _analyze_data_rate(self, sensor_name: str, history: deque, current_time: float):
        """Analyze sensor data rate"""
        
        if len(history) < 2:
            return
        
        # Calculate recent data rate (last 10 messages)
        recent_samples = list(history)[-10:]
        if len(recent_samples) < 2:
            return
        
        time_span = recent_samples[-1]['timestamp'] - recent_samples[0]['timestamp']
        if time_span > 0:
            recent_rate = (len(recent_samples) - 1) / time_span
        else:
            recent_rate = 0.0
        
        # Check against baseline
        baseline_rate = self.performance_baselines[sensor_name].get('avg_rate', 0.0)
        if baseline_rate > 0 and recent_rate < baseline_rate * self.rate_drop_threshold:
            self._generate_alert(sensor_name, 'rate_drop', 'WARN',
                               f"Data rate dropped: {recent_rate:.1f} Hz (baseline: {baseline_rate:.1f} Hz)",
                               recent_rate, baseline_rate * self.rate_drop_threshold)
    
    def _analyze_metric_trends(self, sensor_name: str, history: deque):
        """Analyze trends in sensor metrics"""
        
        if len(history) < 20:  # Need sufficient history for trend analysis
            return
        
        recent_samples = list(history)[-20:]
        
        # Analyze covariance trends (if applicable)
        if sensor_name in ['gps', 'imu', 'lidar', 'encoders']:
            self._analyze_covariance_trends(sensor_name, recent_samples)
        
        # Sensor-specific trend analysis
        if sensor_name == 'gps':
            self._analyze_gps_trends(recent_samples)
        elif sensor_name == 'imu':
            self._analyze_imu_trends(recent_samples)
        elif sensor_name == 'encoder_raw':
            self._analyze_encoder_raw_trends(recent_samples)
    
    def _analyze_covariance_trends(self, sensor_name: str, samples: List[Dict]):
        """Analyze covariance trends"""
        
        covariance_key = None
        if sensor_name == 'gps' and 'position_covariance' in samples[0]['metrics']:
            covariance_values = [np.trace(np.array(s['metrics']['position_covariance']).reshape(3, 3)) 
                               for s in samples if 'position_covariance' in s['metrics']]
        elif sensor_name in ['lidar', 'encoders'] and 'position_covariance_trace' in samples[0]['metrics']:
            covariance_values = [s['metrics']['position_covariance_trace'] for s in samples]
        else:
            return
        
        if len(covariance_values) < 10:
            return
        
        # Check for covariance spikes
        baseline_cov = self.performance_baselines[sensor_name].get('avg_covariance', np.mean(covariance_values))
        max_recent_cov = max(covariance_values[-5:])  # Last 5 samples
        
        if max_recent_cov > baseline_cov * self.covariance_spike_threshold:
            self._generate_alert(sensor_name, 'covariance_spike', 'WARN',
                               f"Covariance spike detected: {max_recent_cov:.3f} (baseline: {baseline_cov:.3f})",
                               max_recent_cov, baseline_cov * self.covariance_spike_threshold)
    
    def _analyze_gps_trends(self, samples: List[Dict]):
        """Analyze GPS-specific trends"""
        
        # HDOP trend analysis
        hdop_values = [s['metrics'].get('hdop', 0.0) for s in samples if 'hdop' in s['metrics']]
        if len(hdop_values) >= 10:
            recent_hdop = np.mean(hdop_values[-5:])
            if recent_hdop > 2.0:
                self._generate_alert('gps', 'hdop_degradation', 'WARN',
                                   f"HDOP degrading: {recent_hdop:.2f}",
                                   recent_hdop, 2.0)
    
    def _analyze_imu_trends(self, samples: List[Dict]):
        """Analyze IMU-specific trends"""
        
        # Check for bias drift in accelerometer
        accel_values = [s['metrics']['accel_magnitude'] for s in samples]
        if len(accel_values) >= 20:
            # Look for systematic drift
            first_half = np.mean(accel_values[:10])
            second_half = np.mean(accel_values[-10:])
            drift = abs(second_half - first_half)
            
            if drift > 2.0:  # 2 m/s² drift is significant
                self._generate_alert('imu', 'accel_drift', 'WARN',
                                   f"Accelerometer drift detected: {drift:.2f} m/s²",
                                   drift, 2.0)
    
    def _analyze_encoder_raw_trends(self, samples: List[Dict]):
        """Analyze raw encoder trends"""
        
        # Analyze slip ratio trends
        slip_values = [s['metrics']['slip_ratio'] for s in samples]
        if len(slip_values) >= 10:
            avg_slip = np.mean(slip_values)
            if avg_slip > 0.1:
                self._generate_alert('encoder_raw', 'persistent_slip', 'WARN',
                                   f"Persistent wheel slip: {avg_slip*100:.1f}%",
                                   avg_slip, 0.1)
    
    def _update_performance_baseline(self, sensor_name: str, history: deque):
        """Update performance baselines for a sensor"""
        
        if len(history) < 50:  # Need sufficient data for reliable baseline
            return
        
        baseline = self.performance_baselines[sensor_name]
        
        # Calculate data rate baseline
        timestamps = [s['timestamp'] for s in history]
        if len(timestamps) > 1:
            time_span = timestamps[-1] - timestamps[0]
            avg_rate = (len(timestamps) - 1) / time_span
            baseline['avg_rate'] = avg_rate
        
        # Calculate metric baselines based on sensor type
        if sensor_name == 'gps':
            hdop_values = [s['metrics'].get('hdop', 0.0) for s in history if 'hdop' in s['metrics']]
            if hdop_values:
                baseline['avg_hdop'] = np.mean(hdop_values)
        
        elif sensor_name == 'imu':
            accel_values = [s['metrics']['accel_magnitude'] for s in history]
            if accel_values:
                baseline['avg_accel'] = np.mean(accel_values)
        
        elif sensor_name in ['lidar', 'encoders']:
            cov_values = [s['metrics'].get('position_covariance_trace', 0.0) for s in history]
            if cov_values:
                baseline['avg_covariance'] = np.mean(cov_values)
    
    def _analyze_trends(self):
        """Analyze long-term trends"""
        
        with self.health_lock:
            for sensor_name, history in self.health_history.items():
                if len(history) < 100:  # Need substantial history for trend analysis
                    continue
                
                self._analyze_long_term_performance(sensor_name, history)
        
        # Publish performance metrics
        self._publish_performance_metrics()
    
    def _analyze_long_term_performance(self, sensor_name: str, history: deque):
        """Analyze long-term performance trends"""
        
        # Divide history into time windows for trend analysis
        samples = list(history)
        window_size = len(samples) // 5  # 5 time windows
        
        if window_size < 10:
            return
        
        # Analyze performance degradation over time
        windows = [samples[i:i+window_size] for i in range(0, len(samples), window_size)]
        
        # Calculate performance metrics for each window
        performance_trend = []
        for window in windows:
            if len(window) < 5:
                continue
            
            # Calculate window metrics based on sensor type
            if sensor_name == 'gps':
                hdop_values = [s['metrics'].get('hdop', float('inf')) for s in window if 'hdop' in s['metrics']]
                avg_performance = np.mean(hdop_values) if hdop_values else float('inf')
            elif sensor_name == 'imu':
                accel_values = [s['metrics']['accel_magnitude'] for s in window]
                # Performance is inverse of deviation from 9.8 m/s²
                deviations = [abs(a - 9.8) for a in accel_values]
                avg_performance = np.mean(deviations)
            else:
                # Generic covariance-based performance
                cov_values = [s['metrics'].get('position_covariance_trace', float('inf')) for s in window]
                avg_performance = np.mean(cov_values) if cov_values else float('inf')
            
            performance_trend.append(avg_performance)
        
        # Check for performance degradation trend
        if len(performance_trend) >= 3:
            # Linear regression to detect trend
            x = np.arange(len(performance_trend))
            z = np.polyfit(x, performance_trend, 1)
            trend_slope = z[0]
            
            # Alert if performance is degrading
            if trend_slope > 0.1:  # Threshold depends on sensor type
                self._generate_alert(sensor_name, 'performance_degradation', 'INFO',
                                   f"Long-term performance degradation detected (slope: {trend_slope:.3f})",
                                   trend_slope, 0.1)
    
    def _publish_health_summary(self):
        """Publish comprehensive health summary"""
        
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        with self.health_lock:
            for sensor_name, history in self.health_history.items():
                if len(history) == 0:
                    continue
                
                diag_status = DiagnosticStatus()
                diag_status.name = f"sensor_health_monitor/{sensor_name}"
                diag_status.hardware_id = sensor_name
                
                # Determine overall health status
                recent_alerts = [a for a in self.alert_history if 
                               a.sensor_name == sensor_name and 
                               a.timestamp > (self.get_clock().now().nanoseconds / 1e9 - 60)]  # Last minute
                
                if any(a.severity == 'ERROR' for a in recent_alerts):
                    diag_status.level = DiagnosticStatus.ERROR
                    diag_status.message = "Sensor has errors"
                elif any(a.severity == 'WARN' for a in recent_alerts):
                    diag_status.level = DiagnosticStatus.WARN
                    diag_status.message = "Sensor has warnings"
                else:
                    diag_status.level = DiagnosticStatus.OK
                    diag_status.message = "Sensor operating normally"
                
                # Add diagnostic values
                latest_sample = history[-1]
                diag_status.values = [
                    KeyValue(key="data_rate", value=f"{self.performance_baselines[sensor_name].get('avg_rate', 0.0):.1f}"),
                    KeyValue(key="samples_count", value=str(len(history))),
                    KeyValue(key="last_update", value=f"{latest_sample['timestamp']:.3f}"),
                    KeyValue(key="recent_alerts", value=str(len(recent_alerts)))
                ]
                
                # Add sensor-specific values
                if sensor_name == 'gps':
                    hdop = latest_sample['metrics'].get('hdop', 0.0)
                    fix_status = latest_sample['metrics'].get('fix_status', -1)
                    diag_status.values.extend([
                        KeyValue(key="hdop", value=f"{hdop:.2f}"),
                        KeyValue(key="fix_status", value=str(fix_status))
                    ])
                
                diag_array.status.append(diag_status)
        
        self.summary_pub.publish(diag_array)
    
    def _publish_sensor_status(self):
        """Publish individual sensor OK/NOK status"""
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        with self.health_lock:
            # GPS status
            gps_ok = self._is_sensor_ok('gps', current_time)
            self.gps_status_pub.publish(Bool(data=gps_ok))
            
            # IMU status
            imu_ok = self._is_sensor_ok('imu', current_time)
            self.imu_status_pub.publish(Bool(data=imu_ok))
            
            # LIDAR status
            lidar_ok = self._is_sensor_ok('lidar', current_time)
            self.lidar_status_pub.publish(Bool(data=lidar_ok))
            
            # Encoder status
            encoder_ok = self._is_sensor_ok('encoders', current_time) and self._is_sensor_ok('encoder_raw', current_time)
            self.encoder_status_pub.publish(Bool(data=encoder_ok))
    
    def _is_sensor_ok(self, sensor_name: str, current_time: float) -> bool:
        """Determine if a sensor is OK based on recent alerts and data"""
        
        # Check if we have recent data
        if sensor_name not in self.health_history or len(self.health_history[sensor_name]) == 0:
            return False
        
        latest_sample = self.health_history[sensor_name][-1]
        time_since_update = current_time - latest_sample['timestamp']
        
        # No data for too long = not OK
        if time_since_update > 5.0:  # 5 seconds
            return False
        
        # Check for recent critical alerts
        recent_critical_alerts = [a for a in self.alert_history if 
                                a.sensor_name == sensor_name and 
                                a.severity in ['ERROR', 'CRITICAL'] and 
                                a.timestamp > (current_time - 30)]  # Last 30 seconds
        
        return len(recent_critical_alerts) == 0
    
    def _publish_performance_metrics(self):
        """Publish performance metrics"""
        
        performance_data = {}
        
        with self.health_lock:
            for sensor_name, baseline in self.performance_baselines.items():
                performance_data[sensor_name] = baseline.copy()
        
        perf_msg = String()
        perf_msg.data = str(performance_data)  # Could use JSON or YAML for better formatting
        self.performance_pub.publish(perf_msg)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    node = SensorHealthMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Sensor Health Monitor")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

"""
Multi-Sensor Synchronization Monitor for Odometry Fusion
========================================================

This node synchronizes multiple odometry sources using message_filters to ensure
temporal alignment for adaptive covariance adjustment and sensor fusion analysis.

Author: ROS 2 Odometry Workspace
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import message_filters
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

from adaptive_covariance_interfaces.msg import SynchronizedSensorData, SensorSyncStatus
from adaptive_covariance_interfaces.srv import GetSyncStatus

import numpy as np
import threading
from collections import defaultdict, deque
from typing import Dict, List, Optional, Tuple
import time


class SyncMonitor(Node):
    """
    Multi-sensor synchronization monitor for odometry fusion.
    
    Uses message_filters to synchronize multiple odometry sources and provides
    temporally aligned sensor data for adaptive covariance adjustment and
    fusion analysis.
    """
    
    def __init__(self):
        super().__init__('sync_monitor')
        
        # Declare parameters
        self._declare_parameters()
        
        # Initialize state
        self._initialize_state()
        
        # Setup QoS profiles
        self._setup_qos_profiles()
        
        # Setup subscribers and synchronizer
        self._setup_subscribers()
        
        # Setup publishers and services
        self._setup_publishers_services()
        
        # Setup monitoring timers
        self._setup_timers()
        
        self.get_logger().info("Sync Monitor initialized successfully")
    
    def _declare_parameters(self):
        """Declare ROS parameters for configuration."""
        
        # Synchronization parameters
        self.declare_parameter('sync_tolerance_ms', 50.0)  # Max time difference for sync
        self.declare_parameter('queue_size', 10)           # Message queue size
        
        # Sensor topic configuration
        self.declare_parameter('topics.visual_odom', '/zed/odom')
        self.declare_parameter('topics.lidar_odom', '/odometry/lidar')
        self.declare_parameter('topics.gnss_odom', '/odometry/gps')
        self.declare_parameter('topics.imu_data', '/mavros/imu/data')
        self.declare_parameter('topics.encoder_odom', '/odometry/encoder')
        
        # Sensor enable/disable flags
        self.declare_parameter('sensors.enable_visual', True)
        self.declare_parameter('sensors.enable_lidar', True)
        self.declare_parameter('sensors.enable_gnss', True)
        self.declare_parameter('sensors.enable_imu', True)
        self.declare_parameter('sensors.enable_encoder', True)
        
        # Monitoring parameters
        self.declare_parameter('max_delay_warning_ms', 100.0)  # Warn if delay > this
        self.declare_parameter('max_delay_critical_ms', 500.0)  # Critical if delay > this
        self.declare_parameter('monitoring_rate_hz', 1.0)      # Status publishing rate
        
        # Debug parameters
        self.declare_parameter('enable_debug_logging', False)
        self.declare_parameter('debug_log_interval_s', 5.0)
        
        # Fallback parameters
        self.declare_parameter('min_sensors_required', 2)  # Minimum sensors for sync
        self.declare_parameter('timeout_sensor_missing_s', 5.0)  # Timeout for missing sensor
    
    def _initialize_state(self):
        """Initialize internal state variables."""
        
        # Get parameters
        self.sync_tolerance = self.get_parameter('sync_tolerance_ms').value / 1000.0
        self.queue_size = self.get_parameter('queue_size').value
        self.max_delay_warning = self.get_parameter('max_delay_warning_ms').value / 1000.0
        self.max_delay_critical = self.get_parameter('max_delay_critical_ms').value / 1000.0
        self.min_sensors_required = self.get_parameter('min_sensors_required').value
        self.sensor_timeout = self.get_parameter('timeout_sensor_missing_s').value
        
        # Sensor configuration
        self.sensor_topics = {
            'visual': self.get_parameter('topics.visual_odom').value,
            'lidar': self.get_parameter('topics.lidar_odom').value,
            'gnss': self.get_parameter('topics.gnss_odom').value,
            'imu': self.get_parameter('topics.imu_data').value,
            'encoder': self.get_parameter('topics.encoder_odom').value
        }
        
        self.sensor_enabled = {
            'visual': self.get_parameter('sensors.enable_visual').value,
            'lidar': self.get_parameter('sensors.enable_lidar').value,
            'gnss': self.get_parameter('sensors.enable_gnss').value,
            'imu': self.get_parameter('sensors.enable_imu').value,
            'encoder': self.get_parameter('sensors.enable_encoder').value
        }
        
        # Synchronization state
        self.subscribers = {}
        self.sync_callback_count = 0
        self.last_sync_time = None
        self.sync_success_rate = 0.0
        
        # Sensor monitoring
        self.sensor_delays = defaultdict(deque)  # Rolling window of delays
        self.sensor_last_seen = defaultdict(float)
        self.sensor_status = defaultdict(str)  # 'ok', 'warning', 'critical', 'missing'
        self.sensor_message_count = defaultdict(int)
        
        # Thread safety
        self.state_lock = threading.Lock()
        
        # Debug logging
        self.enable_debug = self.get_parameter('enable_debug_logging').value
        self.debug_interval = self.get_parameter('debug_log_interval_s').value
        self.last_debug_log = time.time()
    
    def _setup_qos_profiles(self):
        """Setup QoS profiles for different message types."""
        
        # Standard odometry QoS - reliable, volatile
        self.odom_qos = QoSProfile(
            depth=self.queue_size,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # IMU QoS - best effort for high frequency data
        self.imu_qos = QoSProfile(
            depth=self.queue_size * 2,  # Higher queue for IMU
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
    
    def _setup_subscribers(self):
        """Setup message_filters subscribers for synchronization."""
        
        enabled_sensors = []
        
        # Create subscribers for enabled sensors
        if self.sensor_enabled['visual']:
            self.subscribers['visual'] = message_filters.Subscriber(
                self, Odometry, self.sensor_topics['visual'], qos_profile=self.odom_qos)
            enabled_sensors.append('visual')
        
        if self.sensor_enabled['lidar']:
            self.subscribers['lidar'] = message_filters.Subscriber(
                self, Odometry, self.sensor_topics['lidar'], qos_profile=self.odom_qos)
            enabled_sensors.append('lidar')
        
        if self.sensor_enabled['gnss']:
            self.subscribers['gnss'] = message_filters.Subscriber(
                self, Odometry, self.sensor_topics['gnss'], qos_profile=self.odom_qos)
            enabled_sensors.append('gnss')
        
        if self.sensor_enabled['imu']:
            self.subscribers['imu'] = message_filters.Subscriber(
                self, Imu, self.sensor_topics['imu'], qos_profile=self.imu_qos)
            enabled_sensors.append('imu')
        
        if self.sensor_enabled['encoder']:
            self.subscribers['encoder'] = message_filters.Subscriber(
                self, Odometry, self.sensor_topics['encoder'], qos_profile=self.odom_qos)
            enabled_sensors.append('encoder')
        
        # Check minimum sensors requirement
        if len(enabled_sensors) < self.min_sensors_required:
            self.get_logger().error(
                f"Insufficient sensors enabled ({len(enabled_sensors)}) "
                f"< minimum required ({self.min_sensors_required})")
            return
        
        # Setup approximate time synchronizer
        subscriber_list = [self.subscribers[sensor] for sensor in enabled_sensors]
        
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            subscriber_list,
            queue_size=self.queue_size,
            slop=self.sync_tolerance  # Maximum time difference
        )
        
        self.synchronizer.registerCallback(self._synchronized_callback)
        self.enabled_sensors = enabled_sensors
        
        self.get_logger().info(
            f"Synchronizer setup complete. Enabled sensors: {enabled_sensors}")
        self.get_logger().info(
            f"Sync tolerance: {self.sync_tolerance*1000:.1f}ms, "
            f"Queue size: {self.queue_size}")
    
    def _setup_publishers_services(self):
        """Setup publishers and services."""
        
        # Publisher for synchronized sensor data
        self.sync_data_pub = self.create_publisher(
            SynchronizedSensorData,
            '/adaptive_covariance/synchronized_data',
            qos_profile=self.odom_qos
        )
        
        # Publisher for sync status
        self.sync_status_pub = self.create_publisher(
            SensorSyncStatus,
            '/adaptive_covariance/sync_status',
            qos_profile=self.odom_qos
        )
        
        # Service for querying sync status
        self.sync_status_service = self.create_service(
            GetSyncStatus,
            '/adaptive_covariance/get_sync_status',
            self._get_sync_status_callback
        )
    
    def _setup_timers(self):
        """Setup monitoring and status timers."""
        
        # Status monitoring timer
        monitoring_rate = self.get_parameter('monitoring_rate_hz').value
        self.status_timer = self.create_timer(
            1.0 / monitoring_rate,
            self._monitor_sensors_callback
        )
        
        # Debug logging timer (if enabled)
        if self.enable_debug:
            self.debug_timer = self.create_timer(
                self.debug_interval,
                self._debug_log_callback
            )
    
    def _synchronized_callback(self, *messages):
        """
        Callback for synchronized sensor messages.
        
        Args:
            *messages: Variable number of sensor messages in order of enabled_sensors
        """
        
        try:
            current_time = time.time()
            
            with self.state_lock:
                self.sync_callback_count += 1
                self.last_sync_time = current_time
            
            # Create synchronized data message
            sync_data = SynchronizedSensorData()
            sync_data.header.stamp = self.get_clock().now().to_msg()
            sync_data.header.frame_id = "sync_monitor"
            
            # Process messages by sensor type
            message_dict = dict(zip(self.enabled_sensors, messages))
            
            # Calculate synchronization metrics
            timestamps = []
            delays = {}
            
            for sensor_name, message in message_dict.items():
                # Get message timestamp
                if hasattr(message, 'header'):
                    msg_time = self._stamp_to_float(message.header.stamp)
                    timestamps.append(msg_time)
                    
                    # Calculate delay
                    delay = current_time - msg_time
                    delays[sensor_name] = delay
                    
                    # Update sensor monitoring
                    with self.state_lock:
                        self.sensor_delays[sensor_name].append(delay)
                        if len(self.sensor_delays[sensor_name]) > 100:  # Keep last 100
                            self.sensor_delays[sensor_name].popleft()
                        
                        self.sensor_last_seen[sensor_name] = current_time
                        self.sensor_message_count[sensor_name] += 1
                        
                        # Update sensor status based on delay
                        if delay > self.max_delay_critical:
                            self.sensor_status[sensor_name] = 'critical'
                        elif delay > self.max_delay_warning:
                            self.sensor_status[sensor_name] = 'warning'
                        else:
                            self.sensor_status[sensor_name] = 'ok'
            
            # Calculate sync quality metrics
            if len(timestamps) > 1:
                time_spread = max(timestamps) - min(timestamps)
                sync_data.sync_quality = max(0.0, 1.0 - (time_spread / self.sync_tolerance))
            else:
                sync_data.sync_quality = 1.0
            
            sync_data.time_spread_ms = time_spread * 1000.0 if len(timestamps) > 1 else 0.0
            sync_data.sensor_count = len(message_dict)
            
            # Pack sensor data based on enabled sensors
            if 'visual' in message_dict:
                sync_data.visual_odom = message_dict['visual']
                sync_data.has_visual = True
            
            if 'lidar' in message_dict:
                sync_data.lidar_odom = message_dict['lidar']
                sync_data.has_lidar = True
            
            if 'gnss' in message_dict:
                sync_data.gnss_odom = message_dict['gnss']
                sync_data.has_gnss = True
            
            if 'imu' in message_dict:
                sync_data.imu_data = message_dict['imu']
                sync_data.has_imu = True
            
            if 'encoder' in message_dict:
                sync_data.encoder_odom = message_dict['encoder']
                sync_data.has_encoder = True
            
            # Add delay information
            sync_data.sensor_delays = [delays.get(sensor, 0.0) for sensor in self.enabled_sensors]
            sync_data.sensor_names = self.enabled_sensors
            
            # Publish synchronized data
            self.sync_data_pub.publish(sync_data)
            
            # Debug logging
            if self.enable_debug:
                self.get_logger().debug(
                    f"Sync callback #{self.sync_callback_count}: "
                    f"sensors={len(message_dict)}, "
                    f"spread={time_spread*1000:.1f}ms, "
                    f"quality={sync_data.sync_quality:.3f}")
        
        except Exception as e:
            self.get_logger().error(f"Error in synchronized callback: {str(e)}")
    
    def _monitor_sensors_callback(self):
        """Monitor sensor health and publish status."""
        
        try:
            current_time = time.time()
            
            # Create status message
            status_msg = SensorSyncStatus()
            status_msg.header.stamp = self.get_clock().now().to_msg()
            status_msg.header.frame_id = "sync_monitor"
            
            with self.state_lock:
                # Overall sync status
                status_msg.sync_callback_count = self.sync_callback_count
                status_msg.last_sync_age_s = (
                    current_time - self.last_sync_time if self.last_sync_time else -1.0)
                
                # Check for missing sensors
                active_sensors = []
                missing_sensors = []
                
                for sensor_name in self.enabled_sensors:
                    last_seen = self.sensor_last_seen.get(sensor_name, 0.0)
                    age = current_time - last_seen if last_seen > 0 else float('inf')
                    
                    if age > self.sensor_timeout:
                        self.sensor_status[sensor_name] = 'missing'
                        missing_sensors.append(sensor_name)
                    else:
                        active_sensors.append(sensor_name)
                
                status_msg.active_sensors = active_sensors
                status_msg.missing_sensors = missing_sensors
                status_msg.active_sensor_count = len(active_sensors)
                
                # Calculate average delays and status
                sensor_delays_avg = []
                sensor_statuses = []
                
                for sensor_name in self.enabled_sensors:
                    delays = list(self.sensor_delays.get(sensor_name, []))
                    avg_delay = np.mean(delays) if delays else 0.0
                    sensor_delays_avg.append(avg_delay)
                    sensor_statuses.append(self.sensor_status.get(sensor_name, 'unknown'))
                
                status_msg.sensor_names = self.enabled_sensors
                status_msg.sensor_delays_avg = sensor_delays_avg
                status_msg.sensor_statuses = sensor_statuses
                
                # Overall system status
                if len(active_sensors) < self.min_sensors_required:
                    status_msg.overall_status = 'critical'
                elif missing_sensors:
                    status_msg.overall_status = 'warning'
                elif any(status in ['critical', 'warning'] for status in sensor_statuses):
                    status_msg.overall_status = 'degraded'
                else:
                    status_msg.overall_status = 'ok'
                
                # Calculate success rate (approximation)
                total_messages = sum(self.sensor_message_count.values())
                if total_messages > 0:
                    self.sync_success_rate = (
                        self.sync_callback_count * len(self.enabled_sensors) / 
                        max(total_messages, 1))
                status_msg.sync_success_rate = self.sync_success_rate
            
            # Publish status
            self.sync_status_pub.publish(status_msg)
            
            # Log warnings for critical states
            if status_msg.overall_status in ['critical', 'warning']:
                self.get_logger().warn(
                    f"Sync status: {status_msg.overall_status}, "
                    f"active: {len(active_sensors)}/{len(self.enabled_sensors)}, "
                    f"missing: {missing_sensors}")
        
        except Exception as e:
            self.get_logger().error(f"Error in sensor monitoring: {str(e)}")
    
    def _debug_log_callback(self):
        """Debug logging callback."""
        
        try:
            current_time = time.time()
            
            # Avoid too frequent debug logs
            if current_time - self.last_debug_log < self.debug_interval:
                return
            
            with self.state_lock:
                log_parts = [
                    f"Sync Monitor Debug:",
                    f"Callbacks: {self.sync_callback_count}",
                    f"Success rate: {self.sync_success_rate:.3f}",
                ]
                
                for sensor_name in self.enabled_sensors:
                    delays = list(self.sensor_delays.get(sensor_name, []))
                    status = self.sensor_status.get(sensor_name, 'unknown')
                    msg_count = self.sensor_message_count.get(sensor_name, 0)
                    avg_delay = np.mean(delays) if delays else 0.0
                    
                    log_parts.append(
                        f"{sensor_name}: {status} "
                        f"(msgs={msg_count}, delay={avg_delay*1000:.1f}ms)")
            
            self.get_logger().info(" | ".join(log_parts))
            self.last_debug_log = current_time
        
        except Exception as e:
            self.get_logger().error(f"Error in debug logging: {str(e)}")
    
    def _get_sync_status_callback(self, request, response):
        """Service callback for sync status query."""
        
        try:
            current_time = time.time()
            
            with self.state_lock:
                response.success = True
                response.overall_status = self._get_overall_status()
                response.sync_callback_count = self.sync_callback_count
                response.active_sensor_count = len([
                    sensor for sensor in self.enabled_sensors
                    if self.sensor_status.get(sensor, 'missing') != 'missing'
                ])
                response.total_sensor_count = len(self.enabled_sensors)
                response.sync_success_rate = self.sync_success_rate
                
                # Calculate max delay
                max_delay = 0.0
                for sensor_name in self.enabled_sensors:
                    delays = list(self.sensor_delays.get(sensor_name, []))
                    if delays:
                        sensor_max_delay = max(delays)
                        max_delay = max(max_delay, sensor_max_delay)
                
                response.max_delay_ms = max_delay * 1000.0
                response.last_sync_age_s = (
                    current_time - self.last_sync_time if self.last_sync_time else -1.0)
                
                # Detailed sensor status
                sensor_details = []
                for sensor_name in self.enabled_sensors:
                    status = self.sensor_status.get(sensor_name, 'unknown')
                    msg_count = self.sensor_message_count.get(sensor_name, 0)
                    delays = list(self.sensor_delays.get(sensor_name, []))
                    avg_delay = np.mean(delays) if delays else 0.0
                    
                    detail = (f"{sensor_name}: {status} "
                             f"(msgs={msg_count}, avg_delay={avg_delay*1000:.1f}ms)")
                    sensor_details.append(detail)
                
                response.sensor_details = sensor_details
                response.message = f"Sync monitor operational with {response.active_sensor_count}/{response.total_sensor_count} sensors"
        
        except Exception as e:
            response.success = False
            response.message = f"Error getting sync status: {str(e)}"
            self.get_logger().error(response.message)
        
        return response
    
    def _get_overall_status(self) -> str:
        """Calculate overall system status."""
        
        active_count = 0
        critical_count = 0
        warning_count = 0
        
        for sensor_name in self.enabled_sensors:
            status = self.sensor_status.get(sensor_name, 'missing')
            if status != 'missing':
                active_count += 1
                if status == 'critical':
                    critical_count += 1
                elif status == 'warning':
                    warning_count += 1
        
        if active_count < self.min_sensors_required:
            return 'critical'
        elif critical_count > 0:
            return 'critical'
        elif warning_count > 0 or active_count < len(self.enabled_sensors):
            return 'warning'
        else:
            return 'ok'
    
    @staticmethod
    def _stamp_to_float(stamp: Time) -> float:
        """Convert ROS timestamp to float seconds."""
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main(args=None):
    """Main entry point."""
    
    rclpy.init(args=args)
    
    try:
        sync_monitor = SyncMonitor()
        rclpy.spin(sync_monitor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in sync monitor: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

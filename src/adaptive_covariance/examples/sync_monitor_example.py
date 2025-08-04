#!/usr/bin/env python3

"""
Sync Monitor Example Usage
==========================

This script demonstrates how to use the synchronized sensor data from the sync monitor
in the adaptive covariance system. It shows how to receive temporally aligned sensor
data and perform coordinated analysis.

Usage:
    # Terminal 1: Start sync monitor
    ros2 launch adaptive_covariance sync_monitor.launch.py config:=debug
    
    # Terminal 2: Start test data publisher
    python3 test_sync_monitor.py
    
    # Terminal 3: Run this example
    python3 sync_monitor_example.py

Author: ROS 2 Odometry Workspace
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from adaptive_covariance_interfaces.msg import SynchronizedSensorData, SensorSyncStatus
from adaptive_covariance_interfaces.srv import GetSyncStatus

import numpy as np
import time
from typing import Dict, List, Optional


class SyncMonitorExample(Node):
    """
    Example usage of the sync monitor for coordinated sensor analysis.
    
    This demonstrates how to receive and process synchronized sensor data
    for tasks like drift detection, sensor validation, and adaptive fusion.
    """
    
    def __init__(self):
        super().__init__('sync_monitor_example')
        
        # Setup QoS profile
        self.qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscribe to synchronized data
        self.sync_data_sub = self.create_subscription(
            SynchronizedSensorData,
            '/adaptive_covariance/synchronized_data',
            self.synchronized_data_callback,
            self.qos_profile
        )
        
        # Subscribe to sync status
        self.sync_status_sub = self.create_subscription(
            SensorSyncStatus,
            '/adaptive_covariance/sync_status',
            self.sync_status_callback,
            self.qos_profile
        )
        
        # Service client for sync status queries
        self.sync_status_client = self.create_client(
            GetSyncStatus,
            '/adaptive_covariance/get_sync_status'
        )
        
        # Initialize analysis state
        self.sync_data_count = 0
        self.last_analysis_time = time.time()
        self.position_history = {}  # Store position history per sensor
        self.drift_analysis = {}    # Store drift analysis results
        
        # Analysis parameters
        self.analysis_interval = 5.0  # Analyze every 5 seconds
        self.max_history_length = 100  # Keep last 100 measurements per sensor
        
        # Setup analysis timer
        self.analysis_timer = self.create_timer(
            self.analysis_interval,
            self.periodic_analysis_callback
        )
        
        self.get_logger().info("Sync monitor example started")
        self.get_logger().info("Waiting for synchronized sensor data...")
    
    def synchronized_data_callback(self, msg: SynchronizedSensorData):
        """
        Process synchronized sensor data.
        
        This is the key callback where all sensor data is temporally aligned,
        allowing for accurate cross-sensor analysis and comparison.
        """
        
        self.sync_data_count += 1
        current_time = time.time()
        
        # Extract synchronized sensor data
        sensor_positions = {}
        sensor_orientations = {}
        
        if msg.has_visual and msg.visual_odom:
            pos = msg.visual_odom.pose.pose.position
            ori = msg.visual_odom.pose.pose.orientation
            sensor_positions['visual'] = np.array([pos.x, pos.y, pos.z])
            sensor_orientations['visual'] = np.array([ori.x, ori.y, ori.z, ori.w])
        
        if msg.has_lidar and msg.lidar_odom:
            pos = msg.lidar_odom.pose.pose.position
            ori = msg.lidar_odom.pose.pose.orientation
            sensor_positions['lidar'] = np.array([pos.x, pos.y, pos.z])
            sensor_orientations['lidar'] = np.array([ori.x, ori.y, ori.z, ori.w])
        
        if msg.has_gnss and msg.gnss_odom:
            pos = msg.gnss_odom.pose.pose.position
            ori = msg.gnss_odom.pose.pose.orientation
            sensor_positions['gnss'] = np.array([pos.x, pos.y, pos.z])
            sensor_orientations['gnss'] = np.array([ori.x, ori.y, ori.z, ori.w])
        
        if msg.has_encoder and msg.encoder_odom:
            pos = msg.encoder_odom.pose.pose.position
            ori = msg.encoder_odom.pose.pose.orientation
            sensor_positions['encoder'] = np.array([pos.x, pos.y, pos.z])
            sensor_orientations['encoder'] = np.array([ori.x, ori.y, ori.z, ori.w])
        
        # Store position history for trend analysis
        for sensor_name, position in sensor_positions.items():
            if sensor_name not in self.position_history:
                self.position_history[sensor_name] = []
            
            self.position_history[sensor_name].append({
                'timestamp': current_time,
                'position': position,
                'sync_quality': msg.sync_quality
            })
            
            # Limit history length
            if len(self.position_history[sensor_name]) > self.max_history_length:
                self.position_history[sensor_name].pop(0)
        
        # Perform immediate cross-sensor analysis if we have multiple sensors
        if len(sensor_positions) >= 2:
            self.analyze_sensor_agreement(sensor_positions, msg.sync_quality)
        
        # Log periodic summary
        if self.sync_data_count % 100 == 0:
            self.get_logger().info(
                f"Processed {self.sync_data_count} synchronized measurements. "
                f"Current sensors: {list(sensor_positions.keys())}, "
                f"Sync quality: {msg.sync_quality:.3f}")
    
    def sync_status_callback(self, msg: SensorSyncStatus):
        """Process sync status updates."""
        
        # Log significant status changes
        if msg.overall_status in ['warning', 'critical']:
            missing_str = ', '.join(msg.missing_sensors) if msg.missing_sensors else 'none'
            self.get_logger().warn(
                f"Sync status: {msg.overall_status} | "
                f"Active: {msg.active_sensor_count}/{len(msg.sensor_names)} | "
                f"Missing: {missing_str}")
        
        # Update analysis based on sync health
        if msg.sync_success_rate < 0.8:
            self.get_logger().warn(
                f"Low sync success rate: {msg.sync_success_rate:.3f}. "
                "Consider increasing sync tolerance or checking sensor health.")
    
    def analyze_sensor_agreement(self, sensor_positions: Dict[str, np.ndarray], sync_quality: float):
        """
        Analyze agreement between different sensors at the same timestamp.
        
        This is only possible because the sync monitor ensures temporal alignment.
        """
        
        sensor_names = list(sensor_positions.keys())
        
        # Calculate pairwise distances
        for i, sensor1 in enumerate(sensor_names):
            for j, sensor2 in enumerate(sensor_names[i+1:], i+1):
                distance = np.linalg.norm(
                    sensor_positions[sensor1] - sensor_positions[sensor2])
                
                # Store drift analysis
                pair_key = f"{sensor1}_{sensor2}"
                if pair_key not in self.drift_analysis:
                    self.drift_analysis[pair_key] = []
                
                self.drift_analysis[pair_key].append({
                    'timestamp': time.time(),
                    'distance': distance,
                    'sync_quality': sync_quality
                })
                
                # Alert on significant disagreement
                if distance > 1.0:  # 1 meter threshold
                    self.get_logger().warn(
                        f"Large position disagreement between {sensor1} and {sensor2}: "
                        f"{distance:.2f}m (sync_quality: {sync_quality:.3f})")
    
    def periodic_analysis_callback(self):
        """Perform periodic analysis of accumulated data."""
        
        current_time = time.time()
        
        # Analyze position trends
        self.analyze_position_trends()
        
        # Analyze drift patterns
        self.analyze_drift_patterns()
        
        # Query sync status
        self.query_sync_status()
        
        self.last_analysis_time = current_time
    
    def analyze_position_trends(self):
        """Analyze position trends for each sensor."""
        
        for sensor_name, history in self.position_history.items():
            if len(history) < 10:  # Need minimum data points
                continue
            
            # Calculate velocity estimates
            recent_history = history[-10:]  # Last 10 measurements
            
            positions = np.array([h['position'] for h in recent_history])
            timestamps = np.array([h['timestamp'] for h in recent_history])
            
            # Simple velocity estimation
            dt = timestamps[-1] - timestamps[0]
            if dt > 0:
                velocity = (positions[-1] - positions[0]) / dt
                speed = np.linalg.norm(velocity)
                
                # Store analysis results
                if speed > 10.0:  # 10 m/s threshold
                    self.get_logger().warn(
                        f"{sensor_name} sensor shows high speed: {speed:.2f} m/s")
                
                # Check for sudden position jumps
                for i in range(1, len(positions)):
                    jump = np.linalg.norm(positions[i] - positions[i-1])
                    if jump > 5.0:  # 5 meter jump threshold
                        self.get_logger().warn(
                            f"{sensor_name} sensor shows position jump: {jump:.2f}m")
    
    def analyze_drift_patterns(self):
        """Analyze drift patterns between sensor pairs."""
        
        for pair_key, drift_history in self.drift_analysis.items():
            if len(drift_history) < 5:  # Need minimum data points
                continue
            
            # Calculate drift trend
            recent_drifts = drift_history[-20:]  # Last 20 measurements
            distances = [d['distance'] for d in recent_drifts]
            
            mean_distance = np.mean(distances)
            std_distance = np.std(distances)
            
            # Alert on increasing drift
            if mean_distance > 2.0:  # 2 meter average drift
                self.get_logger().warn(
                    f"High average drift for {pair_key}: "
                    f"{mean_distance:.2f}±{std_distance:.2f}m")
            
            # Check for drift trend
            if len(distances) >= 10:
                early_distances = distances[:5]
                late_distances = distances[-5:]
                
                early_mean = np.mean(early_distances)
                late_mean = np.mean(late_distances)
                
                if late_mean > early_mean * 1.5:  # 50% increase
                    self.get_logger().warn(
                        f"Increasing drift trend for {pair_key}: "
                        f"{early_mean:.2f}m → {late_mean:.2f}m")
    
    def query_sync_status(self):
        """Query sync status via service call."""
        
        if not self.sync_status_client.service_is_ready():
            return
        
        request = GetSyncStatus.Request()
        
        future = self.sync_status_client.call_async(request)
        future.add_done_callback(self.sync_status_response_callback)
    
    def sync_status_response_callback(self, future):
        """Handle sync status service response."""
        
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"Sync Status: {response.overall_status} | "
                    f"Active: {response.active_sensor_count}/{response.total_sensor_count} | "
                    f"Success rate: {response.sync_success_rate:.3f} | "
                    f"Max delay: {response.max_delay_ms:.1f}ms")
        except Exception as e:
            self.get_logger().error(f"Failed to get sync status: {e}")


def main(args=None):
    """Main entry point."""
    
    rclpy.init(args=args)
    
    try:
        example_node = SyncMonitorExample()
        
        rclpy.spin(example_node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in sync monitor example: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

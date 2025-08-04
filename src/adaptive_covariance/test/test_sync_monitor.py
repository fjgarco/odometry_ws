#!/usr/bin/env python3

"""
Test script for Sync Monitor
============================

Tests the multi-sensor synchronization monitor with simulated sensor data.
Useful for validating sync behavior before deploying on real robot hardware.

Usage:
    # In terminal 1 - Start sync monitor
    ros2 launch adaptive_covariance sync_monitor.launch.py config:=debug
    
    # In terminal 2 - Run test publisher
    python3 test_sync_monitor.py

Author: ROS 2 Odometry Workspace
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
from std_msgs.msg import Header

import numpy as np
import threading
import time
from typing import Dict, List


class SyncMonitorTestPublisher(Node):
    """
    Test publisher for sync monitor validation.
    
    Publishes simulated sensor data with configurable timing patterns
    to test synchronization behavior under different conditions.
    """
    
    def __init__(self):
        super().__init__('sync_monitor_test_publisher')
        
        # Declare parameters
        self.declare_parameter('test_duration_s', 60.0)
        self.declare_parameter('base_frequency_hz', 30.0)
        self.declare_parameter('add_timing_jitter', True)
        self.declare_parameter('add_sensor_dropouts', True)
        self.declare_parameter('simulate_high_delays', False)
        
        # Get parameters
        self.test_duration = self.get_parameter('test_duration_s').value
        self.base_freq = self.get_parameter('base_frequency_hz').value
        self.add_jitter = self.get_parameter('add_timing_jitter').value
        self.add_dropouts = self.get_parameter('add_sensor_dropouts').value
        self.simulate_delays = self.get_parameter('simulate_high_delays').value
        
        # Setup QoS
        self.odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.imu_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Create publishers
        self.publishers = {
            'visual': self.create_publisher(Odometry, '/zed/odom', self.odom_qos),
            'lidar': self.create_publisher(Odometry, '/odometry/lidar', self.odom_qos),
            'gnss': self.create_publisher(Odometry, '/odometry/gps', self.odom_qos),
            'imu': self.create_publisher(Imu, '/mavros/imu/data', self.imu_qos),
            'encoder': self.create_publisher(Odometry, '/odometry/encoder', self.odom_qos)
        }
        
        # Initialize simulation state
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([1.0, 0.5, 0.0])  # Constant velocity
        self.orientation = np.array([0.0, 0.0, 0.0])  # Roll, pitch, yaw
        self.angular_velocity = np.array([0.0, 0.0, 0.1])  # Slow rotation
        
        # Sensor-specific parameters
        self.sensor_frequencies = {
            'visual': self.base_freq,
            'lidar': self.base_freq * 0.8,    # Slightly slower
            'gnss': self.base_freq * 0.3,     # Much slower (typical for GPS)
            'imu': self.base_freq * 3.0,      # Much faster (typical for IMU)
            'encoder': self.base_freq * 1.2   # Slightly faster
        }
        
        # Timing and dropout simulation
        self.sensor_next_pub = {sensor: time.time() for sensor in self.publishers.keys()}
        self.sensor_dropout_until = {sensor: 0.0 for sensor in self.publishers.keys()}
        
        # Start publishing
        self.start_time = time.time()
        self.running = True
        self.publish_thread = threading.Thread(target=self._publish_loop)
        self.publish_thread.start()
        
        self.get_logger().info(f"Test publisher started - Duration: {self.test_duration}s")
        self.get_logger().info(f"Base frequency: {self.base_freq}Hz")
        self.get_logger().info(f"Jitter: {self.add_jitter}, Dropouts: {self.add_dropouts}")
    
    def _publish_loop(self):
        """Main publishing loop with timing simulation."""
        
        message_count = 0
        
        while self.running and (time.time() - self.start_time) < self.test_duration:
            current_time = time.time()
            
            # Update simulation state
            dt = 0.01  # 10ms update rate
            self._update_simulation_state(dt)
            
            # Check each sensor for publishing
            for sensor_name, publisher in self.publishers.items():
                if current_time >= self.sensor_next_pub[sensor_name]:
                    
                    # Check for dropout simulation
                    if self.add_dropouts and current_time < self.sensor_dropout_until[sensor_name]:
                        # Skip this publication due to simulated dropout
                        self._schedule_next_publication(sensor_name, current_time)
                        continue
                    
                    # Create and publish message
                    if sensor_name == 'imu':
                        msg = self._create_imu_message()
                    else:
                        msg = self._create_odometry_message(sensor_name)
                    
                    # Add simulated delay if enabled
                    if self.simulate_delays and np.random.random() < 0.1:  # 10% chance
                        delay = np.random.uniform(0.1, 0.3)  # 100-300ms delay
                        msg_time = self.get_clock().now().to_msg()
                        msg_time.sec -= int(delay)
                        msg_time.nanosec -= int((delay % 1.0) * 1e9)
                        if msg_time.nanosec < 0:
                            msg_time.sec -= 1
                            msg_time.nanosec += int(1e9)
                        msg.header.stamp = msg_time
                    
                    publisher.publish(msg)
                    message_count += 1
                    
                    # Schedule next publication
                    self._schedule_next_publication(sensor_name, current_time)
                    
                    # Simulate occasional dropouts
                    if self.add_dropouts and np.random.random() < 0.01:  # 1% chance per publish
                        dropout_duration = np.random.uniform(0.5, 2.0)  # 0.5-2s dropout
                        self.sensor_dropout_until[sensor_name] = current_time + dropout_duration
                        self.get_logger().warn(
                            f"Simulating {sensor_name} dropout for {dropout_duration:.1f}s")
            
            time.sleep(dt)
        
        self.get_logger().info(f"Test completed. Published {message_count} messages total.")
    
    def _update_simulation_state(self, dt: float):
        """Update simulated robot state."""
        
        # Update position and orientation
        self.position += self.velocity * dt
        self.orientation += self.angular_velocity * dt
        
        # Add some noise for realism
        if self.add_jitter:
            self.position += np.random.normal(0, 0.01, 3) * dt
            self.orientation += np.random.normal(0, 0.05, 3) * dt
    
    def _schedule_next_publication(self, sensor_name: str, current_time: float):
        """Schedule next publication time for a sensor."""
        
        frequency = self.sensor_frequencies[sensor_name]
        base_interval = 1.0 / frequency
        
        # Add jitter if enabled
        if self.add_jitter:
            jitter = np.random.normal(0, base_interval * 0.1)  # 10% jitter
            interval = base_interval + jitter
        else:
            interval = base_interval
        
        self.sensor_next_pub[sensor_name] = current_time + max(interval, 0.001)  # Min 1ms
    
    def _create_odometry_message(self, sensor_name: str) -> Odometry:
        """Create odometry message for specified sensor."""
        
        msg = Odometry()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"{sensor_name}_odom"
        msg.child_frame_id = "base_link"
        
        # Position
        msg.pose.pose.position = Point(
            x=self.position[0], 
            y=self.position[1], 
            z=self.position[2]
        )
        
        # Orientation (convert from Euler to quaternion)
        q = self._euler_to_quaternion(self.orientation)
        msg.pose.pose.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])
        
        # Velocity
        msg.twist.twist.linear = Vector3(
            x=self.velocity[0],
            y=self.velocity[1], 
            z=self.velocity[2]
        )
        msg.twist.twist.angular = Vector3(
            x=self.angular_velocity[0],
            y=self.angular_velocity[1],
            z=self.angular_velocity[2]
        )
        
        # Add sensor-specific noise and biases
        noise_levels = {
            'visual': 0.02,
            'lidar': 0.01,
            'gnss': 0.5,    # GPS has higher position noise
            'encoder': 0.05
        }
        
        if sensor_name in noise_levels:
            noise = noise_levels[sensor_name]
            msg.pose.pose.position.x += np.random.normal(0, noise)
            msg.pose.pose.position.y += np.random.normal(0, noise)
        
        # Covariance (simplified)
        base_cov = noise_levels.get(sensor_name, 0.1) ** 2
        msg.pose.covariance[0] = base_cov   # x
        msg.pose.covariance[7] = base_cov   # y
        msg.pose.covariance[14] = base_cov  # z
        msg.pose.covariance[35] = (base_cov * 10)  # yaw
        
        return msg
    
    def _create_imu_message(self) -> Imu:
        """Create IMU message."""
        
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"
        
        # Orientation
        q = self._euler_to_quaternion(self.orientation)
        msg.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])
        
        # Angular velocity
        msg.angular_velocity = Vector3(
            x=self.angular_velocity[0] + np.random.normal(0, 0.01),
            y=self.angular_velocity[1] + np.random.normal(0, 0.01),
            z=self.angular_velocity[2] + np.random.normal(0, 0.01)
        )
        
        # Linear acceleration (simplified)
        msg.linear_acceleration = Vector3(
            x=np.random.normal(0, 0.1),
            y=np.random.normal(0, 0.1), 
            z=9.81 + np.random.normal(0, 0.1)  # Gravity + noise
        )
        
        # Covariance
        msg.orientation_covariance[0] = 0.01
        msg.orientation_covariance[4] = 0.01
        msg.orientation_covariance[8] = 0.02
        msg.angular_velocity_covariance[0] = 0.001
        msg.angular_velocity_covariance[4] = 0.001
        msg.angular_velocity_covariance[8] = 0.001
        msg.linear_acceleration_covariance[0] = 0.01
        msg.linear_acceleration_covariance[4] = 0.01
        msg.linear_acceleration_covariance[8] = 0.01
        
        return msg
    
    @staticmethod
    def _euler_to_quaternion(euler):
        """Convert Euler angles to quaternion [w, x, y, z]."""
        
        roll, pitch, yaw = euler
        
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return [w, x, y, z]
    
    def shutdown(self):
        """Shutdown the test publisher."""
        self.running = False
        if self.publish_thread.is_alive():
            self.publish_thread.join()


def main(args=None):
    """Main entry point."""
    
    rclpy.init(args=args)
    
    try:
        test_publisher = SyncMonitorTestPublisher()
        
        # Run for specified duration
        duration = test_publisher.get_parameter('test_duration_s').value
        
        def shutdown_callback():
            test_publisher.get_logger().info("Shutting down test publisher...")
            test_publisher.shutdown()
        
        # Setup shutdown timer
        test_publisher.create_timer(duration, shutdown_callback)
        
        rclpy.spin(test_publisher)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in test publisher: {e}")
    finally:
        if 'test_publisher' in locals():
            test_publisher.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

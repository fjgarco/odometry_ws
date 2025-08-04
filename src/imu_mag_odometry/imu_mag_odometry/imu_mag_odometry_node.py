#!/usr/bin/env python3
"""
IMU and Magnetometer Odometry Node
Estimates orientation and optionally position/velocity from IMU, magnetometer and barometer data
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import math
from collections import deque
import time

# ROS messages
from sensor_msgs.msg import Imu, MagneticField, FluidPressure
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovariance, TwistWithCovariance
from std_msgs.msg import Header
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

# Local imports
from .madgwick_filter import MadgwickFilter


class ImuMagOdometry(Node):
    """
    ROS 2 node for IMU and magnetometer odometry estimation
    """
    
    def __init__(self):
        super().__init__('imu_mag_odometry')
        
        # Declare and get parameters
        self.declare_parameters()
        self.get_parameters()
        
        # Initialize filter
        self.filter = MadgwickFilter(
            sample_rate=self.sample_rate,
            beta=self.madgwick_beta
        )
        
        # State variables
        self.last_imu_time = None
        self.last_pressure = None
        self.reference_pressure = None
        self.velocity = np.zeros(3)  # [vx, vy, vz]
        self.position = np.zeros(3)  # [x, y, z]
        self.last_accel_world = np.zeros(3)
        
        # Data buffers for integration
        self.accel_buffer = deque(maxlen=10)
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            sensor_qos
        )
        
        self.mag_sub = self.create_subscription(
            MagneticField,
            self.mag_topic,
            self.mag_callback,
            sensor_qos
        )
        
        if self.use_barometer:
            self.pressure_sub = self.create_subscription(
                FluidPressure,
                self.pressure_topic,
                self.pressure_callback,
                sensor_qos
            )
        
        # Publishers
        self.odom_pub = self.create_publisher(
            Odometry,
            self.output_topic,
            10
        )
        
        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Data storage
        self.latest_imu = None
        self.latest_mag = None
        self.latest_pressure = None
        
        # Timer for publishing at fixed rate
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_odometry)
        
        self.get_logger().info(f"IMU/Mag Odometry node started")
        self.get_logger().info(f"IMU topic: {self.imu_topic}")
        self.get_logger().info(f"Mag topic: {self.mag_topic}")
        self.get_logger().info(f"Output topic: {self.output_topic}")
        self.get_logger().info(f"Filter: {self.orientation_filter}")
        self.get_logger().info(f"Integration enabled: {self.enable_integration}")
        self.get_logger().info(f"Barometer enabled: {self.use_barometer}")
        
    def declare_parameters(self):
        """Declare all ROS parameters with default values"""
        # Topics
        self.declare_parameter('imu_topic', '/mavros/imu/data')
        self.declare_parameter('mag_topic', '/mavros/imu/mag')
        self.declare_parameter('pressure_topic', '/mavros/imu/atm_pressure')
        self.declare_parameter('output_topic', '/odometry/imu_mag')
        
        # Frames
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        
        # Filter settings
        self.declare_parameter('orientation_filter', 'madgwick')  # 'madgwick', 'use_imu_orientation'
        self.declare_parameter('madgwick_beta', 0.1)
        self.declare_parameter('sample_rate', 100.0)
        
        # Integration settings
        self.declare_parameter('enable_integration', False)
        self.declare_parameter('gravity_compensation', True)
        self.declare_parameter('gravity_magnitude', 9.81)
        
        # Barometer settings
        self.declare_parameter('use_barometer', False)
        self.declare_parameter('pressure_to_altitude_factor', 8.3)  # meters per hPa
        
        # Publishing settings
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('publish_tf', True)
        
        # Covariances
        self.declare_parameter('orientation_covariance', 0.01)
        self.declare_parameter('position_covariance', 1.0)
        self.declare_parameter('velocity_covariance', 0.1)
        
    def get_parameters(self):
        """Get all parameters from ROS parameter server"""
        # Topics
        self.imu_topic = self.get_parameter('imu_topic').value
        self.mag_topic = self.get_parameter('mag_topic').value
        self.pressure_topic = self.get_parameter('pressure_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        
        # Frames
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        
        # Filter settings
        self.orientation_filter = self.get_parameter('orientation_filter').value
        self.madgwick_beta = self.get_parameter('madgwick_beta').value
        self.sample_rate = self.get_parameter('sample_rate').value
        
        # Integration settings
        self.enable_integration = self.get_parameter('enable_integration').value
        self.gravity_compensation = self.get_parameter('gravity_compensation').value
        self.gravity_magnitude = self.get_parameter('gravity_magnitude').value
        
        # Barometer settings
        self.use_barometer = self.get_parameter('use_barometer').value
        self.pressure_to_altitude_factor = self.get_parameter('pressure_to_altitude_factor').value
        
        # Publishing settings
        self.publish_rate = self.get_parameter('publish_rate').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        # Covariances
        self.orientation_covariance = self.get_parameter('orientation_covariance').value
        self.position_covariance = self.get_parameter('position_covariance').value
        self.velocity_covariance = self.get_parameter('velocity_covariance').value
        
    def imu_callback(self, msg):
        """Callback for IMU data"""
        self.latest_imu = msg
        
        if self.orientation_filter == 'madgwick':
            # Extract data
            accel = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
            gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
            
            # Update filter
            if self.latest_mag is not None:
                # Use magnetometer data
                mag = [self.latest_mag.magnetic_field.x, 
                       self.latest_mag.magnetic_field.y, 
                       self.latest_mag.magnetic_field.z]
                self.filter.update_marg(accel, gyro, mag)
            else:
                # IMU only
                self.filter.update_imu(accel, gyro)
        
        # Integration for position/velocity estimation
        if self.enable_integration:
            self.integrate_acceleration(msg)
            
    def mag_callback(self, msg):
        """Callback for magnetometer data"""
        self.latest_mag = msg
        
    def pressure_callback(self, msg):
        """Callback for pressure/barometer data"""
        self.latest_pressure = msg
        
        # Set reference pressure on first measurement
        if self.reference_pressure is None:
            self.reference_pressure = msg.fluid_pressure
            self.get_logger().info(f"Reference pressure set to: {self.reference_pressure:.2f} Pa")
            
    def integrate_acceleration(self, imu_msg):
        """Integrate acceleration to estimate velocity and position"""
        current_time = self.get_clock().now()
        
        if self.last_imu_time is None:
            self.last_imu_time = current_time
            return
            
        dt = (current_time - self.last_imu_time).nanoseconds / 1e9
        if dt <= 0 or dt > 1.0:  # Sanity check
            self.last_imu_time = current_time
            return
            
        # Get acceleration in body frame
        accel_body = np.array([
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z
        ])
        
        # Transform to world frame using current orientation
        if self.orientation_filter == 'madgwick':
            quat = self.filter.get_quaternion()
        else:
            # Use IMU orientation
            quat = np.array([
                imu_msg.orientation.w,
                imu_msg.orientation.x,
                imu_msg.orientation.y,
                imu_msg.orientation.z
            ])
            
        accel_world = self.rotate_vector_by_quaternion(accel_body, quat)
        
        # Gravity compensation
        if self.gravity_compensation:
            accel_world[2] -= self.gravity_magnitude
            
        # Simple integration (trapezoid rule)
        accel_avg = (accel_world + self.last_accel_world) * 0.5
        self.velocity += accel_avg * dt
        self.position += self.velocity * dt
        
        self.last_accel_world = accel_world
        self.last_imu_time = current_time
        
    def rotate_vector_by_quaternion(self, vector, quaternion):
        """Rotate a vector by a quaternion"""
        w, x, y, z = quaternion
        vx, vy, vz = vector
        
        # Quaternion rotation formula
        # v' = v + 2 * r × (r × v + w * v)
        # where r = [x, y, z] (imaginary part of quaternion)
        
        r = np.array([x, y, z])
        v = np.array([vx, vy, vz])
        
        cross1 = np.cross(r, v)
        cross2 = np.cross(r, cross1 + w * v)
        
        return v + 2 * cross2
        
    def publish_odometry(self):
        """Publish odometry message at fixed rate"""
        if self.latest_imu is None:
            return
            
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        
        # Get orientation
        if self.orientation_filter == 'madgwick':
            quat = self.filter.get_quaternion()
            odom_msg.pose.pose.orientation.w = quat[0]
            odom_msg.pose.pose.orientation.x = quat[1]
            odom_msg.pose.pose.orientation.y = quat[2]
            odom_msg.pose.pose.orientation.z = quat[3]
        else:
            # Use IMU orientation directly
            odom_msg.pose.pose.orientation = self.latest_imu.orientation
            
        # Set position
        if self.enable_integration:
            odom_msg.pose.pose.position.x = self.position[0]
            odom_msg.pose.pose.position.y = self.position[1]
            
            # Use barometer for Z if available
            if self.use_barometer and self.latest_pressure is not None and self.reference_pressure is not None:
                pressure_diff = self.reference_pressure - self.latest_pressure.fluid_pressure
                altitude = pressure_diff / 100.0 * self.pressure_to_altitude_factor  # Convert Pa to hPa then to meters
                odom_msg.pose.pose.position.z = altitude
            else:
                odom_msg.pose.pose.position.z = self.position[2]
        else:
            # No position estimation
            odom_msg.pose.pose.position.x = 0.0
            odom_msg.pose.pose.position.y = 0.0
            odom_msg.pose.pose.position.z = 0.0
            
        # Set velocity
        if self.enable_integration:
            odom_msg.twist.twist.linear.x = self.velocity[0]
            odom_msg.twist.twist.linear.y = self.velocity[1]
            odom_msg.twist.twist.linear.z = self.velocity[2]
            
            # Angular velocity from IMU
            odom_msg.twist.twist.angular = self.latest_imu.angular_velocity
        else:
            # No velocity estimation
            odom_msg.twist.twist.linear.x = 0.0
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.linear.z = 0.0
            odom_msg.twist.twist.angular.x = 0.0
            odom_msg.twist.twist.angular.y = 0.0
            odom_msg.twist.twist.angular.z = 0.0
            
        # Set covariances
        self.set_covariances(odom_msg)
        
        # Publish
        self.odom_pub.publish(odom_msg)
        
        # Publish TF
        if self.publish_tf:
            self.publish_transform(odom_msg)
            
    def set_covariances(self, odom_msg):
        """Set covariance matrices for odometry message"""
        # Pose covariance (6x6)
        pose_cov = [0.0] * 36
        
        # Position covariances
        if self.enable_integration:
            pose_cov[0] = self.position_covariance   # x
            pose_cov[7] = self.position_covariance   # y
            pose_cov[14] = self.position_covariance  # z
        else:
            pose_cov[0] = 999999.0   # x - unknown
            pose_cov[7] = 999999.0   # y - unknown
            pose_cov[14] = 999999.0  # z - unknown
            
        # Orientation covariances
        pose_cov[21] = self.orientation_covariance   # roll
        pose_cov[28] = self.orientation_covariance   # pitch
        pose_cov[35] = self.orientation_covariance   # yaw
        
        odom_msg.pose.covariance = pose_cov
        
        # Twist covariance (6x6)
        twist_cov = [0.0] * 36
        
        if self.enable_integration:
            twist_cov[0] = self.velocity_covariance   # vx
            twist_cov[7] = self.velocity_covariance   # vy
            twist_cov[14] = self.velocity_covariance  # vz
            twist_cov[21] = self.velocity_covariance  # wx
            twist_cov[28] = self.velocity_covariance  # wy
            twist_cov[35] = self.velocity_covariance  # wz
        else:
            twist_cov[0] = 999999.0   # vx - unknown
            twist_cov[7] = 999999.0   # vy - unknown
            twist_cov[14] = 999999.0  # vz - unknown
            twist_cov[21] = 999999.0  # wx - unknown
            twist_cov[28] = 999999.0  # wy - unknown
            twist_cov[35] = 999999.0  # wz - unknown
            
        odom_msg.twist.covariance = twist_cov
        
    def publish_transform(self, odom_msg):
        """Publish TF transform"""
        t = TransformStamped()
        t.header = odom_msg.header
        t.child_frame_id = odom_msg.child_frame_id
        
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        t.transform.rotation = odom_msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    
    node = ImuMagOdometry()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

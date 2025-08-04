#!/usr/bin/env python3
"""
GPS Odometry Node
Provides absolute position estimation from GNSS RTK data using NavSatFix messages
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import math
from typing import Optional, Tuple

# ROS messages
from sensor_msgs.msg import NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovariance, TwistWithCovariance
from std_msgs.msg import Header
import tf2_ros

# Local imports
from .geodetic_converter import GeodeticConverter


class GpsOdometry(Node):
    """
    ROS 2 node for GPS/GNSS RTK odometry estimation
    """
    
    def __init__(self):
        super().__init__('gps_odometry')
        
        # Declare and get parameters
        self.declare_parameters()
        self.get_parameters()
        
        # Initialize geodetic converter
        self.converter = GeodeticConverter(self.coordinate_system)
        
        # State variables
        self.origin_set = False
        self.last_valid_fix = None
        self.fix_count = 0
        self.invalid_count = 0
        
        # Set fixed origin if provided
        if self.fixed_origin_lat is not None and self.fixed_origin_lon is not None:
            self.converter.set_origin(
                self.fixed_origin_lat, 
                self.fixed_origin_lon, 
                self.fixed_origin_alt
            )
            self.origin_set = True
            self.get_logger().info(
                f"Fixed origin set to: {self.fixed_origin_lat:.8f}, "
                f"{self.fixed_origin_lon:.8f}, {self.fixed_origin_alt:.2f}"
            )
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriber
        self.gps_sub = self.create_subscription(
            NavSatFix,
            self.gps_topic,
            self.gps_callback,
            sensor_qos
        )
        
        # Publisher
        self.odom_pub = self.create_publisher(
            Odometry,
            self.output_topic,
            10
        )
        
        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Timer for publishing statistics
        if self.publish_stats:
            self.stats_timer = self.create_timer(10.0, self.publish_statistics)
        
        self.get_logger().info(f"GPS Odometry node started")
        self.get_logger().info(f"GPS topic: {self.gps_topic}")
        self.get_logger().info(f"Output topic: {self.output_topic}")
        self.get_logger().info(f"Coordinate system: {self.coordinate_system}")
        self.get_logger().info(f"Origin mode: {'fixed' if self.origin_set else 'auto (first valid fix)'}")
        
    def declare_parameters(self):
        """Declare all ROS parameters with default values"""
        # Topics
        self.declare_parameter('gps_topic', '/mavros/global_position/raw/fix')
        self.declare_parameter('output_topic', '/odometry/gps')
        
        # Frames
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom_gps')
        
        # Coordinate system
        self.declare_parameter('coordinate_system', 'enu')  # 'enu' or 'utm'
        
        # Fixed origin (optional)
        self.declare_parameter('fixed_origin_lat', None)
        self.declare_parameter('fixed_origin_lon', None)
        self.declare_parameter('fixed_origin_alt', 0.0)
        
        # Validation settings
        self.declare_parameter('min_fix_quality', 0)  # Minimum NavSatStatus.status value
        self.declare_parameter('max_position_covariance', 100.0)  # Maximum acceptable covariance
        self.declare_parameter('validate_coordinates', True)  # Check for 0,0 coordinates
        
        # Publishing settings
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_stats', True)
        
        # Covariances
        self.declare_parameter('default_position_covariance', 1.0)
        self.declare_parameter('orientation_covariance', 999999.0)  # No orientation info from GPS
        self.declare_parameter('velocity_covariance', 999999.0)     # No velocity info in this implementation
        
    def get_parameters(self):
        """Get all parameters from ROS parameter server"""
        # Topics
        self.gps_topic = self.get_parameter('gps_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        
        # Frames
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        
        # Coordinate system
        self.coordinate_system = self.get_parameter('coordinate_system').value
        
        # Fixed origin
        self.fixed_origin_lat = self.get_parameter('fixed_origin_lat').value
        self.fixed_origin_lon = self.get_parameter('fixed_origin_lon').value
        self.fixed_origin_alt = self.get_parameter('fixed_origin_alt').value
        
        # Validation settings
        self.min_fix_quality = self.get_parameter('min_fix_quality').value
        self.max_position_covariance = self.get_parameter('max_position_covariance').value
        self.validate_coordinates = self.get_parameter('validate_coordinates').value
        
        # Publishing settings
        self.publish_tf = self.get_parameter('publish_tf').value
        self.publish_stats = self.get_parameter('publish_stats').value
        
        # Covariances
        self.default_position_covariance = self.get_parameter('default_position_covariance').value
        self.orientation_covariance = self.get_parameter('orientation_covariance').value
        self.velocity_covariance = self.get_parameter('velocity_covariance').value
        
    def gps_callback(self, msg: NavSatFix):
        """Callback for GPS NavSatFix messages"""
        self.fix_count += 1
        
        # Validate the GPS fix
        if not self.is_valid_fix(msg):
            self.invalid_count += 1
            return
            
        # Set origin from first valid fix if not already set
        if not self.origin_set:
            self.converter.set_origin(msg.latitude, msg.longitude, msg.altitude)
            self.origin_set = True
            self.get_logger().info(
                f"Origin set from first valid fix: {msg.latitude:.8f}, "
                f"{msg.longitude:.8f}, {msg.altitude:.2f}"
            )
            
        # Convert to local coordinates
        try:
            x, y, z = self.converter.convert_to_local(
                msg.latitude, msg.longitude, msg.altitude
            )
        except Exception as e:
            self.get_logger().warn(f"Coordinate conversion failed: {e}")
            self.invalid_count += 1
            return
            
        # Create and publish odometry message
        odom_msg = self.create_odometry_message(msg, x, y, z)
        self.odom_pub.publish(odom_msg)
        
        # Publish TF if enabled
        if self.publish_tf:
            self.publish_transform(odom_msg)
            
        # Store last valid fix
        self.last_valid_fix = msg
        
    def is_valid_fix(self, msg: NavSatFix) -> bool:
        """
        Validate GPS fix message
        
        Args:
            msg (NavSatFix): GPS fix message
            
        Returns:
            bool: True if fix is valid
        """
        # Check fix status
        if msg.status.status < self.min_fix_quality:
            self.get_logger().debug(
                f"Invalid fix status: {msg.status.status} < {self.min_fix_quality}"
            )
            return False
            
        # Check for invalid coordinates (0,0)
        if self.validate_coordinates:
            if abs(msg.latitude) < 1e-6 and abs(msg.longitude) < 1e-6:
                self.get_logger().debug("Invalid coordinates: (0, 0)")
                return False
                
        # Check coordinate bounds
        if not self.converter.is_valid_coordinate(msg.latitude, msg.longitude):
            self.get_logger().debug(
                f"Coordinates out of bounds: {msg.latitude}, {msg.longitude}"
            )
            return False
            
        # Check position covariance
        if len(msg.position_covariance) >= 9:
            # Check if all covariance values are the "unknown" value (10000.0)
            if all(abs(cov - 10000.0) < 1e-3 for cov in msg.position_covariance[:3]):
                self.get_logger().debug("Position covariance indicates unknown accuracy")
                return False
                
            # Check if maximum covariance is too high
            max_cov = max(msg.position_covariance[0], msg.position_covariance[4], msg.position_covariance[8])
            if max_cov > self.max_position_covariance:
                self.get_logger().debug(
                    f"Position covariance too high: {max_cov} > {self.max_position_covariance}"
                )
                return False
                
        return True
        
    def create_odometry_message(self, gps_msg: NavSatFix, x: float, y: float, z: float) -> Odometry:
        """
        Create odometry message from GPS data and local coordinates
        
        Args:
            gps_msg (NavSatFix): Original GPS message
            x (float): Local X coordinate
            y (float): Local Y coordinate  
            z (float): Local Z coordinate
            
        Returns:
            Odometry: Odometry message
        """
        odom_msg = Odometry()
        
        # Header
        odom_msg.header.stamp = gps_msg.header.stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        
        # Position
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = z
        
        # Orientation (neutral - GPS doesn't provide orientation)
        odom_msg.pose.pose.orientation.w = 1.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0
        
        # Velocity (zero - not estimated in this implementation)
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        
        # Set covariances
        self.set_covariances(odom_msg, gps_msg)
        
        return odom_msg
        
    def set_covariances(self, odom_msg: Odometry, gps_msg: NavSatFix):
        """
        Set covariance matrices for odometry message
        
        Args:
            odom_msg (Odometry): Odometry message to modify
            gps_msg (NavSatFix): Original GPS message with covariance info
        """
        # Pose covariance (6x6)
        pose_cov = [0.0] * 36
        
        # Position covariances from GPS
        if len(gps_msg.position_covariance) >= 9:
            # Map GPS covariance to local coordinate system
            # Note: This is a simplified mapping - for precise applications,
            # proper coordinate transformation of covariance matrix is needed
            pose_cov[0] = gps_msg.position_covariance[0]   # x (east/easting)
            pose_cov[7] = gps_msg.position_covariance[4]   # y (north/northing)
            pose_cov[14] = gps_msg.position_covariance[8]  # z (up/altitude)
        else:
            # Use default covariances
            pose_cov[0] = self.default_position_covariance   # x
            pose_cov[7] = self.default_position_covariance   # y
            pose_cov[14] = self.default_position_covariance  # z
            
        # Orientation covariances (GPS doesn't provide orientation)
        pose_cov[21] = self.orientation_covariance   # roll
        pose_cov[28] = self.orientation_covariance   # pitch
        pose_cov[35] = self.orientation_covariance   # yaw
        
        odom_msg.pose.covariance = pose_cov
        
        # Twist covariance (6x6) - no velocity estimation
        twist_cov = [self.velocity_covariance] * 36
        odom_msg.twist.covariance = twist_cov
        
    def publish_transform(self, odom_msg: Odometry):
        """
        Publish TF transform
        
        Args:
            odom_msg (Odometry): Odometry message to convert to TF
        """
        t = TransformStamped()
        t.header = odom_msg.header
        t.child_frame_id = odom_msg.child_frame_id
        
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        t.transform.rotation = odom_msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
        
    def publish_statistics(self):
        """Publish statistics about GPS fix quality"""
        if self.fix_count > 0:
            valid_rate = ((self.fix_count - self.invalid_count) / self.fix_count) * 100.0
            self.get_logger().info(
                f"GPS Stats: {self.fix_count} total, {self.invalid_count} invalid, "
                f"{valid_rate:.1f}% valid rate"
            )
            
            # Show origin info
            origin = self.converter.get_origin()
            if origin:
                self.get_logger().info(
                    f"Origin: {origin[0]:.8f}, {origin[1]:.8f}, {origin[2]:.2f}"
                )
                
            # Show last fix info
            if self.last_valid_fix:
                self.get_logger().info(
                    f"Last fix: {self.last_valid_fix.latitude:.8f}, "
                    f"{self.last_valid_fix.longitude:.8f}, "
                    f"{self.last_valid_fix.altitude:.2f}, "
                    f"status: {self.last_valid_fix.status.status}"
                )


def main(args=None):
    rclpy.init(args=args)
    
    node = GpsOdometry()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

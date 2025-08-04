#!/usr/bin/env python3
"""
Main Encoder Odometry Node

ROS 2 node for 4WD encoder-based odometry using Jetson GPIO and quadrature encoders.
Publishes standard odometry messages and raw encoder data for debugging.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import math
import time
import threading
from typing import Dict, Any, Optional

# ROS messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import Header
from std_srvs.srv import Empty
import tf2_ros

# Local imports
from .encoder_reader import EncoderManager
from .utils import (
    ticks_to_distance, differential_kinematics_4wd, differential_kinematics_2wd,
    update_pose, calculate_velocities, euler_to_quaternion, normalize_angle
)

# Import custom message (will be generated)
try:
    from encoder_odometry.msg import EncoderTicks
except ImportError:
    print("WARNING: EncoderTicks message not found. Please build the package first.")
    EncoderTicks = None


class EncoderOdometryNode(Node):
    """
    ROS 2 node for encoder-based odometry estimation
    
    Reads quadrature encoder signals from 4 wheels, calculates odometry,
    and publishes standard nav_msgs/Odometry messages.
    """
    
    def __init__(self):
        super().__init__('encoder_odometry_node')
        
        # Declare parameters
        self.declare_parameters()
        self.get_parameters()
        
        # Initialize state variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.time()
        
        # Encoder tick tracking
        self.last_ticks = {
            'front_left': 0,
            'front_right': 0,
            'rear_left': 0,
            'rear_right': 0
        }
        
        # Velocity tracking
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        # Thread safety
        self.odom_lock = threading.Lock()
        
        # Initialize encoder manager
        self.encoder_manager = EncoderManager()
        self.setup_encoders()
        
        # QoS profiles
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        debug_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Publishers
        self.odom_pub = self.create_publisher(
            Odometry,
            self.output_topic,
            odom_qos
        )
        
        if EncoderTicks and self.publish_raw_ticks:
            self.ticks_pub = self.create_publisher(
                EncoderTicks,
                self.ticks_topic,
                debug_qos
            )
        
        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Services
        callback_group = ReentrantCallbackGroup()
        self.reset_srv = self.create_service(
            Empty,
            '~/reset_odometry',
            self.reset_odometry_callback,
            callback_group=callback_group
        )
        
        # Timer for odometry calculation and publishing
        self.odom_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.odometry_timer_callback
        )
        
        # Timer for diagnostics
        if self.publish_diagnostics:
            self.diag_timer = self.create_timer(
                5.0,  # Every 5 seconds
                self.diagnostics_callback
            )
        
        self.get_logger().info(f"Encoder Odometry node started")
        self.get_logger().info(f"Drive mode: {self.drive_mode}")
        self.get_logger().info(f"Output topic: {self.output_topic}")
        self.get_logger().info(f"Wheel diameter: {self.wheel_diameter:.3f}m")
        self.get_logger().info(f"Track width: {self.track_width:.3f}m")
        self.get_logger().info(f"Wheelbase: {self.wheelbase:.3f}m")
        
    def declare_parameters(self):
        """Declare all ROS parameters with default values"""
        # GPIO pin assignments (Jetson BCM numbering)
        self.declare_parameter('pins.front_left.a', 17)
        self.declare_parameter('pins.front_left.b', 18)
        self.declare_parameter('pins.front_right.a', 27)
        self.declare_parameter('pins.front_right.b', 22)
        self.declare_parameter('pins.rear_left.a', 23)
        self.declare_parameter('pins.rear_left.b', 24)
        self.declare_parameter('pins.rear_right.a', 25)
        self.declare_parameter('pins.rear_right.b', 4)
        
        # Robot physical parameters
        self.declare_parameter('robot.wheel_diameter', 0.215)  # 215mm
        self.declare_parameter('robot.track_width', 0.380)     # 380mm
        self.declare_parameter('robot.wheelbase', 0.3645)      # 364.5mm
        self.declare_parameter('robot.encoder_ppr', 537.7)     # Pulses per revolution
        
        # Drive configuration
        self.declare_parameter('robot.drive_mode', '4wd')  # '4wd' or '2wd'
        
        # Direction inversion (for each wheel)
        self.declare_parameter('invert.front_left', False)
        self.declare_parameter('invert.front_right', True)   # Typically inverted
        self.declare_parameter('invert.rear_left', False)
        self.declare_parameter('invert.rear_right', True)    # Typically inverted
        
        # Topics and frames
        self.declare_parameter('topics.output_topic', '/odometry/encoders')
        self.declare_parameter('topics.ticks_topic', '/encoders/ticks')
        self.declare_parameter('frames.base_frame', 'base_link')
        self.declare_parameter('frames.odom_frame', 'odom_encoders')
        
        # Publishing settings
        self.declare_parameter('publishing.publish_rate', 50.0)
        self.declare_parameter('publishing.publish_tf', True)
        self.declare_parameter('publishing.publish_raw_ticks', True)
        self.declare_parameter('publishing.publish_diagnostics', True)
        
        # Advanced settings
        self.declare_parameter('advanced.gpio_pull_mode', 'down')  # 'up', 'down', 'none'
        self.declare_parameter('advanced.debounce_time', 0.001)
        self.declare_parameter('advanced.velocity_filter_alpha', 0.1)
        
    def get_parameters(self):
        """Get all parameters from ROS parameter server"""
        # GPIO pins
        self.gpio_pins = {
            'front_left': {
                'a': self.get_parameter('pins.front_left.a').value,
                'b': self.get_parameter('pins.front_left.b').value
            },
            'front_right': {
                'a': self.get_parameter('pins.front_right.a').value,
                'b': self.get_parameter('pins.front_right.b').value
            },
            'rear_left': {
                'a': self.get_parameter('pins.rear_left.a').value,
                'b': self.get_parameter('pins.rear_left.b').value
            },
            'rear_right': {
                'a': self.get_parameter('pins.rear_right.a').value,
                'b': self.get_parameter('pins.rear_right.b').value
            }
        }
        
        # Robot parameters
        self.wheel_diameter = self.get_parameter('robot.wheel_diameter').value
        self.track_width = self.get_parameter('robot.track_width').value
        self.wheelbase = self.get_parameter('robot.wheelbase').value
        self.encoder_ppr = self.get_parameter('robot.encoder_ppr').value
        self.drive_mode = self.get_parameter('robot.drive_mode').value
        
        # Direction inversions
        self.inversions = {
            'front_left': self.get_parameter('invert.front_left').value,
            'front_right': self.get_parameter('invert.front_right').value,
            'rear_left': self.get_parameter('invert.rear_left').value,
            'rear_right': self.get_parameter('invert.rear_right').value
        }
        
        # Topics and frames
        self.output_topic = self.get_parameter('topics.output_topic').value
        self.ticks_topic = self.get_parameter('topics.ticks_topic').value
        self.base_frame = self.get_parameter('frames.base_frame').value
        self.odom_frame = self.get_parameter('frames.odom_frame').value
        
        # Publishing settings
        self.publish_rate = self.get_parameter('publishing.publish_rate').value
        self.publish_tf = self.get_parameter('publishing.publish_tf').value
        self.publish_raw_ticks = self.get_parameter('publishing.publish_raw_ticks').value
        self.publish_diagnostics = self.get_parameter('publishing.publish_diagnostics').value
        
        # Advanced settings
        self.gpio_pull_mode = self.get_parameter('advanced.gpio_pull_mode').value
        self.debounce_time = self.get_parameter('advanced.debounce_time').value
        self.velocity_filter_alpha = self.get_parameter('advanced.velocity_filter_alpha').value
        
    def setup_encoders(self):
        """Initialize all encoders with their GPIO pins"""
        for wheel_name, pins in self.gpio_pins.items():
            try:
                encoder = self.encoder_manager.add_encoder(
                    name=wheel_name,
                    pin_a=pins['a'],
                    pin_b=pins['b'],
                    invert_direction=self.inversions[wheel_name],
                    callback=self.encoder_callback
                )
                self.get_logger().info(
                    f"Encoder {wheel_name}: GPIO {pins['a']},{pins['b']} "
                    f"(inverted: {self.inversions[wheel_name]})"
                )
            except Exception as e:
                self.get_logger().error(f"Failed to setup encoder {wheel_name}: {e}")
                
    def encoder_callback(self, encoder_name: str, ticks: int, direction: int):
        """Callback when encoder tick changes (optional, for debugging)"""
        if self.get_logger().get_effective_level() <= 10:  # DEBUG level
            self.get_logger().debug(
                f"Encoder {encoder_name}: {ticks} ticks, direction: {direction}"
            )
    
    def odometry_timer_callback(self):
        """Main odometry calculation and publishing timer"""
        current_time = time.time()
        
        with self.odom_lock:
            # Get current encoder ticks
            current_ticks = self.encoder_manager.get_all_ticks()
            
            # Calculate tick deltas
            tick_deltas = {}
            for wheel in self.last_ticks.keys():
                if wheel in current_ticks:
                    tick_deltas[wheel] = current_ticks[wheel] - self.last_ticks[wheel]
                    self.last_ticks[wheel] = current_ticks[wheel]
                else:
                    tick_deltas[wheel] = 0
            
            # Convert ticks to distances
            wheel_distances = {}
            for wheel, delta_ticks in tick_deltas.items():
                wheel_distances[wheel] = ticks_to_distance(
                    delta_ticks, self.encoder_ppr, self.wheel_diameter
                )
            
            # Calculate robot displacement
            linear_disp, angular_disp = self.calculate_displacement(wheel_distances)
            
            # Update pose
            self.x, self.y, self.theta = update_pose(
                self.x, self.y, self.theta, linear_disp, angular_disp
            )
            
            # Calculate velocities
            dt = current_time - self.last_time
            linear_vel, angular_vel = calculate_velocities(linear_disp, angular_disp, dt)
            
            # Apply velocity filtering
            self.linear_velocity = (
                self.velocity_filter_alpha * linear_vel +
                (1 - self.velocity_filter_alpha) * self.linear_velocity
            )
            self.angular_velocity = (
                self.velocity_filter_alpha * angular_vel +
                (1 - self.velocity_filter_alpha) * self.angular_velocity
            )
            
            self.last_time = current_time
        
        # Publish odometry
        self.publish_odometry(current_time)
        
        # Publish raw ticks if enabled
        if EncoderTicks and self.publish_raw_ticks:
            self.publish_encoder_ticks(current_time, current_ticks, tick_deltas, wheel_distances)
    
    def calculate_displacement(self, wheel_distances: Dict[str, float]) -> tuple:
        """
        Calculate linear and angular displacement from wheel distances
        
        Args:
            wheel_distances (Dict[str, float]): Distance for each wheel
            
        Returns:
            tuple: (linear_displacement, angular_displacement)
        """
        if self.drive_mode == '4wd':
            return differential_kinematics_4wd(
                wheel_distances['front_left'],
                wheel_distances['front_right'],
                wheel_distances['rear_left'],
                wheel_distances['rear_right'],
                self.wheelbase,
                self.track_width
            )
        else:  # 2wd mode (use rear wheels)
            return differential_kinematics_2wd(
                wheel_distances['rear_left'],
                wheel_distances['rear_right'],
                self.track_width
            )
    
    def publish_odometry(self, timestamp: float):
        """Publish odometry message"""
        odom_msg = Odometry()
        
        # Header
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        
        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation (convert yaw to quaternion)
        quat = euler_to_quaternion(0.0, 0.0, self.theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        
        # Velocities
        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.angular_velocity
        
        # Covariance (simple diagonal)
        pose_cov = [0.0] * 36
        twist_cov = [0.0] * 36
        
        # Position covariance (increase with distance traveled)
        distance_traveled = math.sqrt(self.x**2 + self.y**2)
        base_pos_cov = 0.001 + distance_traveled * 0.0001
        
        pose_cov[0] = base_pos_cov    # x
        pose_cov[7] = base_pos_cov    # y
        pose_cov[14] = 1e6            # z (not used)
        pose_cov[21] = 1e6            # roll (not used)
        pose_cov[28] = 1e6            # pitch (not used)
        pose_cov[35] = 0.01           # yaw
        
        twist_cov[0] = 0.01   # linear.x
        twist_cov[7] = 1e6    # linear.y (not used)
        twist_cov[14] = 1e6   # linear.z (not used)
        twist_cov[21] = 1e6   # angular.x (not used)
        twist_cov[28] = 1e6   # angular.y (not used)
        twist_cov[35] = 0.01  # angular.z
        
        odom_msg.pose.covariance = pose_cov
        odom_msg.twist.covariance = twist_cov
        
        # Publish
        self.odom_pub.publish(odom_msg)
        
        # Publish TF if enabled
        if self.publish_tf:
            self.publish_transform(odom_msg)
    
    def publish_transform(self, odom_msg: Odometry):
        """Publish TF transform"""
        t = TransformStamped()
        t.header = odom_msg.header
        t.child_frame_id = odom_msg.child_frame_id
        
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        t.transform.rotation = odom_msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_encoder_ticks(
        self,
        timestamp: float,
        current_ticks: Dict[str, int],
        tick_deltas: Dict[str, int],
        wheel_distances: Dict[str, float]
    ):
        """Publish raw encoder ticks for debugging"""
        if not EncoderTicks:
            return
            
        ticks_msg = EncoderTicks()
        
        # Header
        ticks_msg.header.stamp = self.get_clock().now().to_msg()
        ticks_msg.header.frame_id = self.base_frame
        
        # Fill message with data
        ticks_msg.front_left_ticks = current_ticks.get('front_left', 0)
        ticks_msg.front_right_ticks = current_ticks.get('front_right', 0)
        ticks_msg.rear_left_ticks = current_ticks.get('rear_left', 0)
        ticks_msg.rear_right_ticks = current_ticks.get('rear_right', 0)
        
        ticks_msg.front_left_delta = tick_deltas.get('front_left', 0)
        ticks_msg.front_right_delta = tick_deltas.get('front_right', 0)
        ticks_msg.rear_left_delta = tick_deltas.get('rear_left', 0)
        ticks_msg.rear_right_delta = tick_deltas.get('rear_right', 0)
        
        ticks_msg.front_left_distance = wheel_distances.get('front_left', 0.0)
        ticks_msg.front_right_distance = wheel_distances.get('front_right', 0.0)
        ticks_msg.rear_left_distance = wheel_distances.get('rear_left', 0.0)
        ticks_msg.rear_right_distance = wheel_distances.get('rear_right', 0.0)
        
        self.ticks_pub.publish(ticks_msg)
    
    def reset_odometry_callback(self, request, response):
        """Service callback to reset odometry"""
        with self.odom_lock:
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            self.encoder_manager.reset_all_ticks()
            
            # Reset last ticks
            for wheel in self.last_ticks.keys():
                self.last_ticks[wheel] = 0
        
        self.get_logger().info("Odometry reset to origin")
        return response
    
    def diagnostics_callback(self):
        """Publish diagnostic information"""
        encoder_info = self.encoder_manager.get_all_info()
        
        info_strings = []
        for name, info in encoder_info.items():
            info_strings.append(
                f"{name}: {info['ticks']} ticks, "
                f"dir: {info['direction']}, "
                f"pins: {info['pin_a']},{info['pin_b']}"
            )
        
        self.get_logger().info(
            f"Encoders - {', '.join(info_strings)}"
        )
        self.get_logger().info(
            f"Pose: x={self.x:.3f}, y={self.y:.3f}, θ={math.degrees(self.theta):.1f}°"
        )
        self.get_logger().info(
            f"Velocity: linear={self.linear_velocity:.3f}m/s, "
            f"angular={math.degrees(self.angular_velocity):.1f}°/s"
        )
    
    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        self.get_logger().info("Cleaning up encoder odometry node...")
        self.encoder_manager.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    # Use multi-threaded executor for better performance
    executor = MultiThreadedExecutor()
    
    node = EncoderOdometryNode()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

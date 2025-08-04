#!/usr/bin/env python3
"""
Complete Odometry Workspace Launch File
Launches all four odometry nodes: LIDAR, IMU/Magnetometer, GPS, and Encoders
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate complete odometry workspace launch description"""
    
    # Declare global launch arguments
    declare_use_lidar = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Enable LIDAR odometry'
    )
    
    declare_use_imu_mag = DeclareLaunchArgument(
        'use_imu_mag',
        default_value='true',
        description='Enable IMU/Magnetometer odometry'
    )
    
    declare_use_gps = DeclareLaunchArgument(
        'use_gps',
        default_value='true',
        description='Enable GPS odometry'
    )
    
    declare_use_encoders = DeclareLaunchArgument(
        'use_encoders',
        default_value='true',
        description='Enable encoder odometry'
    )
    
    declare_lidar_topic = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/livox/lidar',
        description='LIDAR point cloud topic'
    )
    
    declare_imu_topic = DeclareLaunchArgument(
        'imu_topic',
        default_value='/mavros/imu/data',
        description='IMU data topic'
    )
    
    declare_mag_topic = DeclareLaunchArgument(
        'mag_topic',
        default_value='/mavros/imu/mag',
        description='Magnetometer data topic'
    )
    
    declare_gps_topic = DeclareLaunchArgument(
        'gps_topic',
        default_value='/mavros/global_position/raw/fix',
        description='GPS NavSatFix topic'
    )
    
    # LIDAR Odometry Launch
    lidar_odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lidar_odometry_fastgicp'),
                'launch',
                'lidar_odometry.launch.py'
            ])
        ]),
        launch_arguments={
            'input_cloud_topic': LaunchConfiguration('lidar_topic'),
            'output_odom_topic': '/odometry/lidar',
            'base_frame': 'base_link',
            'odom_frame': 'odom_lidar',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_lidar'))
    )
    
    # IMU/Magnetometer Odometry Launch
    imu_mag_odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('imu_mag_odometry'),
                'launch',
                'imu_mag_odometry.launch.py'
            ])
        ]),
        launch_arguments={
            'imu_topic': LaunchConfiguration('imu_topic'),
            'mag_topic': LaunchConfiguration('mag_topic'),
            'output_topic': '/odometry/imu_mag',
            'base_frame': 'base_link',
            'odom_frame': 'odom_imu_mag',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_imu_mag'))
    )
    
    # GPS Odometry Launch
    gps_odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gps_odometry'),
                'launch',
                'gps_odometry.launch.py'
            ])
        ]),
        launch_arguments={
            'gps_topic': LaunchConfiguration('gps_topic'),
            'output_topic': '/odometry/gps',
            'base_frame': 'base_link',
            'odom_frame': 'odom_gps',
            'coordinate_system': 'enu',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_gps'))
    )
    
    # Encoder Odometry Launch
    encoder_odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('encoder_odometry'),
                'launch',
                'encoder_odometry.launch.py'
            ])
        ]),
        launch_arguments={
            'output_topic': '/odometry/encoders',
            'base_frame': 'base_link',
            'odom_frame': 'odom_encoders',
            'drive_mode': '4wd',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_encoders'))
    )
    
    return LaunchDescription([
        declare_use_lidar,
        declare_use_imu_mag,
        declare_use_gps,
        declare_use_encoders,
        declare_lidar_topic,
        declare_imu_topic,
        declare_mag_topic,
        declare_gps_topic,
        lidar_odometry_launch,
        imu_mag_odometry_launch,
        gps_odometry_launch,
        encoder_odometry_launch,
    ])

#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package name
    package_name = 'imu_mag_odometry'
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare(package_name),
            'config',
            'imu_mag_params.yaml'
        ]),
        description='Path to the configuration file'
    )
    
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/mavros/imu/data',
        description='IMU data topic'
    )
    
    mag_topic_arg = DeclareLaunchArgument(
        'mag_topic',
        default_value='/mavros/imu/mag',
        description='Magnetometer data topic'
    )
    
    pressure_topic_arg = DeclareLaunchArgument(
        'pressure_topic',
        default_value='/mavros/imu/atm_pressure',
        description='Barometer/pressure data topic'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/odometry/imu_mag',
        description='Output odometry topic'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base frame ID'
    )
    
    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame ID'
    )
    
    orientation_filter_arg = DeclareLaunchArgument(
        'orientation_filter',
        default_value='madgwick',
        description='Orientation filter: madgwick or use_imu_orientation'
    )
    
    enable_integration_arg = DeclareLaunchArgument(
        'enable_integration',
        default_value='false',
        description='Enable acceleration integration for position/velocity estimation'
    )
    
    use_barometer_arg = DeclareLaunchArgument(
        'use_barometer',
        default_value='false',
        description='Use barometer for altitude estimation'
    )
    
    madgwick_beta_arg = DeclareLaunchArgument(
        'madgwick_beta',
        default_value='0.1',
        description='Madgwick filter beta gain'
    )
    
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Publish TF transforms'
    )
    
    # IMU/Mag Odometry Node
    imu_mag_odometry_node = Node(
        package=package_name,
        executable='imu_mag_odometry_node',
        name='imu_mag_odometry_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'imu_topic': LaunchConfiguration('imu_topic'),
                'mag_topic': LaunchConfiguration('mag_topic'),
                'pressure_topic': LaunchConfiguration('pressure_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'base_frame': LaunchConfiguration('base_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'orientation_filter': LaunchConfiguration('orientation_filter'),
                'enable_integration': LaunchConfiguration('enable_integration'),
                'use_barometer': LaunchConfiguration('use_barometer'),
                'madgwick_beta': LaunchConfiguration('madgwick_beta'),
                'publish_tf': LaunchConfiguration('publish_tf'),
            }
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        imu_topic_arg,
        mag_topic_arg,
        pressure_topic_arg,
        output_topic_arg,
        base_frame_arg,
        odom_frame_arg,
        orientation_filter_arg,
        enable_integration_arg,
        use_barometer_arg,
        madgwick_beta_arg,
        publish_tf_arg,
        imu_mag_odometry_node
    ])

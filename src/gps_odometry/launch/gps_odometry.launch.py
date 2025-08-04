#!/usr/bin/env python3
"""
Launch file for GPS Odometry node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for GPS odometry"""
    
    # Declare launch arguments
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('gps_odometry'),
            'config',
            'gps_odometry.yaml'
        ]),
        description='Path to GPS odometry configuration file'
    )
    
    declare_gps_topic = DeclareLaunchArgument(
        'gps_topic',
        default_value='/mavros/global_position/raw/fix',
        description='GPS NavSatFix topic name'
    )
    
    declare_output_topic = DeclareLaunchArgument(
        'output_topic',
        default_value='/odometry/gps',
        description='Output odometry topic name'
    )
    
    declare_base_frame = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Robot base frame'
    )
    
    declare_odom_frame = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom_gps',
        description='GPS odometry frame'
    )
    
    declare_coordinate_system = DeclareLaunchArgument(
        'coordinate_system',
        default_value='enu',
        description='Local coordinate system (enu or utm)'
    )
    
    declare_publish_tf = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Whether to publish TF transforms'
    )
    
    # GPS Odometry node
    gps_odometry_node = Node(
        package='gps_odometry',
        executable='gps_odometry_node',
        name='gps_odometry',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'gps_topic': LaunchConfiguration('gps_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'base_frame': LaunchConfiguration('base_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'coordinate_system': LaunchConfiguration('coordinate_system'),
                'publish_tf': LaunchConfiguration('publish_tf'),
            }
        ],
        output='screen',
        remappings=[
            ('gps_input', LaunchConfiguration('gps_topic')),
            ('odometry_output', LaunchConfiguration('output_topic')),
        ]
    )
    
    return LaunchDescription([
        declare_config_file,
        declare_gps_topic,
        declare_output_topic,
        declare_base_frame,
        declare_odom_frame,
        declare_coordinate_system,
        declare_publish_tf,
        gps_odometry_node,
    ])

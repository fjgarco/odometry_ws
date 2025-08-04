#!/usr/bin/env python3
"""
Launch file for Encoder Odometry Node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for encoder odometry"""
    
    # Declare launch arguments
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('encoder_odometry'),
            'config',
            'encoder_params.yaml'
        ]),
        description='Path to encoder odometry configuration file'
    )
    
    declare_output_topic = DeclareLaunchArgument(
        'output_topic',
        default_value='/odometry/encoders',
        description='Output odometry topic name'
    )
    
    declare_base_frame = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Robot base frame'
    )
    
    declare_odom_frame = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom_encoders',
        description='Encoder odometry frame'
    )
    
    declare_publish_rate = DeclareLaunchArgument(
        'publish_rate',
        default_value='50.0',
        description='Odometry publishing rate (Hz)'
    )
    
    declare_drive_mode = DeclareLaunchArgument(
        'drive_mode',
        default_value='4wd',
        description='Drive mode: 4wd or 2wd'
    )
    
    declare_publish_tf = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Whether to publish TF transforms'
    )
    
    declare_publish_ticks = DeclareLaunchArgument(
        'publish_ticks',
        default_value='true',
        description='Whether to publish raw encoder ticks'
    )
    
    # Encoder Odometry node
    encoder_odometry_node = Node(
        package='encoder_odometry',
        executable='encoder_odometry_node',
        name='encoder_odometry',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'topics.output_topic': LaunchConfiguration('output_topic'),
                'frames.base_frame': LaunchConfiguration('base_frame'),
                'frames.odom_frame': LaunchConfiguration('odom_frame'),
                'publishing.publish_rate': LaunchConfiguration('publish_rate'),
                'robot.drive_mode': LaunchConfiguration('drive_mode'),
                'publishing.publish_tf': LaunchConfiguration('publish_tf'),
                'publishing.publish_raw_ticks': LaunchConfiguration('publish_ticks'),
            }
        ],
        output='screen',
        emulate_tty=True,
        remappings=[
            ('odometry_output', LaunchConfiguration('output_topic')),
        ]
    )
    
    return LaunchDescription([
        declare_config_file,
        declare_output_topic,
        declare_base_frame,
        declare_odom_frame,
        declare_publish_rate,
        declare_drive_mode,
        declare_publish_tf,
        declare_publish_ticks,
        encoder_odometry_node,
    ])

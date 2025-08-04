#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package name
    package_name = 'lidar_odometry_fastgicp'
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare(package_name),
            'config',
            'lidar_odometry_params.yaml'
        ]),
        description='Path to the configuration file'
    )
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/livox/lidar',
        description='Input point cloud topic'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/odometry/lidar',
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
    
    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size',
        default_value='0.5',
        description='Voxel grid leaf size for downsampling'
    )
    
    use_cuda_arg = DeclareLaunchArgument(
        'use_cuda',
        default_value='false',
        description='Use CUDA acceleration if available'
    )
    
    # LIDAR Odometry Node
    lidar_odometry_node = Node(
        package=package_name,
        executable='lidar_odometry_node',
        name='lidar_odometry_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'base_frame': LaunchConfiguration('base_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'voxel_size': LaunchConfiguration('voxel_size'),
                'use_cuda': LaunchConfiguration('use_cuda'),
            }
        ],
        remappings=[
            ('input_topic', LaunchConfiguration('input_topic')),
            ('output_topic', LaunchConfiguration('output_topic')),
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        input_topic_arg,
        output_topic_arg,
        base_frame_arg,
        odom_frame_arg,
        voxel_size_arg,
        use_cuda_arg,
        lidar_odometry_node
    ])

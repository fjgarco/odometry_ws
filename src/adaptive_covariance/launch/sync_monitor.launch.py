#!/usr/bin/env python3

"""
Launch file for Sync Monitor
============================

Launches the multi-sensor synchronization monitor with configurable parameters
for different operational scenarios (indoor, outdoor, high-speed, debug).

Usage:
    ros2 launch adaptive_covariance sync_monitor.launch.py
    ros2 launch adaptive_covariance sync_monitor.launch.py config:=indoor
    ros2 launch adaptive_covariance sync_monitor.launch.py config:=outdoor
    ros2 launch adaptive_covariance sync_monitor.launch.py config:=highspeed
    ros2 launch adaptive_covariance sync_monitor.launch.py config:=debug
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """Setup launch configuration based on parameters."""
    
    # Get package directory
    pkg_dir = get_package_share_directory('adaptive_covariance')
    
    # Get configuration name
    config_name = LaunchConfiguration('config').perform(context)
    
    # Map configuration names to parameter namespaces
    config_mapping = {
        'default': 'sync_monitor',
        'indoor': 'sync_monitor_indoor', 
        'outdoor': 'sync_monitor_outdoor',
        'highspeed': 'sync_monitor_highspeed',
        'debug': 'sync_monitor_debug'
    }
    
    # Get parameter namespace
    param_namespace = config_mapping.get(config_name, 'sync_monitor')
    
    # Configuration file path
    config_file = os.path.join(pkg_dir, 'config', 'sync_monitor.yaml')
    
    # Get topic remappings
    visual_topic = LaunchConfiguration('visual_topic').perform(context)
    lidar_topic = LaunchConfiguration('lidar_topic').perform(context)
    gnss_topic = LaunchConfiguration('gnss_topic').perform(context)
    imu_topic = LaunchConfiguration('imu_topic').perform(context)
    encoder_topic = LaunchConfiguration('encoder_topic').perform(context)
    
    # Create parameter overrides for topics if specified
    parameter_overrides = {}
    
    if visual_topic != 'default':
        parameter_overrides['topics.visual_odom'] = visual_topic
    if lidar_topic != 'default':
        parameter_overrides['topics.lidar_odom'] = lidar_topic
    if gnss_topic != 'default':
        parameter_overrides['topics.gnss_odom'] = gnss_topic
    if imu_topic != 'default':
        parameter_overrides['topics.imu_data'] = imu_topic
    if encoder_topic != 'default':
        parameter_overrides['topics.encoder_odom'] = encoder_topic
    
    # Create sync monitor node
    sync_monitor_node = Node(
        package='adaptive_covariance',
        executable='sync_monitor',
        name='sync_monitor',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            {config_file: param_namespace},
            parameter_overrides
        ],
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    return [sync_monitor_node]


def generate_launch_description():
    """Generate launch description with configurable parameters."""
    
    return LaunchDescription([
        
        # === Configuration Selection ===
        DeclareLaunchArgument(
            'config',
            default_value='default',
            description='Configuration preset: default, indoor, outdoor, highspeed, debug'
        ),
        
        # === Namespace Configuration ===
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the sync monitor node'
        ),
        
        # === Topic Remapping ===
        DeclareLaunchArgument(
            'visual_topic',
            default_value='default',
            description='Visual odometry topic (default: use config file)'
        ),
        
        DeclareLaunchArgument(
            'lidar_topic', 
            default_value='default',
            description='LIDAR odometry topic (default: use config file)'
        ),
        
        DeclareLaunchArgument(
            'gnss_topic',
            default_value='default', 
            description='GNSS odometry topic (default: use config file)'
        ),
        
        DeclareLaunchArgument(
            'imu_topic',
            default_value='default',
            description='IMU data topic (default: use config file)'
        ),
        
        DeclareLaunchArgument(
            'encoder_topic',
            default_value='default',
            description='Encoder odometry topic (default: use config file)'
        ),
        
        # === Logging Configuration ===
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level: debug, info, warn, error'
        ),
        
        # === Launch Setup ===
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    generate_launch_description()

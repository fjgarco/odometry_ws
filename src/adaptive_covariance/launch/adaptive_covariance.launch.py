#!/usr/bin/env python3
"""
Adaptive Covariance Launch File

Launches the complete adaptive covariance system including:
- Sensor health monitoring
- EKF covariance adaptation
- Diagnostic and logging nodes

Author: fjgarco
License: MIT
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for adaptive covariance system"""
    
    # Package directory
    pkg_adaptive_covariance = FindPackageShare('adaptive_covariance')
    
    # Configuration files
    config_file = PathJoinSubstitution([
        pkg_adaptive_covariance, 'config', 'adaptive_covariance_params.yaml'
    ])
    
    thresholds_file = PathJoinSubstitution([
        pkg_adaptive_covariance, 'config', 'sensor_health_thresholds.yaml'
    ])
    
    ekf_mapping_file = PathJoinSubstitution([
        pkg_adaptive_covariance, 'config', 'ekf_parameter_mapping.yaml'
    ])
    
    # Launch arguments
    declare_args = [
        DeclareLaunchArgument(
            'enable_adaptive_covariance',
            default_value='true',
            description='Enable adaptive covariance adjustment'
        ),
        DeclareLaunchArgument(
            'enable_ekf_adjustment',
            default_value='true',
            description='Enable real EKF parameter adjustment (vs monitoring only)'
        ),
        DeclareLaunchArgument(
            'enable_sensor_health_monitor',
            default_value='true',
            description='Enable detailed sensor health monitoring'
        ),
        DeclareLaunchArgument(
            'enable_diagnostics',
            default_value='true',
            description='Enable diagnostic publishing'
        ),
        DeclareLaunchArgument(
            'use_sync_monitor',
            default_value='true',
            description='Enable multi-sensor synchronization monitor'
        ),
        DeclareLaunchArgument(
            'sync_config',
            default_value='default',
            description='Sync monitor configuration: default, indoor, outdoor, highspeed, debug'
        ),
        DeclareLaunchArgument(
            'ekf_namespace',
            default_value='robot_localization',
            description='robot_localization namespace'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to main configuration file'
        ),
        DeclareLaunchArgument(
            'thresholds_file',
            default_value=thresholds_file,
            description='Path to sensor health thresholds file'
        ),
        DeclareLaunchArgument(
            'ekf_mapping_file',
            default_value=ekf_mapping_file,
            description='Path to EKF parameter mapping file'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level (debug, info, warn, error)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        )
    ]
    
    # Main adaptive covariance node
    adaptive_covariance_node = Node(
        package='adaptive_covariance',
        executable='adaptive_covariance_node',
        name='adaptive_covariance_node',
        condition=IfCondition(LaunchConfiguration('enable_adaptive_covariance')),
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'thresholds_file': LaunchConfiguration('thresholds_file'),
            'ekf_namespace': LaunchConfiguration('ekf_namespace'),
            'enable_ekf_adjustment': LaunchConfiguration('enable_ekf_adjustment'),
            'enable_diagnostics': LaunchConfiguration('enable_diagnostics'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )
    
    # Sensor health monitor node
    sensor_health_monitor_node = Node(
        package='adaptive_covariance',
        executable='sensor_health_monitor',
        name='sensor_health_monitor',
        condition=IfCondition(LaunchConfiguration('enable_sensor_health_monitor')),
        parameters=[{
            'thresholds_file': LaunchConfiguration('thresholds_file'),
            'enable_diagnostics': LaunchConfiguration('enable_diagnostics'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )
    
    # EKF covariance adapter node
    ekf_covariance_adapter_node = Node(
        package='adaptive_covariance',
        executable='ekf_covariance_adapter',
        name='ekf_covariance_adapter',
        condition=IfCondition(LaunchConfiguration('enable_ekf_adjustment')),
        parameters=[{
            'ekf_mapping_file': LaunchConfiguration('ekf_mapping_file'),
            'ekf_namespace': LaunchConfiguration('ekf_namespace'),
            'enable_parameter_adjustment': LaunchConfiguration('enable_ekf_adjustment'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )
    
    # Sync monitor node
    sync_monitor_node = Node(
        package='adaptive_covariance',
        executable='sync_monitor',
        name='sync_monitor',
        condition=IfCondition(LaunchConfiguration('use_sync_monitor')),
        parameters=[
            PathJoinSubstitution([
                pkg_adaptive_covariance, 'config', 'sync_monitor.yaml'
            ])
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )
    
    # Group all adaptive covariance nodes
    adaptive_covariance_group = GroupAction([
        adaptive_covariance_node,
        sensor_health_monitor_node,
        ekf_covariance_adapter_node,
        sync_monitor_node
    ])
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    for arg in declare_args:
        ld.add_action(arg)
    
    # Add nodes
    ld.add_action(adaptive_covariance_group)
    
    return ld


if __name__ == '__main__':
    generate_launch_description()

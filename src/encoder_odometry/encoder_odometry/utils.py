#!/usr/bin/env python3
"""
Utility functions for encoder odometry calculations
"""

import math
import numpy as np
from typing import Tuple, List


def normalize_angle(angle: float) -> float:
    """
    Normalize angle to [-pi, pi]
    
    Args:
        angle (float): Angle in radians
        
    Returns:
        float: Normalized angle
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def ticks_to_distance(ticks: int, ppr: float, wheel_diameter: float) -> float:
    """
    Convert encoder ticks to linear distance
    
    Args:
        ticks (int): Number of encoder ticks
        ppr (float): Pulses per revolution
        wheel_diameter (float): Wheel diameter in meters
        
    Returns:
        float: Distance in meters
    """
    revolutions = ticks / ppr
    circumference = math.pi * wheel_diameter
    return revolutions * circumference


def differential_kinematics_4wd(
    fl_dist: float, fr_dist: float, rl_dist: float, rr_dist: float,
    wheelbase: float, track_width: float
) -> Tuple[float, float]:
    """
    Calculate linear and angular displacement using 4WD kinematics
    
    Args:
        fl_dist (float): Front left wheel distance
        fr_dist (float): Front right wheel distance  
        rl_dist (float): Rear left wheel distance
        rr_dist (float): Rear right wheel distance
        wheelbase (float): Distance between front and rear axles
        track_width (float): Distance between left and right wheels
        
    Returns:
        Tuple[float, float]: (linear_displacement, angular_displacement)
    """
    # Average distances for left and right sides
    left_avg = (fl_dist + rl_dist) / 2.0
    right_avg = (fr_dist + rr_dist) / 2.0
    
    # Linear displacement (average of both sides)
    linear_displacement = (left_avg + right_avg) / 2.0
    
    # Angular displacement (difference divided by track width)
    angular_displacement = (right_avg - left_avg) / track_width
    
    return linear_displacement, angular_displacement


def differential_kinematics_2wd(
    left_dist: float, right_dist: float, track_width: float
) -> Tuple[float, float]:
    """
    Calculate linear and angular displacement using 2WD kinematics
    
    Args:
        left_dist (float): Left wheel distance
        right_dist (float): Right wheel distance
        track_width (float): Distance between wheels
        
    Returns:
        Tuple[float, float]: (linear_displacement, angular_displacement)
    """
    # Linear displacement (average of both wheels)
    linear_displacement = (left_dist + right_dist) / 2.0
    
    # Angular displacement (difference divided by track width)
    angular_displacement = (right_dist - left_dist) / track_width
    
    return linear_displacement, angular_displacement


def update_pose(
    x: float, y: float, theta: float,
    linear_disp: float, angular_disp: float
) -> Tuple[float, float, float]:
    """
    Update robot pose using odometry displacement
    
    Args:
        x (float): Current x position
        y (float): Current y position
        theta (float): Current orientation
        linear_disp (float): Linear displacement
        angular_disp (float): Angular displacement
        
    Returns:
        Tuple[float, float, float]: (new_x, new_y, new_theta)
    """
    # Update orientation first
    new_theta = normalize_angle(theta + angular_disp)
    
    # Calculate position update
    if abs(angular_disp) < 1e-6:
        # Straight line motion
        dx = linear_disp * math.cos(theta)
        dy = linear_disp * math.sin(theta)
    else:
        # Curved motion
        radius = linear_disp / angular_disp
        dx = radius * (math.sin(new_theta) - math.sin(theta))
        dy = radius * (math.cos(theta) - math.cos(new_theta))
    
    new_x = x + dx
    new_y = y + dy
    
    return new_x, new_y, new_theta


def calculate_velocities(
    linear_disp: float, angular_disp: float, dt: float
) -> Tuple[float, float]:
    """
    Calculate linear and angular velocities
    
    Args:
        linear_disp (float): Linear displacement in meters
        angular_disp (float): Angular displacement in radians
        dt (float): Time delta in seconds
        
    Returns:
        Tuple[float, float]: (linear_velocity, angular_velocity)
    """
    if dt <= 0:
        return 0.0, 0.0
        
    linear_vel = linear_disp / dt
    angular_vel = angular_disp / dt
    
    return linear_vel, angular_vel


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> List[float]:
    """
    Convert Euler angles to quaternion [x, y, z, w]
    
    Args:
        roll (float): Roll angle in radians
        pitch (float): Pitch angle in radians  
        yaw (float): Yaw angle in radians
        
    Returns:
        List[float]: Quaternion [x, y, z, w]
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    
    return [qx, qy, qz, qw]


def lowpass_filter(new_value: float, old_value: float, alpha: float) -> float:
    """
    Simple low-pass filter for noise reduction
    
    Args:
        new_value (float): New measurement
        old_value (float): Previous filtered value
        alpha (float): Filter coefficient (0-1, higher = less filtering)
        
    Returns:
        float: Filtered value
    """
    return alpha * new_value + (1 - alpha) * old_value


def detect_wheel_slip(wheel_distances: List[float], threshold: float = 0.1) -> bool:
    """
    Detect potential wheel slip by comparing wheel distances
    
    Args:
        wheel_distances (List[float]): Distances for all wheels
        threshold (float): Maximum allowed deviation
        
    Returns:
        bool: True if slip detected
    """
    if len(wheel_distances) < 2:
        return False
        
    mean_dist = np.mean(wheel_distances)
    max_deviation = max([abs(d - mean_dist) for d in wheel_distances])
    
    return max_deviation > threshold

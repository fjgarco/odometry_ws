"""
Madgwick orientation filter implementation
Based on the paper: "An efficient orientation filter for inertial and inertial/magnetic sensor arrays"
by Sebastian O.H. Madgwick
"""

import numpy as np
import math


class MadgwickFilter:
    """
    Madgwick orientation filter for IMU and MARG (Magnetic, Angular Rate, Gravity) sensors
    """
    
    def __init__(self, sample_rate=100.0, beta=0.1):
        """
        Initialize the Madgwick filter
        
        Args:
            sample_rate (float): Sample rate in Hz
            beta (float): Filter gain (0.1 is default, lower = more stable, higher = faster convergence)
        """
        self.sample_rate = sample_rate
        self.beta = beta
        self.dt = 1.0 / sample_rate
        
        # Quaternion (w, x, y, z) - initialized as identity
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        
    def update_imu(self, accel, gyro):
        """
        Update filter with IMU data only (no magnetometer)
        
        Args:
            accel: acceleration vector [ax, ay, az] in m/s²
            gyro: gyroscope vector [gx, gy, gz] in rad/s
        """
        ax, ay, az = accel
        gx, gy, gz = gyro
        
        # Normalize accelerometer measurement
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm == 0.0:
            return  # Handle NaN
        ax /= norm
        ay /= norm
        az /= norm
        
        # Extract quaternion components
        q1, q2, q3, q4 = self.q
        
        # Auxiliary variables to avoid repeated arithmetic
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _2q4 = 2.0 * q4
        _4q1 = 4.0 * q1
        _4q2 = 4.0 * q2
        _4q3 = 4.0 * q3
        _8q2 = 8.0 * q2
        _8q3 = 8.0 * q3
        q1q1 = q1 * q1
        q2q2 = q2 * q2
        q3q3 = q3 * q3
        q4q4 = q4 * q4
        
        # Gradient decent algorithm corrective step
        s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay
        s2 = _4q2 * q4q4 - _2q4 * ax + 4.0 * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az
        s3 = 4.0 * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az
        s4 = 4.0 * q2q2 * q4 - _2q2 * ax + 4.0 * q3q3 * q4 - _2q3 * ay
        
        # Normalize step magnitude
        norm = math.sqrt(s1*s1 + s2*s2 + s3*s3 + s4*s4)
        if norm != 0.0:
            s1 /= norm
            s2 /= norm
            s3 /= norm
            s4 /= norm
        
        # Apply feedback step
        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4
        
        # Integrate rate of change of quaternion
        q1 += qDot1 * self.dt
        q2 += qDot2 * self.dt
        q3 += qDot3 * self.dt
        q4 += qDot4 * self.dt
        
        # Normalize quaternion
        norm = math.sqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4)
        self.q[0] = q1 / norm
        self.q[1] = q2 / norm
        self.q[2] = q3 / norm
        self.q[3] = q4 / norm
        
    def update_marg(self, accel, gyro, mag):
        """
        Update filter with MARG data (accelerometer, gyroscope, magnetometer)
        
        Args:
            accel: acceleration vector [ax, ay, az] in m/s²
            gyro: gyroscope vector [gx, gy, gz] in rad/s
            mag: magnetometer vector [mx, my, mz] in Tesla or Gauss
        """
        ax, ay, az = accel
        gx, gy, gz = gyro
        mx, my, mz = mag
        
        # Normalize accelerometer measurement
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm == 0.0:
            return  # Handle NaN
        ax /= norm
        ay /= norm
        az /= norm
        
        # Normalize magnetometer measurement
        norm = math.sqrt(mx*mx + my*my + mz*mz)
        if norm == 0.0:
            return  # Handle NaN
        mx /= norm
        my /= norm
        mz /= norm
        
        # Extract quaternion components
        q1, q2, q3, q4 = self.q
        
        # Auxiliary variables to avoid repeated arithmetic
        _2q1mx = 2.0 * q1 * mx
        _2q1my = 2.0 * q1 * my
        _2q1mz = 2.0 * q1 * mz
        _2q2mx = 2.0 * q2 * mx
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _2q4 = 2.0 * q4
        _2q1q3 = 2.0 * q1 * q3
        _2q3q4 = 2.0 * q3 * q4
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q3 = q3 * q3
        q3q4 = q3 * q4
        q4q4 = q4 * q4
        
        # Reference direction of Earth's magnetic field
        hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4
        hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4
        _2bx = math.sqrt(hx * hx + hy * hy)
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4
        _4bx = 2.0 * _2bx
        _4bz = 2.0 * _2bz
        
        # Gradient descent algorithm corrective step
        s1 = -_2q3 * (2.0 * q2q4 - _2q1q3 - ax) + _2q2 * (2.0 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s2 = _2q4 * (2.0 * q2q4 - _2q1q3 - ax) + _2q1 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q2 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s3 = -_2q1 * (2.0 * q2q4 - _2q1q3 - ax) + _2q4 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q3 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s4 = _2q2 * (2.0 * q2q4 - _2q1q3 - ax) + _2q3 * (2.0 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        
        # Normalize step magnitude
        norm = math.sqrt(s1*s1 + s2*s2 + s3*s3 + s4*s4)
        if norm != 0.0:
            s1 /= norm
            s2 /= norm
            s3 /= norm
            s4 /= norm
        
        # Apply feedback step
        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4
        
        # Integrate rate of change of quaternion
        q1 += qDot1 * self.dt
        q2 += qDot2 * self.dt
        q3 += qDot3 * self.dt
        q4 += qDot4 * self.dt
        
        # Normalize quaternion
        norm = math.sqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4)
        self.q[0] = q1 / norm
        self.q[1] = q2 / norm
        self.q[2] = q3 / norm
        self.q[3] = q4 / norm
        
    def get_quaternion(self):
        """
        Get current orientation as quaternion (w, x, y, z)
        
        Returns:
            numpy.array: Quaternion [w, x, y, z]
        """
        return self.q.copy()
        
    def get_euler(self):
        """
        Get current orientation as Euler angles (roll, pitch, yaw) in radians
        
        Returns:
            tuple: (roll, pitch, yaw) in radians
        """
        w, x, y, z = self.q
        
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
        
    def reset(self):
        """Reset filter to identity quaternion"""
        self.q = np.array([1.0, 0.0, 0.0, 0.0])

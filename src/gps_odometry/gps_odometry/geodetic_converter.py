"""
Geodetic coordinate conversion utilities
Converts between WGS84 (lat/lon/alt) and local coordinate systems (ENU/UTM)
"""

import math
import numpy as np
from typing import Tuple, Optional


class GeodeticConverter:
    """
    Converts between WGS84 geodetic coordinates and local coordinate systems
    Supports ENU (East-North-Up) and UTM coordinate systems
    """
    
    # WGS84 ellipsoid parameters
    WGS84_A = 6378137.0          # Semi-major axis (meters)
    WGS84_E2 = 0.00669437999014  # First eccentricity squared
    
    def __init__(self, coordinate_system='enu'):
        """
        Initialize the geodetic converter
        
        Args:
            coordinate_system (str): 'enu' or 'utm'
        """
        self.coordinate_system = coordinate_system.lower()
        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = None
        self.utm_zone = None
        self.utm_hemisphere = None
        
        if self.coordinate_system not in ['enu', 'utm']:
            raise ValueError("coordinate_system must be 'enu' or 'utm'")
            
    def set_origin(self, latitude: float, longitude: float, altitude: float = 0.0):
        """
        Set the local origin for coordinate conversion
        
        Args:
            latitude (float): Origin latitude in degrees
            longitude (float): Origin longitude in degrees  
            altitude (float): Origin altitude in meters (default: 0.0)
        """
        self.origin_lat = latitude
        self.origin_lon = longitude
        self.origin_alt = altitude
        
        if self.coordinate_system == 'utm':
            self.utm_zone = self._calculate_utm_zone(longitude)
            self.utm_hemisphere = 'N' if latitude >= 0 else 'S'
            
    def convert_to_local(self, latitude: float, longitude: float, altitude: float) -> Tuple[float, float, float]:
        """
        Convert WGS84 coordinates to local coordinate system
        
        Args:
            latitude (float): Latitude in degrees
            longitude (float): Longitude in degrees
            altitude (float): Altitude in meters
            
        Returns:
            Tuple[float, float, float]: (x, y, z) in local coordinate system
        """
        if self.origin_lat is None or self.origin_lon is None:
            raise ValueError("Origin not set. Call set_origin() first.")
            
        if self.coordinate_system == 'enu':
            return self._convert_to_enu(latitude, longitude, altitude)
        elif self.coordinate_system == 'utm':
            return self._convert_to_utm(latitude, longitude, altitude)
            
    def _convert_to_enu(self, lat: float, lon: float, alt: float) -> Tuple[float, float, float]:
        """
        Convert WGS84 to ENU (East-North-Up) coordinates
        
        Args:
            lat (float): Latitude in degrees
            lon (float): Longitude in degrees
            alt (float): Altitude in meters
            
        Returns:
            Tuple[float, float, float]: (east, north, up) in meters
        """
        # Convert degrees to radians
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        origin_lat_rad = math.radians(self.origin_lat)
        origin_lon_rad = math.radians(self.origin_lon)
        
        # Convert both points to ECEF (Earth-Centered, Earth-Fixed)
        x, y, z = self._geodetic_to_ecef(lat_rad, lon_rad, alt)
        x0, y0, z0 = self._geodetic_to_ecef(origin_lat_rad, origin_lon_rad, self.origin_alt)
        
        # Calculate difference in ECEF
        dx = x - x0
        dy = y - y0
        dz = z - z0
        
        # Rotation matrix from ECEF to ENU
        sin_lat = math.sin(origin_lat_rad)
        cos_lat = math.cos(origin_lat_rad)
        sin_lon = math.sin(origin_lon_rad)
        cos_lon = math.cos(origin_lon_rad)
        
        # ENU = R * (ECEF_point - ECEF_origin)
        east = -sin_lon * dx + cos_lon * dy
        north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
        up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
        
        return east, north, up
        
    def _convert_to_utm(self, lat: float, lon: float, alt: float) -> Tuple[float, float, float]:
        """
        Convert WGS84 to UTM coordinates (simplified implementation)
        
        Args:
            lat (float): Latitude in degrees
            lon (float): Longitude in degrees
            alt (float): Altitude in meters
            
        Returns:
            Tuple[float, float, float]: (easting, northing, altitude) relative to origin
        """
        # Convert current position to UTM
        easting, northing = self._latlon_to_utm(lat, lon)
        
        # Convert origin to UTM
        origin_easting, origin_northing = self._latlon_to_utm(self.origin_lat, self.origin_lon)
        
        # Return relative position
        x = easting - origin_easting
        y = northing - origin_northing
        z = alt - self.origin_alt
        
        return x, y, z
        
    def _geodetic_to_ecef(self, lat_rad: float, lon_rad: float, alt: float) -> Tuple[float, float, float]:
        """
        Convert geodetic coordinates to ECEF
        
        Args:
            lat_rad (float): Latitude in radians
            lon_rad (float): Longitude in radians
            alt (float): Altitude in meters
            
        Returns:
            Tuple[float, float, float]: (x, y, z) in ECEF coordinates
        """
        # Radius of curvature in the prime vertical
        N = self.WGS84_A / math.sqrt(1 - self.WGS84_E2 * math.sin(lat_rad)**2)
        
        x = (N + alt) * math.cos(lat_rad) * math.cos(lon_rad)
        y = (N + alt) * math.cos(lat_rad) * math.sin(lon_rad)
        z = (N * (1 - self.WGS84_E2) + alt) * math.sin(lat_rad)
        
        return x, y, z
        
    def _latlon_to_utm(self, lat: float, lon: float) -> Tuple[float, float]:
        """
        Convert latitude/longitude to UTM coordinates (simplified implementation)
        
        Args:
            lat (float): Latitude in degrees
            lon (float): Longitude in degrees
            
        Returns:
            Tuple[float, float]: (easting, northing) in UTM coordinates
        """
        # Convert to radians
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        
        # UTM zone central meridian
        zone = self._calculate_utm_zone(lon)
        central_meridian = math.radians((zone - 1) * 6 - 180 + 3)
        
        # UTM parameters
        k0 = 0.9996  # Scale factor
        E0 = 500000  # False easting
        N0 = 0 if lat >= 0 else 10000000  # False northing
        
        # Ellipsoid parameters
        a = self.WGS84_A
        e2 = self.WGS84_E2
        e = math.sqrt(e2)
        e_prime_sq = e2 / (1 - e2)
        
        # Intermediate values
        nu = a / math.sqrt(1 - e2 * math.sin(lat_rad)**2)
        T = math.tan(lat_rad)**2
        C = e_prime_sq * math.cos(lat_rad)**2
        A = math.cos(lat_rad) * (lon_rad - central_meridian)
        
        # Meridional arc
        M = a * ((1 - e2/4 - 3*e2**2/64 - 5*e2**3/256) * lat_rad -
                 (3*e2/8 + 3*e2**2/32 + 45*e2**3/1024) * math.sin(2*lat_rad) +
                 (15*e2**2/256 + 45*e2**3/1024) * math.sin(4*lat_rad) -
                 (35*e2**3/3072) * math.sin(6*lat_rad))
        
        # Easting
        easting = E0 + k0 * nu * (A + (1-T+C)*A**3/6 + (5-18*T+T**2+72*C-58*e_prime_sq)*A**5/120)
        
        # Northing
        northing = N0 + k0 * (M + nu*math.tan(lat_rad)*(A**2/2 + (5-T+9*C+4*C**2)*A**4/24 + 
                                                        (61-58*T+T**2+600*C-330*e_prime_sq)*A**6/720))
        
        return easting, northing
        
    def _calculate_utm_zone(self, longitude: float) -> int:
        """
        Calculate UTM zone from longitude
        
        Args:
            longitude (float): Longitude in degrees
            
        Returns:
            int: UTM zone number
        """
        return int((longitude + 180) / 6) + 1
        
    def get_origin(self) -> Optional[Tuple[float, float, float]]:
        """
        Get the current origin
        
        Returns:
            Optional[Tuple[float, float, float]]: (lat, lon, alt) or None if not set
        """
        if self.origin_lat is not None and self.origin_lon is not None:
            return self.origin_lat, self.origin_lon, self.origin_alt
        return None
        
    def is_valid_coordinate(self, latitude: float, longitude: float) -> bool:
        """
        Check if coordinates are valid
        
        Args:
            latitude (float): Latitude in degrees
            longitude (float): Longitude in degrees
            
        Returns:
            bool: True if coordinates are valid
        """
        return (-90.0 <= latitude <= 90.0 and 
                -180.0 <= longitude <= 180.0 and
                latitude != 0.0 and longitude != 0.0)

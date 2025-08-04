# Synchronized sensor data message for multi-sensor odometry fusion
# Contains temporally aligned sensor readings for adaptive covariance analysis

std_msgs/Header header

# Synchronization metadata
float64 sync_quality      # 0.0-1.0, quality of temporal synchronization
float64 time_spread_ms    # Time spread between sensors in milliseconds  
int32 sensor_count        # Number of synchronized sensors

# Sensor availability flags
bool has_visual
bool has_lidar
bool has_gnss
bool has_imu
bool has_encoder

# Synchronized sensor data (only populated if has_* is true)
nav_msgs/Odometry visual_odom
nav_msgs/Odometry lidar_odom
nav_msgs/Odometry gnss_odom
sensor_msgs/Imu imu_data
nav_msgs/Odometry encoder_odom

# Delay information
string[] sensor_names     # Names of sensors in order
float64[] sensor_delays   # Individual sensor delays in seconds

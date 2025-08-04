# Sensor synchronization status message
# Provides real-time monitoring of sync monitor health and performance

std_msgs/Header header

# Overall synchronization status
string overall_status        # 'ok', 'warning', 'degraded', 'critical'
int64 sync_callback_count   # Total number of successful synchronizations
float64 last_sync_age_s     # Seconds since last successful sync (-1 if never)
float64 sync_success_rate   # Approximated success rate (0.0-1.0)

# Sensor status information
string[] sensor_names       # All configured sensor names
string[] sensor_statuses    # Per-sensor status: 'ok', 'warning', 'critical', 'missing'
float64[] sensor_delays_avg # Average delays per sensor in seconds
string[] active_sensors     # Currently active sensor names
string[] missing_sensors    # Currently missing sensor names
int32 active_sensor_count   # Number of active sensors

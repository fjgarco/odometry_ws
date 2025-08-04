# Raw encoder ticks message for debugging
# Header for timestamp and frame
std_msgs/Header header

# Tick counts for each wheel (accumulated since start)
int64 front_left_ticks
int64 front_right_ticks
int64 rear_left_ticks
int64 rear_right_ticks

# Tick deltas since last measurement
int32 front_left_delta
int32 front_right_delta
int32 rear_left_delta
int32 rear_right_delta

# Calculated distances for each wheel (meters)
float64 front_left_distance
float64 front_right_distance
float64 rear_left_distance
float64 rear_right_distance

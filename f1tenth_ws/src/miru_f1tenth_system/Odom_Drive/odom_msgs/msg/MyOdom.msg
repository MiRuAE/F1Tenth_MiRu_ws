# Message definition for custom odometry data
std_msgs/Header header

# Position
float64 x          # X position in meters
float64 y          # Y position in meters
float64 yaw        # Yaw angle in radians

# Velocity
float64 linear_velocity     # Linear velocity in m/s
float64 angular_velocity    # Angular velocity in rad/s
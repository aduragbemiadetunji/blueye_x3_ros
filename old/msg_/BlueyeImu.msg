Header header              # Standard ROS message header
#geometry_msgs/Quaternion orientation   # Orientation as a quaternion (optional)
#float64[9] orientation_covariance   # Orientation covariance matrix (optional)
#float64[9] angular_velocity_covariance  # Angular velocity covariance matrix (optional)
#float64[9] linear_acceleration_covariance  # Linear acceleration covariance matrix (optional)
geometry_msgs/Vector3 linear_acceleration  # Linear acceleration (in m/s^2)
geometry_msgs/Vector3 angular_velocity   # Angular velocity (in rad/s)
geometry_msgs/Vector3 magnetic_field     # Magnetic field (in teslas)
float64 temperature         # Temperature (in degrees Celsius)

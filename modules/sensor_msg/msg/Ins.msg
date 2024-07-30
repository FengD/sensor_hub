SensorHeader header
float64 measurement_time
uint8 type
PointLLH position
Point3D euler_angles
Point3D linear_velocity
Point3D angular_velocity
Point3D linear_acceleration
float32[] position_covariance
float32[] euler_angles_covariance
float32[] linear_velocity_covariance
float32[] angular_velocity_covariance
float32[] linear_acceleration_covariance

# Type constatnts
uint8 TYPE_INVALID = 0
uint8 TYPE_CONVERGING = 1
uint8 TYPE_GOOD = 2

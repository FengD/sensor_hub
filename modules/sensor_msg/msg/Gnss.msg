SensorHeader header
float64 measurement_time
float32 velocity_latency 0.0
PointLLH position
Point3D position_std_dev
Point3D linear_velocity
Point3D linear_velocity_std_dev
int32 num_sats
uint8 type
uint32 solution_status
uint32 position_type

# Type constants
uint8 INVALID = 0     # Invalid solution due to insufficient observations,
                     # integrity warning, etc.  
uint8 PROPAGATED = 1  # Propagated by a Kalman filter without new observations.

# It is recommended using the following types of solution.
uint8 SINGLE = 2       # Standard GNSS solution without any corrections.
uint8 PSRDIFF = 3      # Pseudorange differential solution, including WAAS/SBAS
                  # solution.
uint8 PPP = 4          # Precise Point Positioning (PPP) solution.
uint8 RTK_FLOAT = 5    # Real Time Kinematic (RTK) float solution.
uint8 RTK_INTEGER = 6  # RTK integer solution.
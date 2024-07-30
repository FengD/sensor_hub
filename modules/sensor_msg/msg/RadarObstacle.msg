int32 id
Point2D relative_position
Point2D relative_velocity
float64 rcs
uint8 moving_status
float64 width
float64 length
float64 height
float64 theta
Point2D absolute_position
Point2D absolute_velocity
int32 count
int32 moving_frames_count
uint8 status

# Status constants
uint8 NO_TARGET = 0
uint8 NEW_TARGET = 1
uint8 NEW_UPDATED_TARGET = 2
uint8 UPDATED_TARGET = 3
uint8 COASTED_TARGET = 4
uint8 MERGED_TARGET = 5
uint8 INVALID_COASTED_TARGET = 6
uint8 NEW_COASTED_TARGET = 7

# MovingStatus constants
uint8 STATIONARY = 0
uint8 NEARING = 1
uint8 AWAYING = 2
uint8 NONE = 3
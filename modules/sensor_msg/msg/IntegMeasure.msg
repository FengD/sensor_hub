SensorHeader header
uint8 measure_type
uint8 frame_type
Point3D position
Point3D velocity
float64 yaw
int32 zone_id
bool is_have_variance
bool is_gnss_double_antenna
float64[] measure_covar

# FrameType constants
uint8 ENU = 0
uint8 ECEF = 1
uint8 UTM = 2
uint8 ODOMETER = 3

# MeasureType constants
uint8 GNSS_POS_ONLY = 0
uint8 GNSS_POS_VEL = 1
uint8 GNSS_POS_XY = 2
uint8 GNSS_VEL_ONLY = 3
uint8 POINT_CLOUD_POS = 4
uint8 ODOMETER_VEL_ONLY = 5
uint8 VEHICLE_CONSTRAINT = 6
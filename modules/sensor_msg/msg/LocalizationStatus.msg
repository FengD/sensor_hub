SensorHeader header
uint8 fusion_status
uint8 gnss_status
uint8 lidar_status
float64 measurement_time
string state_message

# MeasureState constants
uint8 NO_ERROR = 0
uint8 WARNNING = 1
uint8 ERROR = 2
uint8 CRITICAL_ERROR = 3
uint8 FATAL_ERROR = 4

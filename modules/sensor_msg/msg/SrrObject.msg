uint8 confidence_level
uint8 obj_tick
uint8 obj_size_latitude
uint8 obj_id_number
uint8 height_level
uint8 obj_size_longitude
uint8 object_type
float32 obj_latitude_distance
float32 obj_longitude_distance
float32 obj_latitude_speed
float32 obj_longitude_speed

# ConfidenceLevel constants
uint8 SEVENTY_FIVE_PERCENT = 1
uint8 NINETY_PERCENT = 2
uint8 NINETY_NINE_PERCENT = 3

# HeightLevel constants
uint8 MIDDLE = 1
uint8 LOW = 2
uint8 RESERVED = 3

# StaticObjectType constants
uint8 FRONT_LOW = 1
uint8 FRONT_MEDIAN = 2
uint8 FRONT_HIGH = 3
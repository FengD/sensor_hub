int32 id
Point3D position
float64 theta
Point3D velocity
float64 length
float64 width
float64 height
Point3D[] polygon_point
float64 tracking_time
uint8 type

# Type constants
uint8 VEHICLE = 0
uint8 UNKNOWN_MOVABLE = 1
uint8 UNKNOWN_UNMOVABLE = 2
uint8 PEDESTRIAN = 3
uint8 BICYCLE = 4
uint8 UNKNOWN = 5

float64 timestamp
float64[] point_cloud
float64 confidence
uint8 confidence_type

# ConfidenceType constants
uint8 CONFIDENCE_UNKNOWN = 0
uint8 CONFIDENCE_CNN = 1
uint8 CONFIDENCE_RADAR = 2

Point3D[] drops
Point3D acceleration
Point3D anchor_point
BBox2D bbox2d
uint8 sub_type

# ConfidenceType sub_type
uint8 ST_UNKNOWN = 0
uint8 ST_UNKNOWN_MOVABLE = 1
uint8 ST_UNKNOWN_UNMOVABLE = 2
uint8 ST_CAR = 3
uint8 ST_VAN = 4
uint8 ST_TRUCK = 5
uint8 ST_BUS = 6
uint8 ST_CYCLIST = 7
uint8 ST_MOTORCYCLIST = 8
uint8 ST_TRICYCLIST = 9
uint8 ST_PEDESTRIAN = 10
uint8 ST_TRAFFICCONE = 11
uint8 ST_UNKNOWN_UNMOVABLE_TRAFFIC_CONE = 12
uint8 ST_UNKNOWN_UNMOVABLE_TRAFFIC_PILE = 13
uint8 ST_UNKNOWN_UNMOVABLE_FENCE = 14
uint8 ST_UNKNOWN_UNMOVABLE_WATER_SAFETY_BARRIER = 15
uint8 ST_UNKNOWN_UNMOVABLE_TERRIAN = 16
uint8 ST_UNKNOWN_UNMOVABLE_VEGETATION = 17
uint8 ST_UNKNOWN_UNMOVABLE_ROAD = 18

SensorMeasurement[] measurements
float64 height_above_ground
float64[] position_covariance
float64[] velocity_covariance
float64[] acceleration_covariance
LightStatus light_status
float64 yaw_rate
int32 track_status
float64 track_status_reside_length

uint8 shape_type 1

# ShapeType constants
uint8 POLYGON = 0
uint8 BBOX = 1

bool use_convexhull false

uint16 obstacle_sub_type

# ObstacleSubType constants
uint16 SUB_TYPE_UNKNOWN = 0

# VEHICLE constants
uint16 SUB_TYPE_VEHICLE_CAR = 32
uint16 SUB_TYPE_VEHICLE_XG = 33
uint16 SUB_TYPE_VEHICLE_EXPRESS = 34
uint16 SUB_TYPE_VEHICLE_OTHER = 35

# PEDESTRIAN constants
uint16 SUB_TYPE_PEDESTRIAN_POLICE = 64
uint16 SUB_TYPE_PEDESTRIAN_CONSTRUCTER = 65
uint16 SUB_TYPE_PEDESTRIAN_CHILD = 66
uint16 SUB_TYPE_PEDESTRIAN_RIDER = 67
uint16 SUB_TYPE_PEDESTRIAN_OTHER = 68
uint16 SUB_TYPE_PEDESTRIAN_CHILD_BLIND = 69

# BICYCLE constants
uint16 SUB_TYPE_BICYCLE_MOTORBICYCLE = 96
uint16 SUB_TYPE_BICYCLE_DELIVERYMAN = 97
uint16 SUB_TYPE_BICYCLE_MOTORTRICYCLE = 98

# TRUCK constants
uint16 SUB_TYPE_TRUCK_BUS = 128
uint16 SUB_TYPE_TRUCK_CONSTRUCTION_VEHICLE = 129
uint16 SUB_TYPE_TRUCK_PICKUP = 130
uint16 SUB_TYPE_TRUCK_TRAILER_HEAD = 131
uint16 SUB_TYPE_TRUCK_TRAILER_CONTAINER = 132
uint16 SUB_TYPE_TRUCK_SPRINKLER = 133
uint16 SUB_TYPE_TRUCK_OTHER = 134

# GROUND constants
uint16 SUB_TYPE_GROUND_LANE_DASHED = 160
uint16 SUB_TYPE_GROUND_LANE_SOLID = 161
uint16 SUB_TYPE_GROUND_LANE_STOP = 162
uint16 SUB_TYPE_GROUND_SIDE_WALK = 163
uint16 SUB_TYPE_GROUND_CROSS_WALK = 164
uint16 SUB_TYPE_GROUND_ROAD = 165
uint16 SUB_TYPE_GROUND_ROAD_SIGN = 166
uint16 SUB_TYPE_GROUND_OTHER = 167
uint16 SUB_TYPE_GROUND_RUGGED_ROAD = 168
uint16 SUB_TYPE_GROUND_HOLLOW = 169

# INVALID constants
uint16 SUB_TYPE_INVALID = 180

# TFAFFIC constants
# UNKNOWN_MOVABLE
uint16 SUB_TYPE_UNKNOWN_MOVABLE_ANIMAL = 192
uint16 SUB_TYPE_UNKNOWN_MOVABLE_OTHER = 193
uint16 SUB_TYPE_UNKNOWN_MOVABLE_WIPER = 194

# UNKNOWN_UNMOVABLE
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_WARNING_TRIANGLE = 223
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_SKY = 224
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_BUILDING = 225
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_VEGETATION = 226
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_STONE_BLOCK = 227

uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_PILE = 228
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_CURB = 229
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_SPEED_BUMP = 230
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_TRAFFIC_CONE = 231
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_TRAFFIC_LIGHT = 232
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_TRAFFIC_SIGN = 233
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_WATER_SAFETY_BARRIER = 234
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_TERRIAN = 235
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_TREE = 236
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_FENCE = 237
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_POLE = 238
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_FLOWER_BOX = 239
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_WARNING_SIGN = 240
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_OTHER_OBSTACLE = 241
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_CHAIN = 242
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_OTHER_CONE = 243
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_TRUNK = 244
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_POTHOLE = 245
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_VIRTUAL = 246
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_RAIN_DIRT = 247 # object that may be created by rain dirt noise
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_ABNORMAL_AREA = 248
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_WATER_PIPE = 249
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_BLIND_AREA = 250
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_LOW_OBSTACLE = 251
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_SEEN_AREA = 252
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_ROOF_OBJ = 253
uint16 SUB_TYPE_UNKNOWN_UNMOVABLE_BUMPY_ROAD = 254

uint8 vehicle_state 0

# VehicleState constants
uint8 VEHICLE_STATE_INVALID = 0
uint8 VEHICLE_STATE_PARK = 1
uint8 VEHICLE_STATE_STOP = 2

bool is_crowd false
bool is_speed_bump_passable false
float64 min_height_to_ground 0
float64 visual_theta
bool tracking_convergence true
float64 occlusion 0
Point3D filter_velocity
Point3D flow_velocity
float32 boundary_confidence 0.0
PedestrianActivity pedestrian_activity
float32 detection_status 0.0
bool is_memorized false
bool is_in_voronoi false
bool measured_flag true

uint8 stable_flag_position 0
uint8 stable_flag_size 0
uint8 stable_flag_type 0

# StableFlag constants
uint8 P1_STABLE = 0
uint8 P2_UNSTABLE = 1

int32 reference_id
Box main_box
Box bounding_box
Box stable_box
float32 motion_uncertainty
float32 ghost_probe_info
PerceptionPostTracking perception_post_tracking
string actual_type
string controlled_type

uint8[] action_type

# ObjectActionType constants
uint8 ACTION_UNKNOWN = 0
uint8 ACTION_NORMAL = 1
# PER constants
uint8 ACTION_PER_XG = 64
uint8 ACTION_PER_PUSH = 65
uint8 ACTION_PER_ONOFF = 66
uint8 ACTION_PER_LOAD = 67
uint8 ACTION_PER_SITTING = 68
uint8 ACTION_PER_BOW = 69
uint8 ACTION_PER_LOWHEAD = 70
uint8 ACTION_PER_HANDHELD = 71
uint8 ACTION_PER_CHILD = 72
uint8 ACTION_PER_RUN = 73
# CAR constants
uint8 ACTION_CAR_DOOR = 128
uint8 ACTION_CAR_ONOFF = 129
uint8 ACTION_CAR_LOAD = 130

Point3D[] pos_anchor_points
float64[] default_val
Point3D tight_position
float64 tight_width
float64 tight_length
bool is_occasionally_seen false
byte[] compress_data

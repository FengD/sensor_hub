uint8 local_lidar_consistency
uint8 gnss_consistency
uint8 local_lidar_status
uint8 local_lidar_quality
uint8 gnsspos_position_type
uint8 msf_running_status

# LocalLidarStatus constants
uint8 MSF_LOCAL_LIDAR_NORMAL = 0       # Localization result satisfy threshold
uint8 MSF_LOCAL_LIDAR_MAP_MISSING = 1  # Can't find localization map (config.xml)
uint8 MSF_LOCAL_LIDAR_EXTRINSICS_MISSING = 2  # Missing extrinsic parameters
uint8 MSF_LOCAL_LIDAR_MAP_LOADING_FAILED = 3  # Fail to load localization map
uint8 MSF_LOCAL_LIDAR_NO_OUTPUT = 4  # No output (comparing to timestamp of imu msg)
uint8 MSF_LOCAL_LIDAR_OUT_OF_MAP = 5  # Coverage of online pointcloud and map is lower than threshold
uint8 MSF_LOCAL_LIDAR_NOT_GOOD = 6  # Localization result do not meet threshold
uint8 MSF_LOCAL_LIDAR_UNDEFINED_STATUS = 7  # others

# LocalLidarQuality constants
uint8 MSF_LOCAL_LIDAR_VERY_GOOD = 0
uint8 MSF_LOCAL_LIDAR_GOOD = 1
uint8 MSF_LOCAL_LIDAR_NOT_BAD = 2
uint8 MSF_LOCAL_LIDAR_BAD = 3

# LiDAR-based localization result check (the difference between lidar and sins result)
# LocalLidarConsistency constants
uint8 MSF_LOCAL_LIDAR_CONSISTENCY_00 = 0  # The difference is less than threshold 1
uint8 MSF_LOCAL_LIDAR_CONSISTENCY_01 = 1  # The difference is bigger than threshold 1 but less than threshold 2
uint8 MSF_LOCAL_LIDAR_CONSISTENCY_02 = 2  # The difference is bigger than threshold 2
uint8 MSF_LOCAL_LIDAR_CONSISTENCY_03 = 3  # others

# GNSS-based localization result check (the difference between GNSS and sins result)
# GnssConsistency constants
uint8 MSF_GNSS_CONSISTENCY_00 = 0  # The difference is less than threshold 1
uint8 MSF_GNSS_CONSISTENCY_01 = 1  # The difference is bigger than threshold 1 but less than threshold 2
uint8 MSF_GNSS_CONSISTENCY_02 = 2  # The difference is bigger than threshold 2
uint8 MSF_GNSS_CONSISTENCY_03 = 3  # others

# GnssPositionType constants
uint8 NONE = 0         # No solution
uint8 FIXEDPOS = 1     # Position has been fixed by the FIX POSITION command or by
            # position averaging
uint8  FIXEDHEIGHT = 2  # Position has been fixed by the FIX HEIGHT, or FIX AUTO,
            # command or by position averaging
uint8 FLOATCONV = 4    # Solution from floating point carrier phase anbiguities
uint8 WIDELANE = 5     # Solution from wide-lane ambiguities
uint8 NARROWLANE = 6   # Solution from narrow-lane ambiguities
uint8 DOPPLER_VELOCITY = 8  # Velocity computed using instantaneous Doppler
uint8 SINGLE = 16           # Single point position
uint8 PSRDIFF = 17          # Pseudorange differential solution
uint8 WAAS = 18             # Solution calculated using corrections from an SBAS
uint8 PROPOGATED = 19  # Propagated by a Kalman filter without new observations
uint8 OMNISTAR = 20    # OmniSTAR VBS position
uint8 L1_FLOAT = 32    # Floating L1 albiguity solution
uint8 IONOFREE_FLOAT = 33  # Floating ionospheric free ambiguity solution
uint8  NARROW_FLOAT = 34    # Floating narrow-lane anbiguity solution
uint8 L1_INT = 48          # Integer L1 ambiguity solution
uint8 WIDE_INT = 49        # Integer wide-lane ambiguity solution
uint8 NARROW_INT = 50      # Integer narrow-lane ambiguity solution
uint8 RTK_DIRECT_INS = 51  # RTK status where RTK filter is directly initialized from the INS filter
uint8 INS_SBAS = 52        # INS calculated position corrected for the antenna
uint8 INS_PSRSP = 53  # INS pseudorange single point solution - no DGPS corrections
uint8 INS_PSRDIFF = 54         # INS pseudorange differential solution
uint8 INS_RTKFLOAT = 55        # INS RTK float point ambiguities solution
uint8 INS_RTKFIXED = 56        # INS RTK fixed ambiguities solution
uint8 INS_OMNISTAR = 57        # INS OmniSTAR VBS solution
uint8 INS_OMNISTAR_HP = 58     # INS OmniSTAR high precision solution
uint8 INS_OMNISTAR_XP = 59     # INS OmniSuint8 IMU_TAR extra precision solution
uint8 OMNISTAR_HP = 64         # OmniSTAR high precision
uint8 OMNISTAR_XP = 65         # OmniSTAR extra precision
uint8 PPP_CONVERGING = 68      # Precise Point Position(PPP) solution converging
uint8 PPP = 69                 # Precise Point Position(PPP)solution
uint8 INS_PPP_CONVERGING = 73  # INS NovAtel CORRECT Precise Point Position(PPP)
                    # solution converging
uint8 INS_PPP = 74   # INS NovAtel CORRECT Precise Point Position(PPP) solution
uint8 MSG_LOSS = 91  # Gnss position message loss


# The running status of localization module
# MsfRunningStatus constants
uint8 MSF_SOL_LIDAR_GNSS = 0
uint8 MSF_SOL_X_GNSS = 1
uint8 MSF_SOL_LIDAR_X = 2
uint8 MSF_SOL_LIDAR_XX = 3
uint8 MSF_SOL_LIDAR_XXX = 4
uint8 MSF_SOL_X_X = 5
uint8 MSF_SOL_X_XX = 6
uint8 MSF_SOL_X_XXX = 7
uint8 MSF_SSOL_LIDAR_GNSS = 8
uint8 MSF_SSOL_X_GNSS = 9
uint8 MSF_SSOL_LIDAR_X = 10
uint8 MSF_SSOL_LIDAR_XX = 11
uint8 MSF_SSOL_LIDAR_XXX = 12
uint8 MSF_SSOL_X_X = 13
uint8 MSF_SSOL_X_XX = 14
uint8 MSF_SSOL_X_XXX = 15
uint8 MSF_NOSOL_LIDAR_GNSS = 16
uint8 MSF_NOSOL_X_GNSS = 17
uint8 MSF_NOSOL_LIDAR_X = 18
uint8 MSF_NOSOL_LIDAR_XX = 19
uint8 MSF_NOSOL_LIDAR_XXX = 20
uint8 MSF_NOSOL_X_X = 21
uint8 MSF_NOSOL_X_XX = 22
uint8 MSF_NOSOL_X_XXX = 23
uint8 MSF_RUNNING_INIT = 24

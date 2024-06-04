#define PCL_NO_PRECOMPILE
#pragma once

#include <pcl/point_types.h>

struct EIGEN_ALIGN16 POINT_TYPE_RADAR {
  // PCL_ADD_POINT4D;
  float x;
  float y;
  float z;
  float doppler;
  float range;
  float snr;
  float power;
  float azimuth;
  float elevation;
  float elevation_bin;
  float azimuth_bin;
  float doppler_bin;
  float range_bin;
  float power_bin;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(POINT_TYPE_RADAR,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, doppler, doppler)
  (float, range, range)
  (float, snr, snr)
  (float, power, power)
  (float, azimuth, azimuth)
  (float, elevation, elevation)
  (float, elevation_bin, elevation_bin)
  (float, azimuth_bin, azimuth_bin)
  (float, doppler_bin, doppler_bin)
  (float, range_bin, range_bin)
  (float, power_bin, power_bin)
)

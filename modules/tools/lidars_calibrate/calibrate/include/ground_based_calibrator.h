// Copyright (C) 2020  Hirain Inc.
// License BSD
// Author: ailiang.xing
// This is a class for a single lidar calibration based on ground fitting

#ifndef MODULES_TOOLS_LIDARS_CALIBRATE_CALIBRATE_INCLUDE_GROUND_BASED_CALIBRATOR_H_
#define MODULES_TOOLS_LIDARS_CALIBRATE_CALIBRATE_INCLUDE_GROUND_BASED_CALIBRATOR_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include "tools/lidars_calibrate/calibrate/proto/lidars_calibration_config.pb.h"

namespace crdc {
namespace airi {

typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<pcl::PointXYZI> PointICloud;
typedef typename pcl::PointCloud<pcl::PointXYZI>::Ptr PointICloudPtr;

class GroundBasedCalibrator {
 private:
  float roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_;
  float ego_x_min_, ego_x_max_, ego_y_min_, ego_y_max_, ego_z_min_, ego_z_max_;
  std::vector<float> t_;
  PointICloudPtr ground_cloud_;
  PointICloudPtr input_cloud_;
  PointICloudPtr tf_input_cloud_;
  PointICloudPtr ego_cloud_;

  void init();
  void extractROICloud();
  void getGroundSeedsByRANSAC();
  void fitGroundPlane();
  void extractEgoCloud();
  void getEgoSeedsByRANSAC();
  void fitEgoYaw();
  void fitEgoYawBy2Points();
  void transformCloud();
  void postProcessing();

 public:
  GroundBasedCalibrator();
  virtual ~GroundBasedCalibrator() = default;

  void setParameters(const crdc::airi::CalibratorSetting& node);
  void setInputCloud(const PointICloudPtr& input_cloud);
  void setEgoCloud(const PointICloudPtr& ego_cloud);
  void process();
  const std::vector<float>& getT() const;
  const PointICloudPtr& getTfInputCloud() const;
};

}  // namespace airi
}  // namespace crdc

#endif  // MODULES_TOOLS_LIDARS_CALIBRATE_CALIBRATE_INCLUDE_GROUND_BASED_CALIBRATOR_H_

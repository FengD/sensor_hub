// Copyright (C) 2020  Hirain Inc.
// License BSD
// Author: ailiang.xing
// This is a class for calibrating two lidars using registration method

#ifndef MODULES_TOOLS_LIDARS_CALIBRATE_CALIBRATE_INCLUDE_REGISTRATION_BASED_CALIBRATOR_H_
#define MODULES_TOOLS_LIDARS_CALIBRATE_CALIBRATE_INCLUDE_REGISTRATION_BASED_CALIBRATOR_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/registration.h>
#include <vector>
#include "tools/lidars_calibrate/calibrate/proto/lidars_calibration_config.pb.h"

namespace crdc {
namespace airi {

typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<pcl::PointXYZI> PointICloud;
typedef typename pcl::PointCloud<pcl::PointXYZI>::Ptr PointICloudPtr;

class RegistrationBasedCalibrator {
 private:
  PointICloudPtr target_cloud_;
  PointICloudPtr source_cloud_;
  PointICloudPtr tf_source_cloud_;
  PointICloudPtr ego_rightcloud_;

  int nb_thread_;
  int max_iteration_;
  float resolution_;
  float step_size_;
  float epsion_;
  float ego_x_min_, ego_x_max_, ego_y_min_, ego_y_max_, ego_z_min_, ego_z_max_;
  float k;
  float b;
  std::vector<float> current_guess_;
  std::vector<float> t_;

  pcl::Registration<PointI, PointI>::Ptr registration_;

  void init();

 public:
  RegistrationBasedCalibrator();
  virtual ~RegistrationBasedCalibrator() = default;

  void setParameters(const crdc::airi::CalibratorSetting& node);
  void setTargetCloud(const PointICloudPtr& target_cloud);
  void setSourceCloud(const PointICloudPtr& source_cloud);
  void setCurrentGuess(const std::vector<float>& current_guess);
  void extractEgoRightCloud();
  void fitEgoRightYaw(int lidar_num);
  void process(int lidar_num);
  const std::vector<float>& getT() const;
  const PointICloudPtr& getTfSourceCloud() const;
};

}  // namespace airi
}  // namespace crdc

#endif  // MODULES_TOOLS_LIDARS_CALIBRATE_CALIBRATE_INCLUDE_REGISTRATION_BASED_CALIBRATOR_H_

// Copyright (C) 2020  Hirain Inc.
// License BSD
// Author: ailiang.xing
// This is a class for calibrating two lidars using registration method

#include <pcl/filters/passthrough.h>
#include "include/registration_based_calibrator.h"
#include "include/registrations.h"
#include "include/utils.hpp"

namespace crdc {
namespace airi {

RegistrationBasedCalibrator::RegistrationBasedCalibrator()
    : nb_thread_(2),
      max_iteration_(200),
      resolution_(1.0),
      step_size_(0.1),
      epsion_(0.01) {
  ego_x_min_ = -2.0;
  ego_x_max_ = 0.0;
  ego_y_min_ = -1.0;
  ego_y_max_ = 1.0;
  ego_z_min_ = -0.2;
  ego_z_max_ = 1.8;
  k = 100;
  b = 100;
  target_cloud_.reset(new PointICloud);
  source_cloud_.reset(new PointICloud);
  ego_rightcloud_.reset(new PointICloud);
  tf_source_cloud_.reset(new PointICloud);
}

void RegistrationBasedCalibrator::init() {
  tf_source_cloud_->clear();
  t_.resize(6, 0.0);
  current_guess_.resize(6, 0.0);
  ego_rightcloud_->clear();
}

void RegistrationBasedCalibrator::setParameters(const crdc::airi::CalibratorSetting& node) {
  nb_thread_ = node.registrationparams().nbthread();
  max_iteration_ = node.registrationparams().maxiteration();
  resolution_ = node.registrationparams().resolution();
  step_size_ = node.registrationparams().stepsize();
  epsion_ = node.registrationparams().epsion();
  registration_ = crdc::airi::Registration<PointI>().ndt_omp(nb_thread_, max_iteration_,
                                                      resolution_, step_size_, epsion_);
  ego_x_min_ = node.egoroi().egoxmin();
  ego_x_max_ = node.egoroi().egoxmax();
  ego_y_min_ = node.egoroi().egoymin();
  ego_y_max_ = node.egoroi().egoymax();
  ego_z_min_ = node.egoroi().egozmin();
  ego_z_max_ = node.egoroi().egozmax();
}

void RegistrationBasedCalibrator::setTargetCloud(const PointICloudPtr& target_cloud) {
  target_cloud_ = target_cloud;
  registration_->setInputTarget(target_cloud_);
}

void RegistrationBasedCalibrator::setSourceCloud(const PointICloudPtr& source_cloud) {
  source_cloud_ = source_cloud;
  registration_->setInputSource(source_cloud_);
}

void RegistrationBasedCalibrator::setCurrentGuess(const std::vector<float>& current_guess) {
  current_guess_ = current_guess;
}

void RegistrationBasedCalibrator::extractEgoRightCloud() {
  pcl::PassThrough<PointI> pass;
  pass.setNegative(false);
  pass.setInputCloud(source_cloud_);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(ego_x_min_, ego_x_max_);
  pass.filter(*ego_rightcloud_);
  pass.setInputCloud(ego_rightcloud_);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(ego_y_min_, ego_y_max_);
  pass.filter(*ego_rightcloud_);
  pass.setInputCloud(ego_rightcloud_);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(ego_z_min_, ego_z_max_);
  pass.filter(*ego_rightcloud_);
}

void RegistrationBasedCalibrator::fitEgoRightYaw(int lidar_num) {
  fit2dLineByLeastSquare<PointI>(*ego_rightcloud_, k, b);
  if (lidar_num == 1) {
#ifdef WITH_TDA4
    std::string vehicle_type = "hav3g";
#else
    std::string vehicle_type = std::getenv("vehicle_type");
#endif
    if (0 == vehicle_type.compare("hav3g")) {
      current_guess_[5] = -std::atan(k) + M_PI;
    } else {
      current_guess_[5] = -std::atan(k);
    }
  } else {
    current_guess_[5] = -std::atan(k) + M_PI;
  }
}

void RegistrationBasedCalibrator::process(int lidar_num) {
  init();
  extractEgoRightCloud();
  if (ego_rightcloud_->size() < 10) return;
  fitEgoRightYaw(lidar_num);
  Eigen::Translation3f translation(current_guess_[0], current_guess_[1], current_guess_[2]);
  Eigen::AngleAxisf rot_x(current_guess_[3], Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rot_y(current_guess_[4], Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z(current_guess_[5], Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f init_trans = (translation * rot_z * rot_y * rot_x).matrix();
  registration_->align(*tf_source_cloud_, init_trans);
  Eigen::Matrix4f final_trans = registration_->getFinalTransformation();
  t_[0] = final_trans(0, 3);
  t_[1] = final_trans(1, 3);
  t_[2] = final_trans(2, 3);
  Eigen::Matrix3f rot_mat = final_trans.topLeftCorner(3, 3);
  Eigen::Vector3f rpy = rotationMatrixToEulerAngles(rot_mat);
  t_[3] = rpy[0];
  t_[4] = rpy[1];
  t_[5] = rpy[2];
}

const std::vector<float>& RegistrationBasedCalibrator::getT() const {
  return t_;
}

const PointICloudPtr& RegistrationBasedCalibrator::getTfSourceCloud() const {
  return tf_source_cloud_;
}

}  // namespace airi
}  // namespace crdc

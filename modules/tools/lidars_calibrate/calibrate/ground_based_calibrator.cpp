// Copyright (C) 2020  Hirain Inc.
// License BSD
// Author: ailiang.xing
// This is a class for a single lidar calibration based on ground fitting

#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/io.h>
#include "include/ground_based_calibrator.h"
#include "include/utils.hpp"

namespace crdc {
namespace airi {

GroundBasedCalibrator::GroundBasedCalibrator()
    : roi_x_min_(4.0), roi_x_max_(12.0), roi_y_min_(-2.5), roi_y_max_(0.0) {
  ego_x_min_ = -2.0;
  ego_x_max_ = 0.0;
  ego_y_min_ = -1.0;
  ego_y_max_ = 1.0;
  ego_z_min_ = -0.2;
  ego_z_max_ = 1.8;
  ground_cloud_.reset(new PointICloud);
  input_cloud_.reset(new PointICloud);
  tf_input_cloud_.reset(new PointICloud);
  ego_cloud_.reset(new PointICloud);
  init();
}

void GroundBasedCalibrator::init() {
  t_.resize(6, 0.0);
  ground_cloud_->clear();
  tf_input_cloud_->clear();
  ego_cloud_->clear();
}

void GroundBasedCalibrator::setInputCloud(const PointICloudPtr& input_cloud) {
  input_cloud_ = input_cloud;
}

void GroundBasedCalibrator::setEgoCloud(const PointICloudPtr& ego_cloud) {
  ego_cloud_ = ego_cloud;
}

void GroundBasedCalibrator::setParameters(const crdc::airi::CalibratorSetting& node) {
  roi_x_min_ = node.groundroi().xmin();
  roi_x_max_ = node.groundroi().xmax();
  roi_y_min_ = node.groundroi().ymin();
  roi_y_max_ = node.groundroi().ymax();
  ego_x_min_ = node.egoroi().egoxmin();
  ego_x_max_ = node.egoroi().egoxmax();
  ego_y_min_ = node.egoroi().egoymin();
  ego_y_max_ = node.egoroi().egoymax();
  ego_z_min_ = node.egoroi().egozmin();
  ego_z_max_ = node.egoroi().egozmax();
}

void GroundBasedCalibrator::process() {
  init();
  extractROICloud();
  if (ground_cloud_->size() < 10)
    return;
  getGroundSeedsByRANSAC();
  if (ground_cloud_->size() < 10)
    return;
  fitGroundPlane();
  extractEgoCloud();
  if (ego_cloud_->size() < 10)
    return;
  fitEgoYaw();
  transformCloud();
}

void GroundBasedCalibrator::extractROICloud() {
  pcl::PassThrough<PointI> pass;
  pass.setNegative(false);
  pass.setInputCloud(input_cloud_);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(roi_x_min_, roi_x_max_);
  pass.filter(*ground_cloud_);
  pass.setInputCloud(ground_cloud_);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(roi_y_min_, roi_y_max_);
  pass.filter(*ground_cloud_);
}

void GroundBasedCalibrator::getGroundSeedsByRANSAC() {
  float dist_th = 0.1;
  std::vector<int> inliers;
  Eigen::VectorXf coef;
  fitPlaneByRansac<PointI>(*ground_cloud_, dist_th, inliers, coef);
  pcl::copyPointCloud(*ground_cloud_, inliers, *ground_cloud_);
}

void GroundBasedCalibrator::fitGroundPlane() {
  std::vector<float> plane_params;
  fit3DPlane<PointICloud>(*ground_cloud_, plane_params);
  // compute t_
  float norm = sqrt(std::pow(plane_params[0], 2) + std::pow(plane_params[1], 2) +
                    std::pow(plane_params[2], 2));
  t_[2] = plane_params[3];
  t_[3] = std::fabs(norm - 0.001) > 0.0 ? std::asin(plane_params[1] / norm) : 0.0;
  t_[4] = std::fabs(norm - 0.001) > 0.0 ? -std::asin(plane_params[0] / norm) : 0.0;
}

void GroundBasedCalibrator::extractEgoCloud() {
  pcl::PassThrough<PointI> pass;
  pass.setNegative(false);
  pass.setInputCloud(input_cloud_);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(ego_x_min_, ego_x_max_);
  pass.filter(*ego_cloud_);
  pass.setInputCloud(ego_cloud_);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(ego_y_min_, ego_y_max_);
  pass.filter(*ego_cloud_);
  pass.setInputCloud(ego_cloud_);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(ego_z_min_, ego_z_max_);
  pass.filter(*ego_cloud_);
}

void GroundBasedCalibrator::getEgoSeedsByRANSAC() {
  float dist_th = 0.1;
  std::vector<int> inliers;
  Eigen::VectorXf coef;
  fitLineByRansac<PointI>(*ego_cloud_, dist_th, inliers, coef);
  pcl::copyPointCloud(*ego_cloud_, inliers, *ego_cloud_);
}

void GroundBasedCalibrator::fitEgoYaw() {
  float k = 100;
  float b = 100;
  fit2dLineByLeastSquare<PointI>(*ego_cloud_, k, b);
  t_[5] = -std::atan(k);
}

void GroundBasedCalibrator::fitEgoYawBy2Points() {
  float k = 100;
  float b = 100;
  fit2dLineBy2Points<PointI>(*ego_cloud_, k, b);
  t_[5] = -std::atan(k);
}

void GroundBasedCalibrator::transformCloud() {
  cloudTransformer<PointI>(*input_cloud_, t_[0], t_[1], t_[2], t_[3], t_[4], t_[5],
                           *tf_input_cloud_);
}

void GroundBasedCalibrator::postProcessing() {
  cloudTransformer<PointI>(*ego_cloud_, t_[0], t_[1], t_[2], t_[3], t_[4], t_[5], *ego_cloud_);
  PointI min_pt, max_pt;
  pcl::getMinMax3D(*ego_cloud_, min_pt, max_pt);
  t_[0] = -max_pt.x;
}

const std::vector<float>& GroundBasedCalibrator::getT() const {
  return t_;
}

const PointICloudPtr& GroundBasedCalibrator::getTfInputCloud() const {
  return tf_input_cloud_;
}

}  // namespace airi
}  // namespace crdc

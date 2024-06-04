// Copyright (C) 2020  Hirain Inc.
// License BSD
// Author: ailiang.xing
// This is a class for multi lidar calibration

#include <fstream>
#include <iostream>
#include "include/multi_lidar_calibrator.h"
#include "common/util.h"
#include "include/utils.hpp"

namespace crdc {
namespace airi {

MultiLidarCalibrator::MultiLidarCalibrator()
    : current_frame_num_(0), lidar_num_(0), stop_frame_num_(100), all_finished_(false) {
  ground_based_calibrator_ = new GroundBasedCalibrator();
  registration_based_calibrator_ = new RegistrationBasedCalibrator();
  t_total_.resize(6, 0.0);
}

MultiLidarCalibrator::~MultiLidarCalibrator() = default;

void MultiLidarCalibrator::AddLidar(const std::string &lidarName, unsigned int pairNum,
                                    const std::vector<float> &initT,
                                    const crdc::airi::CalibratorSetting &setting) {
  lidar_num_++;
  pairNum = lidar_num_ == (1 || 2 || 3) ? 0 : pairNum;
  pair_relationship_list_.push_back(pairNum);
  finished_.push_back(false);
  started_.push_back(false);
  std::vector<float> t_tmp = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  t_.push_back(t_tmp);
  init_t_.push_back(initT);
  lidar_name_map_.insert(std::pair<std::string, unsigned int>(lidarName, lidar_num_));
  lidar_names_.push_back(lidarName);
  input_cloud_list_.resize(lidar_num_);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
  output_cloud_list_.push_back(cloud_tmp);
  calibrator_settings_.push_back(setting);
}

void MultiLidarCalibrator::Process() {
  TransformClouds();
  for (size_t i = 0; i < lidar_num_; ++i) {
    if (finished_[i])
      continue;                             // already calibrated
    if (pair_relationship_list_[i] == 0) {  // ground based calibration
      ground_based_calibrator_->setParameters(calibrator_settings_[i]);
      ground_based_calibrator_->setInputCloud(input_cloud_list_[i]);
      ground_based_calibrator_->process();
      t_current_ = ground_based_calibrator_->getT();
      pcl::copyPointCloud(*ground_based_calibrator_->getTfInputCloud(), *output_cloud_list_[i]);
      addVector(t_total_, t_current_);
      started_[i] = true;
      current_frame_num_++;
    } else {
#ifdef WITH_TDA4
      if ((get_product_name() == "HH03-3") || (get_product_name() == "HH03-5")) {
        ground_based_calibrator_->setParameters(calibrator_settings_[i]);
        ground_based_calibrator_->setInputCloud(input_cloud_list_[i]);
        ground_based_calibrator_->process();
        t_current_ = ground_based_calibrator_->getT();
        pcl::copyPointCloud(*ground_based_calibrator_->getTfInputCloud(), *output_cloud_list_[i]);
        addVector(t_total_, t_current_);
        started_[i] = true;
        current_frame_num_++;
      } else {  // registration based calibration
        registration_based_calibrator_->setParameters(calibrator_settings_[i]);
        registration_based_calibrator_->setTargetCloud(
            output_cloud_list_[pair_relationship_list_[i] - 1]);
        registration_based_calibrator_->setSourceCloud(input_cloud_list_[i]);
        if (!started_[i]) {
          registration_based_calibrator_->setCurrentGuess(init_t_[i]);
        } else {
          registration_based_calibrator_->setCurrentGuess(t_current_);
        }
        registration_based_calibrator_->process(i);
        t_current_ = registration_based_calibrator_->getT();
        pcl::copyPointCloud(*registration_based_calibrator_->getTfSourceCloud(),
                            *output_cloud_list_[i]);
        addVector(t_total_, t_current_);
        started_[i] = true;
        current_frame_num_++;
      }
#else
      // registration based calibration
      registration_based_calibrator_->setParameters(calibrator_settings_[i]);
      registration_based_calibrator_->setTargetCloud(
          output_cloud_list_[pair_relationship_list_[i] - 1]);
      registration_based_calibrator_->setSourceCloud(input_cloud_list_[i]);
      if (!started_[i]) {
        registration_based_calibrator_->setCurrentGuess(init_t_[i]);
      } else {
        registration_based_calibrator_->setCurrentGuess(t_current_);
      }
      registration_based_calibrator_->process(i);
      t_current_ = registration_based_calibrator_->getT();
      pcl::copyPointCloud(*registration_based_calibrator_->getTfSourceCloud(),
                          *output_cloud_list_[i]);
      addVector(t_total_, t_current_);
      started_[i] = true;
      current_frame_num_++;
#endif
    }

    if (current_frame_num_ > stop_frame_num_) {
      divideVector(t_[i], t_total_, current_frame_num_);
      initT(t_total_);
      current_frame_num_ = 0;
      initT(t_current_);
      finished_[i] = true;
      all_finished_ = finished_.back();
    }
    break;
  }
}

void MultiLidarCalibrator::setInputCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                         const std::string &lidarName) {
  auto item = lidar_name_map_.find(lidarName);
  if (item != lidar_name_map_.end()) {
    unsigned int current_lidar_num = item->second;
    input_cloud_list_[current_lidar_num - 1] = cloud;
  }
}

void MultiLidarCalibrator::TransformClouds() {
  for (size_t i = 0; i < lidar_num_; ++i) {
    if (finished_[i]) {
      cloudTransformer<pcl::PointXYZI>(*input_cloud_list_[i], t_[i][0], t_[i][1], t_[i][2],
                                       t_[i][3], t_[i][4], t_[i][5], *output_cloud_list_[i]);
    } else {
      pcl::copyPointCloud(*input_cloud_list_[i], *output_cloud_list_[i]);
    }
  }
}

void MultiLidarCalibrator::setParameters(const crdc::airi::LidarsCalibrationConfig &node) {
  int lidar_num = node.lidarnum();
  stop_frame_num_ = node.iternum();
  assert(lidar_num == node.lidarlist_size());
  std::vector<float> init_t_temp(6, 0.0);
  for (size_t i = 0; i < lidar_num; ++i) {
    init_t_temp[0] = node.lidarlist(i).initpose().x();
    init_t_temp[1] = node.lidarlist(i).initpose().y();
    init_t_temp[2] = node.lidarlist(i).initpose().z();
    init_t_temp[3] = node.lidarlist(i).initpose().roll();
    init_t_temp[4] = node.lidarlist(i).initpose().pitch();
    init_t_temp[5] = node.lidarlist(i).initpose().yaw();
    AddLidar(node.lidarlist(i).name(), node.lidarlist(i).pairnum(), init_t_temp,
             node.lidarlist(i).calibratorsetting());
  }
}

const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &MultiLidarCalibrator::getOutputCloudList()
    const {
  return output_cloud_list_;
}

const std::vector<bool> &MultiLidarCalibrator::getFinished() const {
  return finished_;
}

const std::vector<std::vector<float>> &MultiLidarCalibrator::getT() const {
  return t_;
}

const std::vector<float> &MultiLidarCalibrator::getTCurrent() const {
  return t_current_;
}

bool MultiLidarCalibrator::isAllFinished() const {
  return all_finished_;
}

void MultiLidarCalibrator::setStopFrameNum(unsigned int stopFrameNum) {
  stop_frame_num_ = stopFrameNum;
}

}  // namespace airi
}  // namespace crdc

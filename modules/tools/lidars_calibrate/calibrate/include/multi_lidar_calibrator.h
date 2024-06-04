// Copyright (C) 2020  Hirain Inc.
// License BSD
// Author: ailiang.xing
// This is a class for multi lidar calibration

#ifndef MODULES_TOOLS_LIDARS_CALIBRATE_CALIBRATE_INCLUDE_MULTI_LIDAR_CALIBRATOR_H_
#define MODULES_TOOLS_LIDARS_CALIBRATE_CALIBRATE_INCLUDE_MULTI_LIDAR_CALIBRATOR_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <map>
#include <string>
#include <vector>
#include "tools/lidars_calibrate/calibrate/include/ground_based_calibrator.h"
#include "tools/lidars_calibrate/calibrate/include/registration_based_calibrator.h"
#include "tools/lidars_calibrate/calibrate/proto/lidars_calibration_config.pb.h"

namespace crdc {
namespace airi {

class MultiLidarCalibrator {
 private:
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> input_cloud_list_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> output_cloud_list_;
  /**
   * @brief Format: {0,0-1,0-2,...,0-(k-1)} for k clouds
   * Same size as input_cloud_list_
   * Every element correspond to every cloud in input_cloud
   * 0 means calibrated to ground
   * i(i>1) means calibrated to ith cloud (e.g. 1 means calibrated to the first cloud)
   */
  std::vector<unsigned int> pair_relationship_list_;
  unsigned int lidar_num_;
  std::map<std::string, unsigned int> lidar_name_map_;  // lidar num starts from 1
  std::vector<std::string> lidar_names_;
  std::vector<bool> finished_;
  std::vector<bool> started_;
  std::vector<std::vector<float>> t_;
  std::vector<std::vector<float>> init_t_;
  std::vector<crdc::airi::CalibratorSetting> calibrator_settings_;

  std::vector<float> t_current_;
  std::vector<float> t_total_;
  unsigned int current_frame_num_;

  unsigned int stop_frame_num_;  // stop condition
  bool all_finished_;

  GroundBasedCalibrator *ground_based_calibrator_;
  RegistrationBasedCalibrator *registration_based_calibrator_;

  /**
   * @brief transform lidar clouds refer to t_
   */
  void TransformClouds();
  void AddLidar(const std::string &lidarName, unsigned int pairNum, const std::vector<float> &initT,
                const crdc::airi::CalibratorSetting &setting);

 public:
  MultiLidarCalibrator();
  ~MultiLidarCalibrator();

  /**
   * @brief function for adding lidars
   * @param lidarName e.g. "left" "upper" ...
   * @param pairNum 0 means: calibrated based on ground, i (i>0) means calibrated to ith lidar.
   *                lidar list counts from 1
   * @param initT  init pose to calibration obj
   */

  void Process();

  void setInputCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                     const std::string &lidarName);

  void setParameters(const crdc::airi::LidarsCalibrationConfig &node);

  const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &getOutputCloudList() const;

  const std::vector<bool> &getFinished() const;

  const std::vector<std::vector<float>> &getT() const;

  const std::vector<float> &getTCurrent() const;

  bool isAllFinished() const;

  void setStopFrameNum(unsigned int stopFrameNum);
};
}  // namespace airi
}  // namespace crdc
#endif  // MODULES_TOOLS_LIDARS_CALIBRATE_CALIBRATE_INCLUDE_MULTI_LIDAR_CALIBRATOR_H_

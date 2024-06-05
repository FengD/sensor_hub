// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: compensator

#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#ifdef WITH_ROS2
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msg/msg/point_clouds2.hpp"
#else
#include "cyber/sensor_proto/lidar.pb.h"
#endif
#include "lidar_drivers/parser/parser.h"
#include "lidar_drivers/proto/lidar_component_config.pb.h"

namespace crdc {
namespace airi {

struct CompensationInfo {
  bool translation_only;
  uint64_t start_utime_;
  uint64_t end_utime_;
  // Eigen::Affine3d start_transform_;
  // Eigen::Affine3d end_transform_;
  std::vector<std::vector<double>> transforms_;
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct TfValue {
  float x_;
  float y_;
  float z_;
  float roll_;
  float pitch_;
  float yaw_;
};

class Compensator {
 public:
  /**
   * @brief Constructor
   *        Initail all the param in the constructor
   */
  Compensator() = default;
  virtual ~Compensator() = default;
  /**
   * @brief Init the lidar component config
   */
  bool init(const LidarComponent& config);
  bool init(const LidarComponentConfig& config);
  /**
   * @brief this function is used to transform the could and do the motion_compensation
   *
   */
  bool motion_compensation(const std::vector<std::shared_ptr<LidarPointCloud>>& clouds,
                           std::shared_ptr<PointClouds2>& clouds_compensated,
                           std::shared_ptr<CompensationInfo>& comp_info);

  std::shared_ptr<PointCloud2> cloud_transform(const std::shared_ptr<LidarPointCloud>& cloud);

  template<typename T>
  void transforms(Eigen::Matrix4f& transform, T points, int cloud_num);

 private:
  bool calculate_timestamp(const std::vector<std::shared_ptr<LidarPointCloud>>& clouds,
                          std::shared_ptr<CompensationInfo>& compensation_info);

  Eigen::Matrix4f calibration_transform_matrix(const std::string& config_file_path);

  LidarComponent config_;
  std::vector<std::string> calibration_path_;
  std::string cali_path_;
  std::unordered_map<std::string, Eigen::Matrix4f> lidar_transforms_;
  bool xytransfrom_;
};

}  // namespace airi
}  // namespace crdc

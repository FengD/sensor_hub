// Copyright (C) 2022 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: lidar fusion

#pragma once

#include <gflags/gflags.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>
#include "common/common.h"
#include "lidar_drivers/compensator/compensator.h"
#include "lidar_drivers/lidar.h"
#ifdef WITH_TDA4
#include "tools/lidars_calibrate/calibrate.h"
#endif
#if (defined WITH_TDA4) || (defined CALIBRATE)
#include "tools/lidars_calibrate/lidars_calibration.h"
#endif
#ifdef WITH_ROS2
#include "lidar_drivers/output/ros_output.h"
#include "sensor_msg/msg/point_clouds2.hpp"
#else
#include "cyber/sensor_proto/lidar.pb.h"
#include "lidar_drivers/output/cyber_output.h"
#endif

#ifdef WITH_ROS2
using PointClouds2 = sensor_msg::msg::PointClouds2;
#endif

namespace crdc {
namespace airi {

struct CloudArray {
  bool stricted_strategy_;
  uint64_t max_timestamp_gap_;
  std::set<std::string> sensors_;
  std::unordered_map<std::string, std::shared_ptr<LidarPointCloud>> clouds_;
};

class LidarFusion : public common::Thread {
 public:
  explicit LidarFusion(const LidarComponent& config);
  virtual ~LidarFusion();
  void stop();

 private:
  void run() override;
  void process(const std::shared_ptr<LidarPointCloud> &cloud);
  void assemble(const uint64_t &timestamp);
  void remove_clouds_point(std::shared_ptr<PointClouds2> &clouds);
  void reset_point(LidarPoint *pt);
  void check_blind_zone_points(const int &i, const float &x_blind_zone, char *data_buf,
                               std::shared_ptr<PointClouds2> &clouds);
  void parse_point_to_pcl_cloud(std::shared_ptr<PointCloud2> &pcd2_msg,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);
  pcl::PointXYZI point_;

#if (defined WITH_TDA4) || (defined CALIBRATE)
  void vehicle_module_judge();
  void calibrate(const std::vector<std::shared_ptr<LidarPointCloud>> &clouds);
#endif

#ifdef WITH_TDA4
  std::shared_ptr<Calibrate> lidars_calibration_;
#else
#ifdef CALIBRATE
  std::shared_ptr<LidarsCalibration> lidars_calibration_;
#endif
#endif

  bool stop_;
  LidarComponent config_;
  std::shared_ptr<Compensator> compensator_;
  std::vector<std::shared_ptr<Lidar>> lidars_;
  std::shared_ptr<PointClouds2> clouds_compensated_;
  std::mutex lock_;
  std::vector<CloudArray> cloud_array_;
  common::ThreadSafeQueue<std::vector<std::shared_ptr<LidarPointCloud>>> clouds_queue_;
};

}  // namespace airi
}  // namespace crdc


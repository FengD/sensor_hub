// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: lidar

#pragma once

#include <utility>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include "common/common.h"
#include "lidar_drivers/input/input.h"
#include "lidar_drivers/parser/parser.h"
#include "lidar_drivers/proto/lidar_config.pb.h"
#include "lidar_drivers/proto/lidar_component_config.pb.h"
#include "cyber/sensor_proto/lidar.pb.h"

namespace crdc {
namespace airi {

class Lidar : public Thread {
 public:
  Lidar() = default;
  virtual ~Lidar();
  void stop();
  void init(const LidarComponentConfig& config);

  void set_callback(std::function<void(const std::shared_ptr<PointCloud>&)> callback) {
    callback_ = callback;
  }

 private:
  void run() override;
  
  LidarComponentConfig config_;
  LidarConfig lidar_config_;
  std::function<void(const std::shared_ptr<PointCloud>&)> callback_;
  std::shared_ptr<LidarInput> input_;
  std::shared_ptr<LidarParser> parser_;
  uint32_t sensor_position_id_;
};

}  // namespace airi
}  // namespace crdc

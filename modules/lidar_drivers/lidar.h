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

class Lidar : public common::Thread {
 public:
  explicit Lidar(const LidarComponentConfig& config);
  virtual ~Lidar() = default;
  void stop();
  void set_callback(std::function<void(const std::shared_ptr<PointCloud>&)> callback) {
    callback_ = callback;
  }

  std::string get_name() const {
    return lidar_name_;
  }

 private:
  /**
   * @brief The run function execute the whole process of the lidar loop process.
   */
  void run() override;
  /**
   * @brief Init the encoder if defined.
   */
  bool init_parser();

  /**
   * @brief Init the input if defined.
   */
  bool init_input();

  bool stop_;
  int32_t reload_;
  std::string lidar_name_;
  LidarComponentConfig config_;
  LidarConfig lidar_config_;
  std::function<void(const std::shared_ptr<PointCloud>&)> callback_;
  std::shared_ptr<LidarInput> input_;
  std::shared_ptr<LidarParser> parser_;
  uint32_t sensor_position_id_;
};

}  // namespace airi
}  // namespace crdc

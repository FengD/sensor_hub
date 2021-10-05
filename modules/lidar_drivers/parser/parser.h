// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: lidar parser

#pragma once

#include <memory>
#include <string>

#include "common/common.h"
#include "lidar_drivers/proto/lidar_config.pb.h"
#include "cyber/sensor_proto/lidar.pb.h"
#include "cyber/sensor_proto/eth_packet.pb.h"

namespace crdc {
namespace airi {

class LidarParser {
 public:
  LidarParser() = default;
  virtual ~LidarParser() = default;
  /**
   * @brief Init the parser. It needs to be redefined for each subclass.
   * @param The lidar input config.
   * @return status
   */
  virtual bool init(const ParserConfig& config) {
    return false;
  }

  virtual bool parse_lidar_packet(const Packet* raw_packet, std::shared_ptr<PointCloud>* cloud) {
    return false;
  }
};

REGISTER_COMPONENT(LidarParser);
#define REGISTER_LIDAR_PARSER(name) REGISTER_CLASS(LidarParser, name)

}  // namespace airi
}  // namespace crdc
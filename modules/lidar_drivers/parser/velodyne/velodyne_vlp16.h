// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: lidar parser vlp16

#pragma once

#include <memory>
#include "common/common.h"
#include "lidar_drivers/parser/parser.h"

namespace crdc {
namespace airi {

class LidarParserVlp16 : public LidarParser {
 public:
  LidarParserVlp16() = default;
  virtual ~LidarParserVlp16() = default;
  
  bool init(const ParserConfig& config) override;

  bool parse_lidar_packet(const Packet* raw_packet, std::shared_ptr<PointCloud>* cloud) override;
};

}  // namespace airi
}  // namespace crdc
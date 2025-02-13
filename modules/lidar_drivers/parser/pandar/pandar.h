// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author:  Feng DING
// Description: lidar parser Hesai Pandar

#pragma once

#include <memory>
#include "common/common.h"
#include "lidar_drivers/parser/parser.h"

namespace sensor {
namespace hub {

class LidarParserPandar: public LidarParser {
 public:
  LidarParserPandar() = default;
  virtual ~LidarParserPandar() = default;

 protected:
  virtual void calibrate_point(LidarParserInfo& parser_info);
  virtual bool is_lidar_packet_valid(const Packet* packet);
  virtual void init_lidar_parser_params();
  virtual void init_lidar_parser_calib_info();
};

}  // namespace hub
}  // namespace sensor

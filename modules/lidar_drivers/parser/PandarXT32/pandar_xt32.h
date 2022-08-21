// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author:  Feng DING / Jian Fei JIANG
// Description: lidar parser Pxt32

#pragma once

#include <memory>
#include "common/common.h"
#include "lidar_drivers/parser/parser.h"

namespace crdc {
namespace airi {

class LidarParserPxt32 : public LidarParser {
 public:
  LidarParserPxt32() = default;
  virtual ~LidarParserPxt32() = default;

  bool init_lidar_parser() override;

  void get_block_info(const uint8_t* data, LidarParserInfo& parser_info) override;

  void get_point_raw_info(const uint8_t* data, LidarParserInfo& parser_info) override;

  uint64_t get_packet_timestamp(const Packet* packet) override;

  void calibrate_point(LidarParserInfo& parser_info) override;

  bool is_lidar_packet_valid(const Packet* packet) override;

 private:
  const float LIDAR_LINE_ANGLE32[32] = {
      15.0f, 14.0f,  13.0f,  12.0f,  11.0f,  10.0f,  9.0f,   8.0f,
      7.0f,  6.0f,   5.0f,   4.0f,   3.0f,   2.0f,   1.0f,   0.0f,
      -1.0f, -2.0f,  -3.0f,  -4.0f,  -5.0f,  -6.0f,  -7.0f,  -8.0f,
      -9.0f, -10.0f, -11.0f, -12.0f, -13.0f, -14.0f, -15.0f, -16.0f};
};

}  // namespace airi
}  // namespace crdc

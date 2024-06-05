// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author:  Feng DING / JianFei JIANG
// Description: lidar parser PandarP40

#pragma once

#include <string>
#include <memory>
#include "common/common.h"
#include "lidar_drivers/parser/pandar/pandar.h"

namespace crdc {
namespace airi {

class LidarParserP40 : public LidarParserPandar {
 public:
  LidarParserP40() = default;
  virtual ~LidarParserP40() = default;

  bool init_lidar_parser() override;

  void get_block_info(const uint8_t* data, LidarParserInfo& parser_info) override;

  void get_point_raw_info(const uint8_t* data, LidarParserInfo& parser_info) override;

  uint64_t get_packet_timestamp(const Packet* packet) override;

  std::string get_name() const override {
    return "LidarParserP40";
  }

 private:
  const float LIDAR_LINE_ANGLE40[40] = {
    7.00, 6.00, 5.00, 4.00, 3.00, 2.00, 1.67, 1.33,
    1.00, 0.67, 0.33, 0.00, -0.33, -0.67, -1.00, -1.33,
    -1.67, -2.00, -2.33, -2.67, -3.00, -3.33, -3.67, -4.00,
    -4.33, -4.67, -5.00, -5.33, -5.67, -6.00, -7.00, -8.00,
    -9.00, -10.0, -11.0, -12.0, -13.0, -14.0, -15.0, -16.0
    };

  const float HORIZON_CORRECT_ANGLE40[40] = {
    0.0, 0.0, 0.0, 0.0, -2.5, -2.5, 2.5, -5.0,
    -2.5, 2.5, -5.0, 2.5, 2.5, -5.0, 0.0, 2.5,
    -5.0, 0.0, 5.0, -2.5, 0.0, 5.0, -2.5, 0.0,
    5.0, -2.5, 2.5, 5.0, -2.5, 2.5, 2.5, 2.5,
    0.0, 0.0, 0.0, 0.0, -2.5, -2.5, -2.5, -2.5
    };
};

}  // namespace airi
}  // namespace crdc

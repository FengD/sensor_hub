// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING / Jianfei JIANG
// Description: lidar parser vlp16

#pragma once

#include <string>
#include <memory>
#include "common/common.h"
#include "lidar_drivers/parser/parser.h"

namespace crdc {
namespace airi {

class LidarParserVlp16 : public LidarParser {
 public:
  LidarParserVlp16() = default;
  virtual ~LidarParserVlp16() = default;

  bool init_lidar_parser() override;

  void get_block_info(const uint8_t* data, LidarParserInfo& parser_info) override;

  void get_point_raw_info(const uint8_t* data, LidarParserInfo& parser_info) override;

  uint64_t get_packet_timestamp(const Packet* packet) override;

  void calibrate_point(LidarParserInfo& parser_info) override;

  bool is_lidar_packet_valid(const Packet* packet) override;

  void calculate_points_data_xyz(const LidarParserInfo& parser_info, LidarPoint& pt) override;

  std::string get_name() const override {
    return "LidarParserVlp16";
  }

 private:
  const float LIDAR_LINE_ANGLE[16] = {
      -15.00, 1.00, -13.00, 3.00,  -11.00, 5.00,  -9.00, 7.00,
      -7.00,  9.00, -5.00,  11.00, -3.00,  13.00, -1.00, 15.00};
  const float RING[16] = {0.00, 8.00,  1.00, 9.00,  2.00, 10.00,
                                     3.00, 11.00, 4.00, 12.00, 5.00, 13.00,
                                     6.00, 14.00, 7.00, 15.00};
};

}  // namespace airi
}  // namespace crdc

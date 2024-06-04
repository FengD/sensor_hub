// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING / Jianfei JIANG
// Description: lidar parser lanhai_lds50cs

#pragma once

#include <string>
#include <memory>
#include "common/common.h"
#include "lidar_drivers/parser/parser.h"

namespace crdc {
namespace airi {

class LidarParserLds50cs : public LidarParser {
 public:
  LidarParserLds50cs();
  virtual ~LidarParserLds50cs() = default;

  bool init_lidar_parser() override;

  void get_block_info(const uint8_t* data, LidarParserInfo& parser_info) override;

  void get_point_raw_info(const uint8_t* data, LidarParserInfo& parser_info) override;

  uint64_t get_packet_timestamp(const Packet* packet) override;

  void calibrate_point(LidarParserInfo& parser_info) override;

  bool is_lidar_packet_valid(const Packet* packet) override;

  bool parse_lidar_packet(const Packet* packet,
                          std::shared_ptr<LidarPointCloud>* cloud) override;

  std::string get_name() const override {
    return "LidarParserLds50cs";
  }

 private:
  void pixel_xyz(const uint8_t* data);
  static const int frame_pkts_size_ = 10;
  int lidar_point_sum_;
  bool frame_end_;

  struct RawData {
    uint16_t code_;
    uint16_t N_;
    uint16_t angle_;
    uint16_t data_[1000];
  };
  #ifdef WITH_TEST
  FRIEND_TEST(LidarParserTest, lanhailds50cs_pixel_xyz_test);
  #endif
};

}  // namespace airi
}  // namespace crdc

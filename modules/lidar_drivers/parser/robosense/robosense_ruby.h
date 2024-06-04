// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author:  Feng DING / JianFei JIANG / Yuan Sun
// Description: lidar parser Robosense RS-Ruby-Lite / RS-Ruby-80V

#pragma once
#ifdef WITH_TEST
#include <gtest/gtest.h>
#endif
#include <string>
#include <memory>
#include "common/common.h"
#include "lidar_drivers/parser/parser.h"

namespace crdc {
namespace airi {

class LidarParserRSRuby : public LidarParser {
 public:
  LidarParserRSRuby() = default;
  virtual ~LidarParserRSRuby() = default;

  bool init_lidar_parser() override;

  void get_block_info(const uint8_t* data, LidarParserInfo& parser_info) override;

  void get_point_raw_info(const uint8_t* data, LidarParserInfo& parser_info) override;

  uint64_t get_packet_timestamp(const Packet* packet) override;

  void calibrate_point(LidarParserInfo& parser_info) override;

  bool is_lidar_packet_valid(const Packet* packet) override;

  void calculate_points_data_xyz(const LidarParserInfo& parser_info, LidarPoint& pt) override;

  std::string get_name() const override {
    return "LidarParserRSRuby";
  }

 private:
  // vertical angle
  const float LIDAR_LINE_ANGLE80_V2R_Lite[80] = {
  -13.565, -1.09, -4.39, -6.65, -0.29, -3.59, -5.79, -2.79,
  -4.99, -1.99, -4.19, -19.582, -1.29, -3.39, -7.15, -0.49,
  -2.59, -5.99, -1.79, -5.19, -0.99, -4.29, -25, -0.19,
  -3.49, -7.65, -2.69, -6.09, -1.89, -5.29, -16.042, -1.19,
  -4.49, -6.85, -0.39, -3.69, -5.89, -2.89, -5.09, -2.09,
  -8.352, -0.69, -3.99, -6.19, 0.11, -3.19, -5.39, -2.39,
  -4.59, -1.59, -3.79, -10.346, -0.89, -2.99, -6.39, -0.09,
  -2.19, -5.59, -1.39, -4.79, -0.59, -3.89, -11.742, 0.21,
  -3.09, -6.5, -2.29, -5.69, -1.49, -4.89, -9.244, -0.79,
  -4.09, -6.29, 0.01, -3.29, -5.49, -2.49, -4.69, -1.69
  };

  // horizontal angle correction
  const float HORIZON_CORRECT_ANGLE80_V2R_Lite[80] = {
  5.95, 4.25, 2.55, 5.95, 4.25, 2.55, 5.95, 2.55,
  5.95, 2.55, 5.95, 2.55, 0.85, 5.95, 2.55, 0.85,
  5.95, 2.55, 5.95, 2.55, 5.95, 4.25, 0.85, 5.95,
  4.25, 0.85, 4.25, 0.85, 4.25, 0.85, 4.25, 2.55,
  0.85, 4.25, 2.55, 0.85, 4.25, 0.85, 4.25, 0.85,
  -0.85, -2.55, -4.25, -0.85, -2.55, -4.25, -0.85, -4.25,
  -0.85, -4.25, -0.85, -4.25, -5.95, -0.85, -4.25, -5.95,
  -0.85, -4.25, -0.85, -4.25, -0.85, -2.55, -5.95, -0.85,
  -2.55, -5.95, -2.55, -5.95, -2.55, -5.95, -2.55, -4.25,
  -5.95, -2.55, -4.25, -5.95, -2.55, -5.95, -2.55, -5.95
  };

  // vertical angle
  const float LIDAR_LINE_ANGLE80_V2R_80V[80] = {
  -11.78, -10.37, -9.27, -8.38, -16.07, -25.1, -19.64, -13.61,
  -6.52, -6.4, -6.31, -6.21, -7.67, -7.17, -6.87, -6.67,
  -5.71, -5.6, -5.51, -5.41, -6.1, -6.01, -5.91, -5.81,
  -4.9, -4.8, -4.7, -4.6, -5.3, -5.2, -5.1, -5,
  -4.1, -4, -3.9, -3.8, -4.5, -4.4, -4.3, -4.2,
  -3.3, -3.2, -3.1, -3, 3.7, -3.6, -3.5, -3.4,
  -2.5, -2.39, -2.3, -2.2, -2.9, -2.8, -2.7, -2.6,
  -1.69, -1.59, -1.49, -1.39, -2.09, -2, -1.9, 1.8,
  -0.89, -0.79, -0.69, -0.59, -1.29, -1.19, -1.09, -0.99,
  -0.09, 0.01, 0.11, 0.21, -0.49, -0.39, -0.29, -0.19
  };

  // horizontal angle correction
  const float HORIZON_CORRECT_ANGLE80_V2R_80V[80] = {
  5.94, 2.39, -1.15, -4.69, 4.70, 1.17, -2.38, -5.92,
  5.94, 2.39, -1.15, -4.69, 4.72, 1.18, -2.36, -5.91,
  5.94, 2.39, -1.15, -4.69, 4.72, 1.18, -2.36, -5.91,
  5.94, 2.39, -1.15, -4.69, 4.72, 1.18, -2.36, -5.91,
  5.94, 2.39, -1.15, -4.69, 4.72, 1.18, -2.36, -5.91,
  5.94, 2.39, -1.15, -4.69, 4.72, 1.18, -2.36, -5.9,
  5.94, 2.39, -1.15, -4.69, 4.72, 1.18, -2.36, -5.9,
  5.94, 2.39, -1.15, -4.69, 4.72, 1.18, -2.36, -5.9,
  5.94, 2.39, -1.15, -4.69, 4.72, 1.18, -2.36, -5.9,
  5.94, 2.39, -1.15, -4.69, 4.72, 1.18, -2.36, -5.9
  };
  #ifdef WITH_TEST
  FRIEND_TEST(LidarParserTest, robosense_init_parser_test);
  #endif
};

}  // namespace airi
}  // namespace crdc

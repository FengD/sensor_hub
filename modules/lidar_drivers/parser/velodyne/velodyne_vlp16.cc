// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: lidar parser vlp16

#include "lidar_drivers/parser/velodyne/velodyne_vlp16.h"

namespace crdc {
namespace airi {

bool LidarParserVlp16::init_lidar_parser() {
  // todo
  distance_resolution_ = 0.002f;
  is_init_distance_resolution_ = true;
  is_init_calib_azimuth_ = true;
  is_init_calib_elevation_ = true;

  // some todo
  return true;
}

void LidarParserVlp16::get_block_info(const uint8_t* data, LidarParserInfo& parser_info) {
  // needs correct
  parser_info.start_of_block_ = (data[0] << 8) | data[1];
  parser_info.pixel_azimuth_ = (data[3] << 8) | data[2];
}

void LidarParserVlp16::get_point_raw_info(const uint8_t* data, LidarParserInfo& parser_info) {
  // needs correct
  parser_info.pixel_distance_ = (data[1] << 8) | data[0];
  parser_info.pixel_intensity_ = data[2];
}

uint64_t LidarParserVlp16::get_packet_timestamp(const Packet* packet) {
  // todo
  return 1;
}

void LidarParserVlp16::calibrate_point(LidarParserInfo& parser_info) {
  parser_info.distance_ = parser_info.pixel_distance_ * distance_resolution_;
  // parser_info.timestamp_ = parser_info.packet_timestamp_; todo
  parser_info.intensity_ = parser_info.pixel_intensity_;
  // parser_info.azimuth_ = parser_info.pixel_azimuth_ * ROTATION_RESOLUTION +
  //                        parser_info.pixel_azimuth_diff_ *; todo
}

}  // namespace airi
}  // namespace crdc

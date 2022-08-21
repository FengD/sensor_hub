// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING / Jian Fei JIANG
// Description: lidar parser Pandarxt32

#include "lidar_drivers/parser/PandarXT32/pandar_xt32.h"

namespace crdc {
namespace airi {

bool LidarParserPxt32::init_lidar_parser() {
  distance_resolution_ = 0.004f;
  is_init_distance_resolution_ = true;
  is_init_calib_azimuth_ = true;
  is_init_calib_elevation_ = true;

  auto packet_config = config_.mutable_lidar_packet_config();
  if (packet_config->ring_map_size() != packet_config->lasers()) {
     packet_config->clear_ring_map();
     for (auto i = 0; i < packet_config->lasers(); ++i) {
       packet_config->add_ring_map(i);
     }
  }

  if (packet_config->calib_elevation_size() != packet_config->lasers()) {
     packet_config->clear_calib_elevation();
     for (auto i = 0; i < packet_config->lasers(); ++i) {
       packet_config->add_calib_elevation(LIDAR_LINE_ANGLE32[i] * RAD_PER_DEGREE);
     }
  }

  if (packet_config->calib_azimuth_size() != packet_config->lasers()) {
     packet_config->clear_calib_azimuth();
     for (auto i = 0; i < packet_config->lasers(); ++i) {
       packet_config->add_calib_azimuth(0.0);
     }
  }

  for (auto i = 0; i < packet_config->lasers(); ++i) {
    LidarCalibInfo lidar_info;
    lidar_info.elevation_cos_ = std::cos(packet_config->calib_elevation(i));
    lidar_info.elevation_sin_ = std::sin(packet_config->calib_elevation(i));
    calib_info_.emplace_back(lidar_info);
  }

  return true;
}

void LidarParserPxt32::get_block_info(const uint8_t* data, LidarParserInfo& parser_info) {
  parser_info.start_of_block_ = 0;
  parser_info.pixel_azimuth_ = (data[1] << 8) | data[0];
}

void LidarParserPxt32::get_point_raw_info(const uint8_t* data, LidarParserInfo& parser_info) {
  parser_info.pixel_distance_ = (data[3] << 8) | data[2];
  parser_info.pixel_intensity_ = data[4];
}

uint64_t LidarParserPxt32::get_packet_timestamp(const Packet* packet) {
  const uint8_t* data = (const uint8_t*) packet->data().data();
  uint64_t timestamp_us = ((data[1074] << 24) | (data[1073] << 16)
                           | (data[1072] << 8) | data[1071]);
  struct tm time_info;
  time_info.tm_year = data[1065];
  time_info.tm_mon = data[1066] - 1;
  time_info.tm_mday = data[1067];
  time_info.tm_hour = data[1068] + 8;
  time_info.tm_min = data[1069];
  time_info.tm_sec = data[1070];
  uint64_t timestamp = mktime(&time_info)*1000000000 + timestamp_us * 1000;
  return timestamp;
}

void LidarParserPxt32::calibrate_point(LidarParserInfo& parser_info) {
  parser_info.distance_ = parser_info.pixel_distance_ * distance_resolution_;
  parser_info.timestamp_ = parser_info.packet_timestamp_;
  parser_info.intensity_ = parser_info.pixel_intensity_;
  parser_info.azimuth_ = parser_info.pixel_azimuth_ * ROTATION_RESOLUTION;
}

bool LidarParserPxt32::is_lidar_packet_valid(const Packet* packet) {
  if (packet->size() != config_.lidar_packet_config().size()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] packet size " << packet->size()
               << " wrong " << config_.lidar_packet_config().size();
    return false;
  }

  const uint8_t* data = (const uint8_t*) packet->data().data();
  uint32_t header_checksum = (data[1] << 8) | data[0];
  if (header_checksum != config_.lidar_packet_config().check_sum()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] checksum " << header_checksum
               << " wrong " << config_.lidar_packet_config().check_sum();
    return false;
  }
  return true;
}
}  // namespace airi
}  // namespace crdc

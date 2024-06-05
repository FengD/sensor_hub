// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING / JianFei JIANG
// Description: lidar parser Pandarxt32

#include "lidar_drivers/parser/pandar/pandar_xt32.h"

namespace crdc {
namespace airi {

bool LidarParserPxt32::init_lidar_parser() {
  init_lidar_parser_params();
  auto packet_config = config_.mutable_lidar_packet_config();
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

  init_lidar_parser_calib_info();
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

uint64_t LidarParserPxt32::conversion_time(const int& year, const int& mon,
                                     const int& day, const int& hour,
                                     const int& min, const int& second) {
  int year_new = year, month = mon;
  if ((month -= 2) <= 0) {
    month += 12;
    year_new -= 1;
  }
  int year_to_day = (year_new - 1) * 365 + year_new / 4 - year_new / 100 + year_new / 400;
  int mon_to_day = (367 * month / 12) - 30 + 59;
  int day_to_day = day - 1;
  uint64_t timestamp = (((uint64_t)(((year_to_day + mon_to_day + day_to_day - 719162) * 24 +
            hour) * 60 + min) * 60) + second);
  return timestamp;
}

uint64_t LidarParserPxt32::get_packet_timestamp(const Packet* packet) {
#ifdef WITH_ROS2
  const uint8_t* data = (const uint8_t*) packet->data.data();
#else
  const uint8_t* data = (const uint8_t*) packet->data().data();
#endif
  uint64_t timestamp_us = ((data[1074] << 24) | (data[1073] << 16)
                           | (data[1072] << 8) | data[1071]);
  int year = data[1065] + 1900;
  int mon = data[1066];
  int day = data[1067];
  int hour = data[1068];
  int min = data[1069];
  int sec = data[1070];
  uint64_t timestamp = conversion_time(year, mon, day, hour, min, sec) * 1000000000 +
      timestamp_us * 1000 + config_.time_zone() * 3600 + config_.tai_time_offset();
  return timestamp / 1000;
}

}  // namespace airi
}  // namespace crdc

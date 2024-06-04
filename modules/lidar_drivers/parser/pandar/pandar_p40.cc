// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING / JianFei JIANG
// Description: lidar parser PandarP40

#include "lidar_drivers/parser/pandar/pandar_p40.h"

namespace crdc {
namespace airi {

bool LidarParserP40::init_lidar_parser() {
  init_lidar_parser_params();
  auto packet_config = config_.mutable_lidar_packet_config();
  if (packet_config->calib_elevation_size() != packet_config->lasers()) {
     packet_config->clear_calib_elevation();
     for (auto i = 0; i < packet_config->lasers(); ++i) {
       packet_config->add_calib_elevation(LIDAR_LINE_ANGLE40[i] * RAD_PER_DEGREE);
     }
  }

  if (packet_config->calib_azimuth_size() != packet_config->lasers()) {
     packet_config->clear_calib_azimuth();
     for (auto i = 0; i < packet_config->lasers(); ++i) {
       packet_config->add_calib_azimuth(HORIZON_CORRECT_ANGLE40[i]);
     }
  }

  init_lidar_parser_calib_info();
  return true;
}

void LidarParserP40::get_block_info(const uint8_t* data, LidarParserInfo& parser_info) {
  parser_info.start_of_block_ = (data[1] << 8) | data[0];
  parser_info.pixel_azimuth_ = (data[3] << 8) | data[2];
}

void LidarParserP40::get_point_raw_info(const uint8_t* data, LidarParserInfo& parser_info) {
  parser_info.pixel_distance_ = (data[5] << 8) | data[4];
  parser_info.pixel_intensity_ = data[6];
}

uint64_t LidarParserP40::get_packet_timestamp(const Packet* packet) {
#ifdef WITH_ROS2
  const uint8_t* data = (const uint8_t*) packet->data.data();
#else
  const uint8_t* data = (const uint8_t*) packet->data().data();
#endif
  uint64_t timestamp = ((data[1250]) + (data[1251] << 8) +
                   (data[1252] << 16) + (data[1253] << 24));
  return timestamp;
}

}  // namespace airi
}  // namespace crdc

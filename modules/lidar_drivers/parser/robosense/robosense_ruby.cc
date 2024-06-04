// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING / JianFei JIANG / Yuan Sun
// Description: lidar parser Robosense RS-Ruby-Lite / RS-Ruby-80V

#include "lidar_drivers/parser/robosense/robosense_ruby.h"

namespace crdc {
namespace airi {

bool LidarParserRSRuby::init_lidar_parser() {
  distance_resolution_ = 0.005f;
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
        if (packet_config->block_check_sum() == 0xD1FE)
          packet_config->add_calib_elevation(LIDAR_LINE_ANGLE80_V2R_80V[i] * RAD_PER_DEGREE);
        else
          packet_config->add_calib_elevation(LIDAR_LINE_ANGLE80_V2R_Lite[i] * RAD_PER_DEGREE);
     }
  }

  if (packet_config->calib_azimuth_size() != packet_config->lasers()) {
     packet_config->clear_calib_azimuth();
     for (auto i = 0; i < packet_config->lasers(); ++i) {
        if (packet_config->block_check_sum() == 0xD1FE)
          packet_config->add_calib_azimuth(HORIZON_CORRECT_ANGLE80_V2R_80V[i]);
        else
          packet_config->add_calib_azimuth(HORIZON_CORRECT_ANGLE80_V2R_Lite[i]);
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

void LidarParserRSRuby::get_block_info(const uint8_t* data, LidarParserInfo& parser_info) {
  parser_info.start_of_block_ = (data[1] << 8) + data[0];
  parser_info.pixel_azimuth_ = (data[3] + (data[2] << 8));
}

void LidarParserRSRuby::get_point_raw_info(const uint8_t* data, LidarParserInfo& parser_info) {
  parser_info.pixel_distance_ = (data[4] << 8) + data[5];
  parser_info.pixel_intensity_ = data[6];
}

uint64_t LidarParserRSRuby::get_packet_timestamp(const Packet* packet) {
#ifdef WITH_ROS2
  const uint8_t* data = (const uint8_t*) packet->data.data();
#else
  const uint8_t* data = (const uint8_t*) packet->data().data();
#endif
  uint64_t timestamp = ((data[15]) + (data[14] << 8) + (data[13] << 16) + (data[12] << 24));
  uint64_t timestamp_ns = ((data[19]) + (data[18] << 8) + (data[17] << 16) + (data[16] << 24));
  if (timestamp > 946656000) {  // 2000-01-01 0:0:0
    return timestamp * 1000000 + timestamp_ns;
  } else {
    return 0;
  }
}

void LidarParserRSRuby::calibrate_point(LidarParserInfo& parser_info) {
  parser_info.distance_ = parser_info.pixel_distance_ * distance_resolution_;
  parser_info.timestamp_ = parser_info.packet_timestamp_;
  parser_info.intensity_ = parser_info.pixel_intensity_;
  parser_info.azimuth_ = parser_info.pixel_azimuth_ * ROTATION_RESOLUTION;
}

bool LidarParserRSRuby::is_lidar_packet_valid(const Packet* packet) {
#ifdef WITH_ROS2
  if (packet->size != config_.lidar_packet_config().size()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] packet size " << packet->size
               << " wrong " << config_.lidar_packet_config().size();
    return false;
  }

  const uint8_t* data = (const uint8_t*) packet->data.data();
#else
  if (packet->size() != config_.lidar_packet_config().size()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] packet size " << packet->size()
               << " wrong " << config_.lidar_packet_config().size();
    return false;
  }

  const uint8_t* data = (const uint8_t*) packet->data().data();
#endif
  uint32_t header_checksum = (data[1] << 8) | data[0];
  if (header_checksum != config_.lidar_packet_config().check_sum()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] checksum " << header_checksum
               << " wrong " << config_.lidar_packet_config().check_sum();
    return false;
  }
  return true;
}

void LidarParserRSRuby::calculate_points_data_xyz(
                    const LidarParserInfo& parser_info, LidarPoint& pt) {
  auto& packet_config = config_.lidar_packet_config();
  int azimuth_rad = (parser_info.azimuth_ + packet_config.calib_azimuth(parser_info.laser_))
                      * AZIMUTH_SCALE + AZIMUTH_OFFSET;
  double xy_distance = parser_info.distance_ * calib_info_[parser_info.laser_].elevation_cos_;
  if (config_.mutable_lidar_packet_config()->block_check_sum() == 0xD1FE) {
    pt.x_ = xy_distance * SINS[azimuth_rad];
    pt.y_ = xy_distance * COSS[azimuth_rad];
  } else {
    pt.x_ = -xy_distance * COSS[azimuth_rad];
    pt.y_ = xy_distance * SINS[azimuth_rad];
  }
  pt.z_ = parser_info.distance_ * calib_info_[parser_info.laser_].elevation_sin_;
}

}  // namespace airi
}  // namespace crdc

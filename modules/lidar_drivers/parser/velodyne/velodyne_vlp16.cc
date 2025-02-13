// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING / Jianfei JIANG
// Description: lidar parser vlp16

#include "lidar_drivers/parser/velodyne/velodyne_vlp16.h"

namespace sensor {
namespace hub {

bool LidarParserVlp16::init_lidar_parser() {
  distance_resolution_ = 0.002f;
  is_init_distance_resolution_ = true;
  is_init_calib_azimuth_ = true;
  is_init_calib_elevation_ = true;

  auto packet_config = config_.mutable_lidar_packet_config();
  if (packet_config->ring_map_size() != packet_config->lasers()) {
     packet_config->clear_ring_map();
     for (auto i = 0; i < packet_config->lasers(); ++i) {
       packet_config->add_ring_map(RING[i]);
     }
  }

  if (packet_config->calib_elevation_size() != packet_config->lasers()) {
     packet_config->clear_calib_elevation();
     for (auto i = 0; i < packet_config->lasers(); ++i) {
       packet_config->add_calib_elevation(LIDAR_LINE_ANGLE[i] * RAD_PER_DEGREE);
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

void LidarParserVlp16::get_block_info(const uint8_t* data, LidarParserInfo& parser_info) {
  parser_info.start_of_block_ = (data[1] << 8) + data[0];
  parser_info.pixel_azimuth_ = (data[3] << 8) + data[2];
}

void LidarParserVlp16::get_point_raw_info(const uint8_t* data, LidarParserInfo& parser_info) {
  parser_info.pixel_distance_ = (data[1] << 8) + data[0];
  parser_info.pixel_intensity_ = data[2];
}

uint64_t LidarParserVlp16::get_packet_timestamp(const Packet* packet) {
#ifdef WITH_ROS2
  const uint8_t* data = (const uint8_t*) packet->data.data();
#else
  const uint8_t* data = (const uint8_t*) packet->data().data();
#endif
  uint64_t timestamp = ((data[1200]) + (data[1201] << 8) + (data[1202] << 16) + (data[1203] << 24));
  return timestamp;
}

void LidarParserVlp16::calibrate_point(LidarParserInfo& parser_info) {
  parser_info.distance_ = parser_info.pixel_distance_ * distance_resolution_;
  parser_info.timestamp_ = parser_info.packet_timestamp_;
  parser_info.intensity_ = parser_info.pixel_intensity_;
  if (1 == parser_info.firing_) {
    parser_info.azimuth_ = parser_info.pixel_azimuth_ * ROTATION_RESOLUTION + 0.2f;
  } else {
    parser_info.azimuth_ = parser_info.pixel_azimuth_ * ROTATION_RESOLUTION;
  }
}

void LidarParserVlp16::calculate_points_data_xyz(
    const LidarParserInfo& parser_info, LidarPoint& pt) {
  auto& packet_config = config_.lidar_packet_config();
  int azimuth_rad = (parser_info.azimuth_ + packet_config.calib_azimuth(parser_info.laser_))
                      * AZIMUTH_SCALE + AZIMUTH_OFFSET;
  double xy_distance = parser_info.distance_ * calib_info_[parser_info.laser_].elevation_cos_;
  pt.x_ = xy_distance * COSS[azimuth_rad];
  pt.y_ = -xy_distance * SINS[azimuth_rad];
  pt.z_ = parser_info.distance_ * calib_info_[parser_info.laser_].elevation_sin_;
}

bool LidarParserVlp16::is_lidar_packet_valid(const Packet* packet) {
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

}  // namespace hub
}  // namespace sensor

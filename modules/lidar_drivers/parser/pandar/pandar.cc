// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author:  Feng DING
// Description: lidar parser Hesai Pandar

#include "lidar_drivers/parser/pandar/pandar.h"

namespace crdc {
namespace airi {

void LidarParserPandar::calibrate_point(LidarParserInfo& parser_info) {
    parser_info.distance_ = parser_info.pixel_distance_ * distance_resolution_;
    parser_info.timestamp_ = parser_info.packet_timestamp_;
    parser_info.intensity_ = parser_info.pixel_intensity_;
    parser_info.azimuth_ = parser_info.pixel_azimuth_ * ROTATION_RESOLUTION;
}

bool LidarParserPandar::is_lidar_packet_valid(const Packet* packet) {
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

void LidarParserPandar::init_lidar_parser_params() {
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
}

void LidarParserPandar::init_lidar_parser_calib_info() {
    auto packet_config = config_.mutable_lidar_packet_config();
    for (auto i = 0; i < packet_config->lasers(); ++i) {
        LidarCalibInfo lidar_info;
        lidar_info.elevation_cos_ = std::cos(packet_config->calib_elevation(i));
        lidar_info.elevation_sin_ = std::sin(packet_config->calib_elevation(i));
        calib_info_.emplace_back(lidar_info);
    }
}

}  // namespace airi
}  // namespace crdc

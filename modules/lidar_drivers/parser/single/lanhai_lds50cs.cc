// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING / Jianfei JIANG
// Description: lidar parser lanhai_lds50cs

#include "lidar_drivers/parser/single/lanhai_lds50cs.h"

namespace crdc {
namespace airi {

LidarParserLds50cs::LidarParserLds50cs() {
  lidar_point_sum_ = 0;
  frame_end_ = false;
}

bool LidarParserLds50cs::init_lidar_parser() {
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
      packet_config->add_calib_elevation(i);
    }
  }

  if (packet_config->calib_azimuth_size() != packet_config->lasers()) {
    packet_config->clear_calib_azimuth();
    for (auto i = 0; i < packet_config->lasers(); ++i) {
      packet_config->add_calib_azimuth(0.0);
    }
  }
  return true;
}

void LidarParserLds50cs::get_block_info(const uint8_t* data, LidarParserInfo& parser_info) {
  (void)data;
  (void)parser_info;
}

void LidarParserLds50cs::get_point_raw_info(const uint8_t* data, LidarParserInfo& parser_info) {
  (void)data;
  (void)parser_info;
}

uint64_t LidarParserLds50cs::get_packet_timestamp(const Packet* packet) {
  (void)packet;
  uint64_t now = get_now_microsecond();
  return now;
}

void LidarParserLds50cs::calibrate_point(LidarParserInfo& parser_info) {
  (void)parser_info;
}

bool LidarParserLds50cs::is_lidar_packet_valid(const Packet* packet) {
#ifdef WITH_ROS2
  const uint8_t* data = (const uint8_t*)packet->data.data();
#else
  const uint8_t* data = (const uint8_t*)packet->data().data();
#endif
  uint32_t header_checksum = (data[1] << 8) | data[0];
  if (header_checksum != config_.lidar_packet_config().check_sum()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] checksum " << header_checksum
               << " wrong " << config_.lidar_packet_config().check_sum();
    return false;
  }
  return true;
}

void LidarParserLds50cs::pixel_xyz(const uint8_t* data) {
  if (data == NULL) {
    return;
  }
  RawData* data_pkt = reinterpret_cast<RawData*>(const_cast<uint8_t*>(data));
  for (int i = 0; i < data_pkt->N_; ++i) {
    lidar_point_sum_++;
    uint16_t start_angle = data_pkt->angle_ / 10;
    float curent_angle = start_angle + 36.0 / data_pkt->N_ * i;
    float distance = (data_pkt->data_[i] & 0x1FFF) / 100.0;

    LidarPoint& pt = lidar_point_[lidar_point_sum_];
    int azimuth_rad = curent_angle * AZIMUTH_SCALE + AZIMUTH_OFFSET;
    pt.x_ = distance * COSS[azimuth_rad];
    pt.y_ = distance * SINS[azimuth_rad];
    pt.z_ = 0;
  }

  if (data_pkt->angle_ == 3240) {
    frame_end_ = true;
  } else {
    frame_end_ = false;
  }
}

bool LidarParserLds50cs::parse_lidar_packet(
    const Packet* packet, std::shared_ptr<LidarPointCloud>* cloud) {
  bool ret = false;

  if (!is_lidar_packet_valid(packet)) {
    LOG(ERROR) << "[" << config_.frame_id() << "] packet is invalid.";
    return ret;
  }

  // parse_lidar_other_info(packet);
  auto &packet_config = config_.lidar_packet_config();
  LidarParserInfo parser_info;
#ifdef WITH_ROS2
  const uint8_t *data = reinterpret_cast<const uint8_t *>(packet->data.data());
#else
  const uint8_t *data = reinterpret_cast<const uint8_t *>(packet->data().data());
#endif

  parser_info.packet_timestamp_ = get_now_microsecond();
  frame_end_utime_ = parser_info.packet_timestamp_;

  if (unlikely(cloud_ == nullptr)) {
    if (!update_frame(parser_info.packet_timestamp_, parser_info)) {
      return false;
    }
  } else {
    if (frame_end_) {
#ifdef WITH_ROS2
      auto header = cloud_->cloud_msg_->header;
      header.stamp.sec = frame_end_utime_ / 1000000;
      header.stamp.nanosec = frame_end_utime_ % 1000000 * 1000;
      cloud_->cloud_msg_->data.resize(lidar_point_sum_ * sizeof(LidarPoint));
      cloud_->cloud_msg_->width = lidar_point_sum_ / packet_config.lasers();
      cloud_->cloud_msg_->height = packet_config.lasers();
      cloud_->cloud_msg_->row_step = sizeof(LidarPoint) * cloud_->cloud_msg_->width;
      cloud_->last_packet_utime_ = packet->time_system;
      *cloud = cloud_;
#else
      auto header = cloud_->proto_cloud_->mutable_header();
      header->set_sequence_num(frame_seq_++);
      header->set_lidar_timestamp(frame_end_utime_);
      cloud_->proto_cloud_->mutable_data()->resize(lidar_point_sum_ * sizeof(LidarPoint));
      cloud_->proto_cloud_->set_width(lidar_point_sum_ / packet_config.lasers());
      cloud_->proto_cloud_->set_height(packet_config.lasers());
      cloud_->proto_cloud_->set_row_step(sizeof(LidarPoint) * cloud_->proto_cloud_->width());
      cloud_->last_packet_utime_ = packet->time_system();
      *cloud = cloud_;
#endif
      ret = true;
      lidar_point_sum_ = 0;
      frame_end_ = false;
#ifdef WITH_ROS2
      if (!update_frame(packet->time_system, parser_info)) {
        return false;
      }
#else
      if (!update_frame(packet->time_system(), parser_info)) {
        return false;
      }
#endif
    }
  }
  pixel_xyz(data);
  return ret;
}

}  // namespace airi
}  // namespace crdc

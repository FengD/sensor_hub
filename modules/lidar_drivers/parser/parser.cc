// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: lidar parser

#include <vector>
#include <algorithm>
#include "lidar_drivers/parser/parser.h"
#define GAP_SHREHOLD 500
#define TIME_DIFF_SHREHOLD 50000

namespace crdc {
namespace airi {

bool LidarParser::init(const ParserConfig& config) {
  config_ = config;
  time_zone_microseconds_ = int64_t(config_.time_zone()) * 3600 * 1000000;

  if (!init_pool()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] failed to init pool.";
    return false;
  }

  if (!init_lidar_parser()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] failed to init lidar parser.";
    return false;
  }

  if (!init_calib()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] failed to init calib.";
    return false;
  }

  return true;
}

bool LidarParser::init_pool() {
  cloud_pool_.reset(new common::CCObjectPool<LidarPointCloud>(config_.pool_size()));
  cloud_pool_->ConstructAll();
  std::vector<std::shared_ptr<LidarPointCloud>> temp_clouds;
  for (auto i = 0; i < config_.pool_size(); ++i) {
    auto cloud = cloud_pool_->GetObject();
    if (cloud == nullptr) {
      LOG(ERROR) << "[" << config_.frame_id() << "] failed to getobject: " << i;
      return false;
    }

    cloud->proto_cloud_ = std::make_shared<PointCloud2>();
    cloud->proto_cloud_->clear_fields();
    PointField* f;
    ADD_FIELD(cloud->proto_cloud_, "x", PointField::FLOAT32, 0, 1);
    ADD_FIELD(cloud->proto_cloud_, "y", PointField::FLOAT32, 4, 1);
    ADD_FIELD(cloud->proto_cloud_, "z", PointField::FLOAT32, 8, 1);
    ADD_FIELD(cloud->proto_cloud_, "intensity", PointField::UINT8, 12, 1);
    // ADD_FIELD(cloud->proto_cloud_, "distance", PointField::FLOAT32, 13, 1);
    // ADD_FIELD(cloud->proto_cloud_, "sec", PointField::UINT32, 17, 1);
    // ADD_FIELD(cloud->proto_cloud_, "usec", PointField::UINT32, 21, 1);
    // ADD_FIELD(cloud->proto_cloud_, "ring", PointField::UINT16, 25, 1);
    // ADD_FIELD(cloud->proto_cloud_, "azimuth", PointField::FLOAT32, 27, 1);
    // ADD_FIELD(cloud->proto_cloud_, "elevation", PointField::FLOAT32, 31, 1);
    // ADD_FIELD(cloud->proto_cloud_, "semantic_flag", PointField::UINT8, 35, 1);
    cloud->proto_cloud_->mutable_data()->resize(config_.max_points() * sizeof(LidarPoint));
    cloud->proto_cloud_->set_point_step(sizeof(LidarPoint));
    cloud->proto_cloud_->set_height(config_.lidar_packet_config().lasers());
    cloud->proto_cloud_->set_is_dense(true);
    cloud->proto_cloud_->mutable_header()->set_frame_id(config_.frame_id());
    cloud->proto_cloud_->set_is_bigendian(false);
    temp_clouds.emplace_back(cloud);
  }

  temp_clouds.clear();
  return true;
}

bool LidarParser::init_calib() {
  auto& packet_config = config_.lidar_packet_config();
  if (packet_config.ring_map_size() == 0) {
    for (auto i = 0; i < packet_config.lasers(); ++i) {
      config_.mutable_lidar_packet_config()->add_ring_map(i);
    }
  }

  LOG(INFO) << "[" << config_.frame_id() << "] " << config_.DebugString();
  if (packet_config.ring_map_size() != packet_config.lasers() ||
      packet_config.calib_azimuth_size() != packet_config.lasers() ||
      packet_config.calib_elevation_size() != packet_config.lasers()) {
    LOG(INFO) << "[" << config_.frame_id() << "] config size is wrong.";
    return false;
  }

  for (auto laser = 0; laser < packet_config.lasers(); ++laser) {
    LidarCalibInfo info;
    info.calib_azimuth_ = packet_config.calib_azimuth(laser);
    info.offset_x_ = 0;
    info.elevation_cos_ = std::cos(packet_config.calib_elevation(laser));
    info.elevation_sin_ = std::sin(packet_config.calib_elevation(laser));
  }

  return true;
}

bool LidarParser::is_frame_end(const uint64_t& azimuth) {
  auto last_azimuth = last_pixel_azimuth_;
  last_pixel_azimuth_ = azimuth;
  uint16_t gap = last_azimuth > azimuth ?
                 (azimuth + ROTATION_MAX_UNITS - last_azimuth) : (azimuth - last_azimuth);
  LOG_IF(WARNING, gap > GAP_SHREHOLD) << "Azimuth gap: " << gap;
  max_azimuth_gap_ = std::max(gap, max_azimuth_gap_);
  if (azimuth >= config_.split_azimuth() &&
      (azimuth < last_azimuth || last_azimuth < config_.split_azimuth())) {
    return true;
  }

  return false;
}

bool LidarParser::is_lidar_packet_valid(const Packet* packet) {
  if (packet->size() != config_.lidar_packet_config().size()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] packet size " << packet->size()
               << " wrong " << config_.lidar_packet_config().size();
    return false;
  }

  const uint8_t* data = (const uint8_t*) packet->data().data();
  uint32_t header_checksum = (data[0] << 8) | (data[1]);
  if (header_checksum != config_.lidar_packet_config().check_sum()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] checksum " << header_checksum
               << " wrong " << config_.lidar_packet_config().check_sum();
    return false;
  }
  return true;
}

uint64_t LidarParser::get_timestamp(const Packet* packet) {
  device_timestamp_ = get_packet_timestamp(packet);
  auto timestamp = device_timestamp_;
  auto diff = packet->time_system() > timestamp ?
              packet->time_system() - timestamp : timestamp - packet->time_system();
  if (config_.correct_utime() && diff > TIME_DIFF_SHREHOLD) {
    LOG_EVERY_N(ERROR, 100) << "[" << config_.frame_id() << "] got unexpected device time: "
                            << device_timestamp_ << ", override with system utime: "
                            << packet->time_system();
    timestamp = packet->time_system();
  }

  return timestamp;
}

bool LidarParser::parse_lidar_packet(const Packet* packet,
                                     std::shared_ptr<LidarPointCloud>* cloud) {
  bool ret = false;
  if (!is_lidar_packet_valid(packet)) {
    LOG(ERROR) << "[" << config_.frame_id() << "] packet is invalid.";
    return ret;
  }

  if (lidar_point_count_ > static_cast<int32_t>(config_.max_points()) - packet_points_) {
    LOG(ERROR) << "[" << config_.frame_id() << "] lidar point is close to max ["
               << lidar_point_count_ << ", " << config_.max_points() << "]";
  }

  parse_lidar_other_info(packet);
  auto& packet_config = config_.lidar_packet_config();
  LidarParserInfo parser_info;
  parser_info.packet_timestamp_ = get_timestamp(packet);

  const uint8_t* data = reinterpret_cast<const uint8_t*>(packet->data().data())
                        + packet_config.block_offset();
  for (parser_info.block_ = 0; parser_info.block_ < packet_config.blocks(); ++parser_info.block_) {
    get_block_info(data, parser_info);

    if (parser_info.start_of_block_ != packet_config.block_check_sum()) {
      LOG(WARNING) << "[" << config_.frame_id() << "] invalid header "
                   << parser_info.block_ << ", " << std::hex << parser_info.start_of_block_;
      continue;
    }

    if (parser_info.pixel_azimuth_ > ROTATION_MAX_UNITS) {
      continue;
    }

    parser_info.pixel_azimuth_diff_ = (ROTATION_MAX_UNITS + parser_info.pixel_azimuth_
                                       - last_pixel_azimuth_) % ROTATION_MAX_UNITS;
    if (unlikely(cloud_ == nullptr)) {
      if (!is_frame_end(parser_info.pixel_azimuth_)) {
        return false;
      }

      if (!update_frame(parser_info.packet_timestamp_, parser_info)) {
        return false;
      }
    } else {
      if (is_frame_end(parser_info.pixel_azimuth_)) {
        calculate_lidar_point_pos();
        auto header = cloud_->proto_cloud_->mutable_header();
        header->set_sequence_num(frame_seq_++);
        header->set_lidar_timestamp(frame_end_utime_);
        cloud_->proto_cloud_->mutable_data()->resize(lidar_point_count_ * sizeof(LidarPoint));
        cloud_->proto_cloud_->set_width(lidar_point_count_ / packet_config.lasers());
        cloud_->proto_cloud_->set_row_step(sizeof(LidarPoint) * cloud_->proto_cloud_->width());
        cloud_->last_packet_utime_ = packet->time_system();
        *cloud = cloud_;
        ret = true;
        if (!update_frame(packet->time_system(), parser_info)) {
          return false;
        }
      }
    }

    auto p = data + packet_config.firing_offset();
    for (parser_info.firing_ = 0; parser_info.firing_ < packet_config.firings();
         ++parser_info.firing_) {
      for (parser_info.laser_ = 0; parser_info.laser_ < packet_config.lasers();
           ++parser_info.laser_) {
        get_point_raw_info(p, parser_info);
        p += packet_config.point_size();
        const auto ring = packet_config.ring_map(parser_info.laser_);
        LidarPoint &pt = lidar_point_[lidar_point_count_
                         + calib_info_[parser_info.laser_].offset_x_ + ring];
        calibrate_point(parser_info);
        calculate_points_data_xyz(parser_info, pt);
        // pt.ring_ = ring;
        // pt.timestamp_ = parser_info.timestamp_;
        // pt.intensity_ = parser_info.intensity_;
        // pt.distance = parser_info.distance_;
        if (is_invalid_point(parser_info)) {
          lidar_point_count_--;
          continue;
        }
      }
      lidar_point_count_ += packet_config.lasers();
    }
    frame_end_utime_ = parser_info.timestamp_;
    data += packet_config.block_size();
  }
  return ret;
}

void LidarParser::calculate_points_data_xyz(const LidarParserInfo& parser_info, LidarPoint& pt) {
  auto& packet_config = config_.lidar_packet_config();
  double azimuth_rad = parser_info.azimuth_ * RAD_PER_DEGREE;
  double xy_distance = parser_info.distance_ * calib_info_[parser_info.laser_].elevation_sin_;
  pt.x_ = xy_distance * std::cos(azimuth_rad);
  pt.y_ = -xy_distance * std::sin(azimuth_rad);
  pt.z_ = parser_info.distance_ * calib_info_[parser_info.laser_].elevation_sin_;
}

}  // namespace airi
}  // namespace crdc

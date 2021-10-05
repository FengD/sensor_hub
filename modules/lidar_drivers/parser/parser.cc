// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: lidar parser

#include <vector>
#include "lidar_drivers/parser/parser.h"

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

bool LidarParser::parse_lidar_packet(const Packet* raw_packet, 
                                     std::shared_ptr<PointCloud>* cloud) {

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

    cloud->proto_cloud_ = std::make_shared<PointCloud>();
    cloud->proto_cloud_->Clear();
    cloud->proto_cloud_->set_height(config_.lidar_packet_config().lasers());
    cloud->proto_cloud_->set_width(1);
    cloud->proto_cloud_->set_is_dense(true);
    cloud->proto_cloud_->set_frame_id(config_.frame_id());
    temp_clouds.emplace_back(cloud);
  }

  temp_clouds.clear();
  return true;
}

bool LidarParser::init_lidar_parser() {

}

bool LidarParser::init_calib() {

}

bool LidarParser::is_frame_end(const uint64_t& azimuth) {

}

bool LidarParser::is_lidar_packet_valid(const Packet* packet) {

}

uint64_t LidarParser::get_timestamp(const Packet* packet) {

}

void LidarParser::calculate_points_data_xyz(const Packet* packet, LidarPoint& pt) {

}

}  // namespace airi
}  // namespace crdc
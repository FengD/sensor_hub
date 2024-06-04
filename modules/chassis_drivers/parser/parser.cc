// Copyright (C) 2021 Hirain Technologies Inc.
// License: Modified BSD Software License Agreement
// Author: shichong.wang
// Description: chassis parser for asensing chassis570d device

#include "chassis_drivers/parser/parser.h"
#include <algorithm>
#include <vector>

#define GAP_SHREHOLD 500
#define TIME_DIFF_SHREHOLD 50000

namespace crdc {
namespace airi {

bool ChassisParser::init(const ChassisParserConfig& config) {
  config_ = config;
  time_zone_microseconds_ = int64_t(config_.time_zone()) * 3600 * 1000000;

  if (!init_chassis_parser()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] failed to init chassis parser.";
    return false;
  }

  return true;
}

bool ChassisParser::init_pool() {
  chassis_data_pool_.reset(new common::CCObjectPool<ChassisData>(config_.pool_size()));
  chassis_data_pool_->ConstructAll();
  std::vector<std::shared_ptr<ChassisData>> temp_chassis_data_pool;
  for (auto i = 0; i < config_.pool_size(); ++i) {
    auto chassis_data = chassis_data_pool_->GetObject();
    if (chassis_data == nullptr) {
      LOG(ERROR) << "[" << config_.frame_id() << "] failed to getobject: " << i;
      return false;
    }

    LOG(INFO) << "init pool in loop: " << i << std::endl;
    chassis_data->proto_ins_data_ = std::make_shared<Ins>();
    chassis_data->proto_ins_data_->header.frame_id = config_.frame_id();
    chassis_data->proto_ins_data_->linear_velocity.x = 0;
    chassis_data->proto_ins_data_->linear_velocity.y = 0;
    chassis_data->proto_ins_data_->linear_velocity.z = 0;

    temp_chassis_data_pool.emplace_back(chassis_data);
  }

  temp_chassis_data_pool.clear();
  return true;
}

bool ChassisParser::is_chassis_packet_valid(const Packet* packet) {
  auto packet_size = packet->size;
  if (packet_size != config_.chassis_packet_config().size()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] packet size "
               << packet_size << " wrong "
               << config_.chassis_packet_config().size();
    return false;
  }

#ifdef USE_CHECKSUM
  const uint8_t* data = (const uint8_t*)packet->data().data();
  uint32_t header_checksum =
      (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
  if (header_checksum != config_.chassis_packet_config().check_sum()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] checksum " << header_checksum
               << " wrong " << config_.chassis_packet_config().check_sum();
    return false;
  }
#endif

  return true;
}

uint64_t ChassisParser::get_timestamp(const Packet* packet) {
  device_timestamp_ = get_packet_timestamp(packet);
  auto timestamp = device_timestamp_;
  auto time_system = packet->time_system;
  auto diff = time_system > timestamp
                  ? time_system - timestamp
                  : timestamp - time_system;
  if (config_.correct_utime() && diff > TIME_DIFF_SHREHOLD) {
    LOG_EVERY_N(ERROR, 100)
        << "[" << config_.frame_id()
        << "] got unexpected device time: " << device_timestamp_
        << ", override with system utime: " << time_system;
    timestamp = time_system;
  }

  return timestamp;
}

bool ChassisParser::parse_chassis_packet(const Packet* packet,
                                 std::shared_ptr<ChassisData>* chassis_data) {
  bool ret = false;
  if (!is_chassis_packet_valid(packet)) {
    LOG(ERROR) << "[" << config_.frame_id() << "] packet is invalid.";
    return ret;
  }

  ChassisParserInfo parser_info;
  if (!update_frame(parser_info.packet_timestamp_, parser_info)) {
    return false;
  }

  parse_chassis_can_frame(reinterpret_cast<char *>(
                          const_cast<unsigned char*>(packet->data.data())));
  *chassis_data = chassis_data_;
  ret = true;

  return ret;
}

}  // namespace airi
}  // namespace crdc

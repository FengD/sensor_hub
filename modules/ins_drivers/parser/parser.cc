// Copyright (C) 2021 Hirain Technologies Inc.
// License: Modified BSD Software License Agreement
// Author: shichong.wang
// Description: ins parser for asensing ins570d device

#include "ins_drivers/parser/parser.h"
#include <algorithm>
#include <vector>

#define GAP_SHREHOLD 500
#define TIME_DIFF_SHREHOLD 50000

namespace crdc {
namespace airi {

bool InsParser::init(const ParserConfig& config) {
  config_ = config;
  time_zone_microseconds_ = int64_t(config_.time_zone()) * 3600 * 1000000;

  if (!init_pool()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] failed to init pool.";
    return false;
  }

  if (!init_ins_parser()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] failed to init ins parser.";
    return false;
  }

  return true;
}

bool InsParser::init_pool() {
  ins_data_pool_.reset(new common::CCObjectPool<InsData>(config_.pool_size()));
  ins_data_pool_->ConstructAll();
  std::vector<std::shared_ptr<InsData>> temp_ins_data_pool;
  for (auto i = 0; i < config_.pool_size(); ++i) {
    auto ins_data = ins_data_pool_->GetObject();
    if (ins_data == nullptr) {
      LOG(ERROR) << "[" << config_.frame_id() << "] failed to getobject: " << i;
      return false;
    }

    std::cout << "init pool in loop: " << i << std::endl;
    ins_data->proto_ins_data_ = std::make_shared<Ins>();
    ins_data->proto_ins_data_->mutable_header()->set_frame_id(config_.frame_id());
    ins_data->proto_ins_data_->mutable_linear_velocity()->set_x(0);
    ins_data->proto_ins_data_->mutable_linear_velocity()->set_y(0);
    ins_data->proto_ins_data_->mutable_linear_velocity()->set_z(0);

    temp_ins_data_pool.emplace_back(ins_data);
  }

  temp_ins_data_pool.clear();
  return true;
}

bool InsParser::is_ins_packet_valid(const Packet* packet) {
  if (packet->size() != config_.ins_packet_config().size()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] packet size "
               << packet->size() << " wrong "
               << config_.ins_packet_config().size();
    return false;
  }

#ifdef USE_CHECKSUM
  const uint8_t* data = (const uint8_t*)packet->data().data();
  uint32_t header_checksum =
      (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
  if (header_checksum != config_.ins_packet_config().check_sum()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] checksum " << header_checksum
               << " wrong " << config_.ins_packet_config().check_sum();
    return false;
  }
#endif

  return true;
}

uint64_t InsParser::get_timestamp(const Packet* packet) {
  device_timestamp_ = get_packet_timestamp(packet);
  auto timestamp = device_timestamp_;
  auto diff = packet->time_system() > timestamp
                  ? packet->time_system() - timestamp
                  : timestamp - packet->time_system();
  if (config_.correct_utime() && diff > TIME_DIFF_SHREHOLD) {
    LOG_EVERY_N(ERROR, 100)
        << "[" << config_.frame_id()
        << "] got unexpected device time: " << device_timestamp_
        << ", override with system utime: " << packet->time_system();
    timestamp = packet->time_system();
  }

  return timestamp;
}

bool InsParser::parse_ins_packet(const Packet* packet,
                                 std::shared_ptr<InsData>* ins_data) {
  bool ret = false;
  if (!is_ins_packet_valid(packet)) {
    LOG(ERROR) << "[" << config_.frame_id() << "] packet is invalid.";
    return ret;
  }

  // TODO(shichong.wang): use parser_info
  InsParserInfo parser_info;
  if (!update_frame(parser_info.packet_timestamp_, parser_info)) {
    return false;
  }

  parse_ins_can_frame(const_cast<char*>(packet->data().data()));
  *ins_data = ins_data_;
  ret = true;

  return ret;
}

}  // namespace airi
}  // namespace crdc

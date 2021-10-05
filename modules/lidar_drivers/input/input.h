// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: input

#pragma once

#include <memory>
#include <string>
#include "common/common.h"
#include "lidar_drivers/proto/lidar_config.pb.h"
#include "cyber/sensor_proto/eth_packet.pb.h"

namespace crdc {
namespace airi {

class LidarInput {
 public:
  LidarInput() = default;
  virtual ~LidarInput() = default;

  virtual bool init(const LidarInputConfig& config) {
    return false;
  }

  virtual int32_t get_lidar_data(Packet** packet) {
    return DeviceStatus::NO_ERROR;
  }

  virtual std::string get_name() const = 0;

  bool is_packet_pool_full() {
    return packet_cur_index_ >= config_.pool_size();
  }

  void clear_pool() {
    packet_cur_index_ = 0;
  }

  int32_t get_lidar_packets(std::shared_ptr<Packets>* packets) {
    *packets = packets_;
    return packet_cur_index_;
  }


 protected:
  Packet* get_raw_packet() {
    if (packet_cur_index_ >= config_.pool_size()) {
      LOG(ERROR) << "[" << config_.frame_id() << "] pool is full.";
      return nullptr;
    }

    return packets_->mutable_packet(packet_cur_index_++);
  }

  bool init_pool() {
    packet_cur_index_ = 0;
    packets_ = std::make_shared<Packets>();
    for (auto i = 0; i < config_.pool_size(); ++i) {
      auto packet = packets_->add_packet();
      packet->mutable_data()->reserve(config_.packet_size());
      packet->set_version(0x0001);
    }
    return true;
  }

  LidarInputConfig config_;
  std::shared_ptr<Packets> packets_ = nullptr;
  int32_t packet_cur_index_;
};

REGISTER_COMPONENT(LidarInput);
#define REGISTER_LIDAR_INPUT(name) REGISTER_CLASS(LidarInput, name)

}  // namespace airi
}  // namespace crdc

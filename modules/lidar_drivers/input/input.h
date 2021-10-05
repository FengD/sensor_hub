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

  /**
   * @brief init the LidarInput, need to be redefined in subclass
   * @param LidarInputConfig
   * @return status
   */
  virtual bool init(const LidarInputConfig& config) {
    return false;
  }

  /**
   * @brief get lidar data, need to be redefined in subclass
   * @param packet lists
   * @return status
   */
  virtual int32_t get_lidar_data(Packet** packet) {
    return DeviceStatus::NO_ERROR;
  }

  virtual std::string get_name() const = 0;

  /**
   * @brief Check if t
   * @return statushe packet pool is full.
   */
  bool is_packet_pool_full() {
    return packet_cur_index_ >= config_.pool_size();
  }

  /**
   * @brief Clear the packet pool
   */
  void clear_pool() {
    packet_cur_index_ = 0;
  }

  /**
   * @brief Get all the lidar packets
   * @param packet
   * @return packet size
   */
  int32_t get_lidar_packets(std::shared_ptr<Packets>* packets) {
    *packets = packets_;
    return packet_cur_index_;
  }


 protected:
  /**
   * @brief get raw packet
   * @return packet
   */
  Packet* get_raw_packet() {
    if (packet_cur_index_ >= config_.pool_size()) {
      LOG(ERROR) << "[" << config_.frame_id() << "] pool is full.";
      return nullptr;
    }

    return packets_->mutable_packet(packet_cur_index_++);
  }

  /**
   * @brief init the pool
   * @return status
   */
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

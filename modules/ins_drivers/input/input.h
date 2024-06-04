// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: input

#pragma once

#include <memory>
#include <string>
#include "common/common.h"
#include "ins_drivers/proto/ins_config.pb.h"
#ifdef WITH_ROS2
#include "sensor_msg/msg/packet.hpp"
#include "sensor_msg/msg/packets.hpp"
#else
#include "cyber/sensor_proto/eth_packet.pb.h"
#endif

#ifdef WITH_ROS2
using Packet = sensor_msg::msg::Packet;
using Packets = sensor_msg::msg::Packets;
#endif

namespace crdc {
namespace airi {

class InsInput {
 public:
  InsInput() = default;
  virtual ~InsInput() = default;

  /**
   * @brief init the InsInput, need to be redefined in subclass
   * @param InsInputConfig
   * @return status
   */
  virtual bool init(const InsInputConfig& config) {
    (void)config;
    return false;
  }

  /**
   * @brief get ins data, need to be redefined in subclass
   * @param packet lists
   * @return status
   */
  virtual int32_t get_ins_data(Packet** packet) {
    (void)packet;
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
   * @brief Get all the ins packets
   * @param packet
   * @return packet size
   */
  int32_t get_ins_packets(std::shared_ptr<Packets>* packets) {
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

#ifdef WITH_ROS2
    return &(packets_->packet[packet_cur_index_++]);
#else
    return packets_->mutable_packet(packet_cur_index_++);
#endif
  }

  /**
   * @brief init the pool
   * @return status
   */
  bool init_pool() {
    packet_cur_index_ = 0;
    packets_ = std::make_shared<Packets>();
#ifdef WITH_ROS2
    Packet* packet = new Packet();
    for (auto i = 0; i < config_.pool_size(); ++i) {
      packet->data.reserve(config_.packet_size());
      packet->version = 0x0001;
      packets_->packet.emplace_back(*packet);
    }
    delete packet;
#else
    for (auto i = 0; i < config_.pool_size(); ++i) {
      auto packet = packets_->add_packet();
      packet->mutable_data()->reserve(config_.packet_size());
      packet->set_version(0x0001);
    }
#endif
    return true;
  }

  InsInputConfig config_;
  std::shared_ptr<Packets> packets_ = nullptr;
  int32_t packet_cur_index_;
};

REGISTER_COMPONENT(InsInput);
#define REGISTER_INS_INPUT(name) REGISTER_CLASS(InsInput, name)

}  // namespace airi
}  // namespace crdc

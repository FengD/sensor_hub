// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: input

#pragma once

#include <memory>
#include <string>
#include "common/common.h"
#include "chassis_drivers/proto/chassis_config.pb.h"
#ifdef WITH_ROS2
#include "sensor_msg/msg/packet.hpp"
#include "sensor_msg/msg/packets.hpp"
#endif

#ifdef WITH_ROS2
using Packet = sensor_msg::msg::Packet;
using Packets = sensor_msg::msg::Packets;
#endif

namespace crdc {
namespace airi {

class ChassisInput {
 public:
  ChassisInput() = default;
  virtual ~ChassisInput() = default;

  /**
   * @brief init the ChassisInput, need to be redefined in subclass
   * @param ChassisInputConfig
   * @return status
   */
  virtual bool init(const ChassisInputConfig& config) {
    return false;
  }

  /**
   * @brief get chassis data, need to be redefined in subclass
   * @param packet lists
   * @return status
   */
  virtual int32_t get_chassis_data(Packet** packet) {
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
   * @brief Get all the chassis packets
   * @param packet
   * @return packet size
   */
  int32_t get_chassis_packets(std::shared_ptr<Packets>* packets) {
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
#endif
    return true;
  }

  ChassisInputConfig config_;
  std::shared_ptr<Packets> packets_ = nullptr;
  int32_t packet_cur_index_;
};

REGISTER_COMPONENT(ChassisInput);
#define REGISTER_CHASSIS_INPUT(name) REGISTER_CLASS(ChassisInput, name)

}  // namespace airi
}  // namespace crdc

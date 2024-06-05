// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: input

#pragma once
#ifdef WITH_TEST
#include <gtest/gtest.h>
#endif
#include <memory>
#include <string>
#include "common/common.h"
#include "lidar_drivers/proto/lidar_config.pb.h"
#ifdef WITH_ROS2
#include "sensor_msg/msg/packets.hpp"
#include "sensor_msg/msg/packet.hpp"
#else
#include "cyber/sensor_proto/eth_packet.pb.h"
#endif

#ifdef WITH_ROS2
using Packet = sensor_msg::msg::Packet;
using Packets = sensor_msg::msg::Packets;
#endif

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
    (void)config;
    return false;
  }

  /**
   * @brief get lidar data, need to be redefined in subclass
   * @param packet lists
   * @return status
   */
  virtual int32_t get_lidar_data(Packet** packet) {
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

  LidarInputConfig config_;
  std::shared_ptr<Packets> packets_ = nullptr;
  int32_t packet_cur_index_;
  #ifdef WITH_TEST
  FRIEND_TEST(LidarTest, robosense_packet_test);
  FRIEND_TEST(LidarTest, vlp16_packet_test);
  FRIEND_TEST(LidarTest, innovizone_parser_test);
  FRIEND_TEST(LidarTest, phoenixA0_packet_test);
  FRIEND_TEST(LidarInputTest, cyber_init_test);
  #endif
};

REGISTER_COMPONENT(LidarInput);
#define REGISTER_LIDAR_INPUT(name) REGISTER_CLASS(LidarInput, name)

}  // namespace airi
}  // namespace crdc

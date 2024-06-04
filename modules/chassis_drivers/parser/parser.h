// Copyright (C) 2021 Hirain Technologies Inc.
// License: Modified BSD Software License Agreement
// Author: shichong.wang
// Description:

#pragma once

#include <memory>
#include <vector>
#include <string>
#include "common/common.h"
#include "chassis_drivers/proto/chassis_config.pb.h"
#include "sensor_msg/msg/packet.hpp"
#include "sensor_msg/msg/packets.hpp"
#include "sensor_msg/msg/ins.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using Ins = sensor_msg::msg::Ins;
using Imu = sensor_msgs::msg::Imu;
using Vehicle = geometry_msgs::msg::TwistStamped;
using Packet = sensor_msg::msg::Packet;
using Packets = sensor_msg::msg::Packets;


namespace crdc {
namespace airi {

struct ChassisData {
  uint64_t first_packet_utime_;
  uint64_t last_packet_utime_;
  std::shared_ptr<Ins> proto_ins_data_;
  std::shared_ptr<Imu> proto_imu_data_;
  std::shared_ptr<Vehicle> proto_vehicle_data_;
};

struct ChassisParserInfo {
  uint64_t packet_timestamp_ = 0;
  double longitude_ = 0;
  double latitude_ = 0;
  double altitude_ = 0;
  uint64_t timestamp_ = 0;
  uint8_t intensity_ = 0;
  ChassisParserInfo() {}
};

class ChassisParser {
 public:
  ChassisParser() = default;
  virtual ~ChassisParser() = default;
  /**
   * @brief Init the parser. It needs to be redefined for each subclass.
   * @param The chassis input config.
   * @return status
   */
  virtual bool init(const ChassisParserConfig& config);

  /**
   * @brief Parser the chassis packet to chassis_data
   * @param The input raw ethernet packet
   * @param The output chassis_data
   * @return status
   */
  virtual bool parse_chassis_packet(const Packet* packet,
                                std::shared_ptr<ChassisData>* chassis_data);
  virtual std::string get_name() const = 0;

 protected:
  /**
   * @brief Parser the chassis packet timestamp
   * @param The input raw ethernet packet
   * @return microseconds timestamp
   */
  uint64_t get_timestamp(const Packet* packet);

  /**
   * @brief Init chassis parser
   * @return status
   */
  virtual bool init_chassis_parser() { return false; }

  /**
   * @brief Check if chassis packet valid
   * @param The input raw ethernet packet
   * @return status
   */
  virtual bool is_chassis_packet_valid(const Packet* packet);

  /**
   * @brief If there is some other info in the packet
   * @param The input raw ethernet packet
   */
  virtual void parse_chassis_other_info(const Packet* packet) {}

  /**
   * @brief parse the timestamp in the packets, need to be redefined
   * @param The input raw ethernet packet
   */
  virtual uint64_t get_packet_timestamp(const Packet* packet) = 0;

  /**
   * @brief parse chassis can frame, need to be redfeined
   * @param can_frame_(input): ethernet packet data which include can info
   */
  virtual void parse_chassis_can_frame(char* can_frame_) = 0;

  /**
   * @brief Init chassis_data pool
   * @return status
   */
  virtual bool init_pool();

  /** TBD
   * @brief init the frame in each loop
   * @return status
   */
  inline bool frame_init() {
    chassis_data_ = chassis_data_pool_->GetObject();
    if (chassis_data_ == nullptr) {
      LOG(ERROR) << "[" << config_.frame_id()
                 << "] Failed to get chassis_data data.";
      return false;
    }
    return true;
  }

  /**
   * @brief update the frame
   * @param packet timestamp
   * @param chassis parser info
   * @return status
   */
  inline bool update_frame(const uint64_t& packet_time,
                           ChassisParserInfo& parser_info) {
    if (!frame_init()) {
      LOG(ERROR) << "[" << config_.frame_id() << "] Failed to init frame.";
      return false;
    }

    frame_start_utime_ = parser_info.timestamp_;
    chassis_data_->first_packet_utime_ = packet_time;
    return true;
  }

  ChassisParserConfig config_;
  int64_t time_zone_microseconds_;

  std::shared_ptr<common::CCObjectPool<ChassisData>> chassis_data_pool_ = nullptr;
  std::shared_ptr<ChassisData> chassis_data_;
  uint64_t device_timestamp_;
  uint64_t frame_start_utime_;
  uint64_t frame_end_utime_;
  uint32_t frame_seq_;
};

REGISTER_COMPONENT(ChassisParser);
#define REGISTER_CHASSIS_PARSER(name) REGISTER_CLASS(ChassisParser, name)

}  // namespace airi
}  // namespace crdc

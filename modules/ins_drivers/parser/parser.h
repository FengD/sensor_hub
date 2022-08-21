// Copyright (C) 2021 Hirain Technologies Inc.
// License: Modified BSD Software License Agreement
// Author: shichong.wang
// Description: ins parser for asensing ins570d device

#pragma once

#include <memory>
#include <vector>
#include "common/common.h"
#include "cyber/sensor_proto/eth_packet.pb.h"
#include "cyber/sensor_proto/ins.pb.h"
#include "ins_drivers/proto/ins_config.pb.h"

namespace crdc {
namespace airi {

struct InsData {
  uint64_t first_packet_utime_;
  uint64_t last_packet_utime_;
  std::shared_ptr<Ins> proto_ins_data_;
  InsData() {}
};

struct InsParserInfo {
  uint64_t packet_timestamp_ = 0;
  double longitude_ = 0;
  double latitude_ = 0;
  double altitude_ = 0;
  uint64_t timestamp_ = 0;
  uint8_t intensity_ = 0;
  InsParserInfo() {}
};

class InsParser {
 public:
  InsParser() = default;
  virtual ~InsParser() = default;
  /**
   * @brief Init the parser. It needs to be redefined for each subclass.
   * @param The ins input config.
   * @return status
   */
  virtual bool init(const ParserConfig& config);

  /**
   * @brief Parser the ins packet to ins_data
   * @param The input raw ethernet packet
   * @param The output ins_data
   * @return status
   */
  virtual bool parse_ins_packet(const Packet* packet,
                                std::shared_ptr<InsData>* ins_data);

 protected:
  /**
   * @brief Parser the ins packet timestamp
   * @param The input raw ethernet packet
   * @return microseconds timestamp
   */
  uint64_t get_timestamp(const Packet* packet);

  /**
   * @brief Init ins parser
   * @return status
   */
  virtual bool init_ins_parser() { return false; }

  /**
   * @brief Check if ins packet valid
   * @param The input raw ethernet packet
   * @return status
   */
  virtual bool is_ins_packet_valid(const Packet* packet);

  /**
   * @brief If there is some other info in the packet
   * @param The input raw ethernet packet
   */
  virtual void parse_ins_other_info(const Packet* packet) {}

  /**
   * @brief parse the timestamp in the packets, need to be redefined
   * @param The input raw ethernet packet
   */
  virtual uint64_t get_packet_timestamp(const Packet* packet) = 0;

  /**
   * @brief parse ins can frame, need to be redfeined
   * @param can_frame_(input): ethernet packet data which include can info
   */
  virtual void parse_ins_can_frame(char* can_frame_) = 0;

  /**
   * @brief Init ins_data pool
   * @return status
   */
  virtual bool init_pool();

  /** TBD
   * @brief init the frame in each loop
   * @return status
   */
  inline bool frame_init() {
    ins_data_ = ins_data_pool_->GetObject();
    if (ins_data_ == nullptr) {
      LOG(ERROR) << "[" << config_.frame_id()
                 << "] Failed to get ins_data data.";
      return false;
    }
    return true;
  }

  /**
   * @brief update the frame
   * @param packet timestamp
   * @param ins parser info
   * @return status
   */
  inline bool update_frame(const uint64_t& packet_time,
                           InsParserInfo& parser_info) {
    if (!frame_init()) {
      LOG(ERROR) << "[" << config_.frame_id() << "] Failed to init frame.";
      return false;
    }

    frame_start_utime_ = parser_info.timestamp_;
    ins_data_->first_packet_utime_ = packet_time;
    return true;
  }

  ParserConfig config_;
  int64_t time_zone_microseconds_;

  std::shared_ptr<common::CCObjectPool<InsData>> ins_data_pool_ = nullptr;
  std::shared_ptr<InsData> ins_data_;
  uint64_t device_timestamp_;
  uint64_t frame_start_utime_;
  uint64_t frame_end_utime_;
  uint32_t frame_seq_;
};

REGISTER_COMPONENT(InsParser);
#define REGISTER_INS_PARSER(name) REGISTER_CLASS(InsParser, name)

}  // namespace airi
}  // namespace crdc

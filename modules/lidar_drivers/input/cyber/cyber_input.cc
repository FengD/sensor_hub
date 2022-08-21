// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: input cyber

#include "lidar_drivers/input/cyber/cyber_input.h"

namespace crdc {
namespace airi {

bool CyberInput::init(const LidarInputConfig& config) {
  config_ = config;

  if (!config_.has_cyber_config()) {
    AERROR << "[" << config_.frame_id() << "] " << "has no cyber input config";
    return false;
  }

  reader_ = std::make_shared<RecordReader>(config_.cyber_config().file_path());
  const auto &type = reader_->GetMessageType(config_.cyber_config().channel());
  if (!type.empty()) {
    CHECK_EQ(type, "crdc.airi.Packets");
  } else {
    AERROR << "[" << config_.frame_id() << "] "
           << "Failed to get message type for " << config_.cyber_config().channel();
    return false;
  }

  if (!init_pool()) {
    AERROR << "[" << config_.frame_id() << "] " << "Failed to init raw data pool";
    return false;
  }

  return true;
}

int CyberInput::get_lidar_data(Packet** packet) {
  if (proto_packets_.packet_size() == 0 || cur_index_ >= proto_packets_.packet_size()) {
    apollo::cyber::record::RecordMessage msg;
    while (reader_->ReadMessage(&msg)) {
      if (msg.channel_name == config_.cyber_config().channel()) {
        proto_packets_.Clear();
        if (proto_packets_.ParseFromString(msg.content)) {
          cur_index_ = 0;
          break;
        }
      }
    }
    if (proto_packets_.packet_size() == 0 || cur_index_ >= proto_packets_.packet_size()) {
      AERROR << "[" << config_.frame_id() << "] " << "failed to get cyber message";
      return DeviceStatus::FAILED_GET_CYBER_MESSAGE;
    }
  }

  if (cur_index_ < proto_packets_.packet_size()) {
    Packet* raw_packet = get_raw_packet();
    if (!raw_packet) {
      AERROR << "[" << config_.frame_id() << "] " << "failed to get raw packet";
      return DeviceStatus::GET_RAW_PACKET_ERROR;
    }
    *raw_packet = proto_packets_.packet(cur_index_++);
    *packet = raw_packet;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return DeviceStatus::SUCCESS;
  }

  return DeviceStatus::NO_ERROR;
}

}  // namespace airi
}  // namespace crdc

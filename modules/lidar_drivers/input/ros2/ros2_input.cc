// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: input ros2

#include "lidar_drivers/input/ros2/ros2_input.h"

namespace sensor {
namespace hub {

bool ROS2Input::init(const LidarInputConfig& config) {
  config_ = config;

  if (!config_.has_ros2_config()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] " << "has no ros2 input config";
    return false;
  }

  if (config_.ros2_config().file_path().size() == 0) {
    LOG(FATAL) << "No ros2 record file given!";
  }

  file_index_ = 0;
  sleeptime_ = config_.ros2_config().sleeptime();
  reader_.open(config_.ros2_config().file_path(file_index_));
  auto topics = reader_.get_all_topics_and_types();
  if (!topics.empty()) {
    for (const auto & topic_with_type : topics) {
      topic_and_type[topic_with_type.name] = topic_with_type.type;
    }
    CHECK_EQ(topic_and_type[config_.ros2_config().channel()], "sensor_msg/msg/Packets");
  } else {
    LOG(ERROR) << "[" << config_.frame_id() << "] "
           << "Failed to get message type for " << config_.ros2_config().channel();
    return false;
  }

  if (!init_pool()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] " << "Failed to init raw data pool";
    return false;
  }

  return true;
}

int ROS2Input::get_lidar_data(Packet** packet) {
  rclcpp::Serialization<sensor_msg::msg::Packets> serialization_packets;
  if (proto_packets_.packet.size() == 0 || cur_index_ >= proto_packets_.packet.size()) {
    while (reader_.has_next()) {
      auto bag_message = reader_.read_next();
      if (bag_message->topic_name == config_.ros2_config().channel()) {
        rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
        serialization_packets.deserialize_message(&extracted_serialized_msg, &proto_packets_);
        cur_index_ = 0;
        break;
      }
    }
    if (proto_packets_.packet.size() == 0 || cur_index_ >= proto_packets_.packet.size()) {
      LOG(ERROR) << "[" << config_.frame_id() << "] " << "failed to get ros2 message";
      file_index_++;
      if (file_index_ < config_.ros2_config().file_path().size()) {
        reader_.open(config_.ros2_config().file_path(file_index_));
        LOG(ERROR) << "[INPUT_PACKET] " << config_.ros2_config().file_path(file_index_);
      } else {
        LOG(FATAL) << "Read ros2 Files Finished!";
      }
      return DeviceStatus::FAILED_GET_CYBER_MESSAGE;
    }
  }

  if (cur_index_ < proto_packets_.packet.size()) {
    Packet* raw_packet = get_raw_packet();
    if (!raw_packet) {
      LOG(ERROR) << "[" << config_.frame_id() << "] " << "failed to get raw packet";
      return DeviceStatus::GET_RAW_PACKET_ERROR;
    }
    *raw_packet = proto_packets_.packet[cur_index_++];
    *packet = raw_packet;
    std::this_thread::sleep_for(std::chrono::microseconds(sleeptime_));
    return DeviceStatus::SUCCESS;
  }

  return DeviceStatus::NO_ERROR;
}

}  // namespace hub
}  // namespace sensor

// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: input ros2

#pragma once
#include <memory>
#include <string>
#include <unordered_map>
#include "lidar_drivers/input/input.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"

namespace sensor {
namespace hub {


class ROS2Input : public LidarInput {
 public:
  ROS2Input() = default;
  virtual ~ROS2Input() = default;

  bool init(const LidarInputConfig& config) override;

  int get_lidar_data(Packet** packet) override;

  std::string get_name() const override { return "ROS2Input"; }

 private:
  rosbag2_cpp::Reader reader_;
  std::unordered_map<std::string, std::string> topic_and_type;
  sensor_msg::msg::Packets proto_packets_;
  uint cur_index_;
  int file_index_;
  int sleeptime_;
};

}  // namespace hub
}  // namespace sensor

// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: input cyber

#pragma once
#include <memory>
#include <string>
#include "lidar_drivers/input/input.h"
#include "cyber/record/record_reader.h"

namespace sensor {
namespace hub {

using apollo::cyber::record::RecordReader;

class CyberInput : public LidarInput {
 public:
  CyberInput() = default;
  virtual ~CyberInput() = default;

  bool init(const LidarInputConfig& config) override;

  int get_lidar_data(Packet** packet) override;

  std::string get_name() const override { return "CyberInput"; }

 private:
  std::shared_ptr<RecordReader> reader_;
  Packets proto_packets_;
  int cur_index_;
  int file_index_;
  int sleeptime_;
};

}  // namespace hub
}  // namespace sensor

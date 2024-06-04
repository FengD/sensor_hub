// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: input socket

#pragma once

#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <string>
#include <memory>
#include "lidar_drivers/input/input.h"
#include "lidar_drivers/input/socket/write.h"

namespace crdc {
namespace airi {
class SocketInput : public LidarInput {
 public:
  SocketInput() = default;
  virtual ~SocketInput();

  bool init(const LidarInputConfig& config) override;
  std::string get_name() const override {
    return "SocketInput";
  }
  int32_t get_lidar_data(Packet** packet) override;

 private:
  int32_t set_udp_socket(const uint16_t& port);
  int32_t check_socket_status();

  int32_t socket_num_ = 0;

  struct pollfd poll_fds_[2];
  uint16_t ports_[2];

  int32_t POLL_TIMEOUT;
  std::shared_ptr<SocketWrite> writing_config_;
  std::string multicast_ip_str_;
};

}  // namespace airi
}  // namespace crdc

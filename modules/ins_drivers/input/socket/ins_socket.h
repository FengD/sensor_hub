// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: input socket

#pragma once

#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <string>
#include "ins_drivers/input/input.h"

namespace crdc {
namespace airi {
class InsSocketInput : public InsInput {
 public:
  InsSocketInput() = default;
  virtual ~InsSocketInput();

  bool init(const InsInputConfig& config) override;
  std::string get_name() const override {
    return "InsSocketInput";
  }
  int32_t get_ins_data(Packet** packet) override;

 private:
  int32_t set_udp_socket(const uint16_t& port);
  int32_t check_socket_status();

  int32_t socket_num_ = 0;

  struct pollfd poll_fds_[2];
  uint16_t ports_[2];

  static const int32_t POLL_TIMEOUT = 1000;
};

}  // namespace airi
}  // namespace crdc

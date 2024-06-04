#pragma once

#include <pcap.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string>
#include <memory>
#include <cstring>
#include "ins_drivers/input/input.h"
#ifdef WITH_ROS2
#include <rclcpp/rclcpp.hpp>
#include "sensor_msg/msg/ins.hpp"
#endif
namespace crdc {
namespace airi {

class InsPcapInput : public InsInput {
 public:
  InsPcapInput() = default;
  virtual ~InsPcapInput() {
    pcap_close(pcap_handle_);
  }

  bool init(const InsInputConfig& config) override;
  std::string get_name() const override {
    return "InsPcapInput";
  }
  int32_t get_ins_data(Packet** packet) override;

 private:
  std::string file_name_;
  pcap_t *pcap_handle_;
  char error_buf[PCAP_ERRBUF_SIZE];
  struct pcap_pkthdr* header_;
  const unsigned char* pkt_data_;
  const int PCAP_HEAD_DIZE = 46;
  const int SOMEIP_HEAD_SIZE = 16;
  std::string topic_;
  int packet_count_;
};

}  // namespace airi
}  // namespace crdc

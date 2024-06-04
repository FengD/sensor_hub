/**
  * Copyright (C) 2022 Hirain Technologies
  * License: Modified BSD Software License Agreement
  * Author: Liping.liu
  * Description: This file used to define the udp input
  */
#pragma once
#include <unistd.h>
#include <netinet/in.h>
#include <pcap.h>
#include <string>
#include <vector>
#include <memory>
#include "lidar_drivers/input/input.h"


namespace crdc {
namespace airi {

class PcapInput : public LidarInput {
 public:
  PcapInput() = default;
  virtual ~PcapInput();

  bool init(const LidarInputConfig& config) override;
  int initParam();
  int get_receive_len();
  int get_lidar_data(Packet** packet) override;
  int get_packet_data(char *pkt);
  std::string get_name() const override { return "PcapInput"; }

  void set_source_ip_str(const std::string& source_ip_str);
  void set_port(const int& port);
  void set_receive_len(const int& receive_len);

 private:
  std::string source_ip_str_;
  int port_;
  int file_index_;
  int file_count_;
  int receive_len_;
  std::string filename_;
  pcap_t *pcap_;
  bpf_program pcap_packet_filter_;
  char errbuf_[PCAP_ERRBUF_SIZE];
  bool read_once_ = false;
  bool read_fast_;
  int packet_rate_;
  int packet_count = 0;
  uint64_t timestamp_;
};

}  // namespace airi
}  // namespace crdc

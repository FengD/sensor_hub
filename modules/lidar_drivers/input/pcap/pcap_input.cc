// Copyright (C) 2022 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: input pcap
#include "lidar_drivers/input/pcap/pcap_input.h"
#include <arpa/inet.h>
#include <utility>

namespace sensor {
namespace hub {

PcapInput::~PcapInput() {
  pcap_close(pcap_);
}

bool PcapInput::init(const LidarInputConfig& config) {
    config_ = config;
    if (!config_.has_pcap_config()) {
      LOG(ERROR) << "[" << config_.frame_id() << "] has no PcapInput config";
      return false;
    }
    if (!config_.has_lidar_port()) {
      LOG(ERROR) << "[" << config_.frame_id() << "] has no lidar port";
      return false;
    }
    if (config_.pcap_config().file_path().size() == 0) {
      LOG(FATAL) << "No pcap file given!";
      return false;
    }
    file_count_ = config_.pcap_config().file_path().size();
    if (config_.pcap_config().source_ip_str().empty()) {
      LOG(FATAL) << "empty source ip!";
      return false;
    }
    if (!init_pool()) {
      LOG(ERROR) << "[" << config_.frame_id() << "] " << "Failed to init raw data pool";
      return false;
    }
    file_index_ = 0;
    filename_ = config_.pcap_config().file_path(file_index_);
    source_ip_str_ = config_.pcap_config().source_ip_str();
    port_ = config_.lidar_port();
    read_fast_ = config_.pcap_config().read_fast();
    packet_rate_ = config_.pcap_config().packet_rate();
    receive_len_ = config_.packet_size();
    int init_status = initParam();
    if (init_status != DeviceStatus::NO_ERROR) {
        return false;
    }
    return true;
}

int PcapInput::initParam() {
  if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_) ) == NULL) {
      LOG(ERROR) << "Error opening pcap file.\n";
      return DeviceStatus::CONNECT_ERROR;
  }
  std::stringstream filter;
  filter << "src host " << source_ip_str_ << " && ";
  filter << "udp dst port " << port_;
  pcap_compile(pcap_, &pcap_packet_filter_,
          filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
  return DeviceStatus::NO_ERROR;
}

int32_t PcapInput::get_lidar_data(Packet** packet) {
      Packet *raw_packet = get_raw_packet();
      if (!raw_packet) {
        LOG(ERROR) << "[" << config_.frame_id() << "] failed to get raw packet.";
        return DeviceStatus::GET_RAW_PACKET_ERROR;
      }

#ifdef WITH_ROS2
      raw_packet->data.resize(config_.packet_size());
      char* data_buf = reinterpret_cast<char*>(raw_packet->data.data());
#else
      raw_packet->mutable_data()->resize(config_.packet_size());
      char *data_buf = const_cast<char*>(raw_packet->mutable_data()->data());
#endif
      int status = get_packet_data(data_buf);
      if (status!= DeviceStatus::SUCCESS) {
          return status;
      }
      int receive_size = get_receive_len();

#ifdef WITH_ROS2
      raw_packet->data.resize(receive_size);
      raw_packet->size = receive_size;
      raw_packet->port = port_;
      raw_packet->time_system = timestamp_;
#else
      raw_packet->mutable_data()->resize(receive_size);
      raw_packet->set_size(receive_size);
      raw_packet->set_port(port_);
      raw_packet->set_time_system(timestamp_);
#endif
      *packet = raw_packet;
      return DeviceStatus::SUCCESS;
  return DeviceStatus::NO_ERROR;
}

int PcapInput::get_packet_data(char *pkt) {
    if (read_once_) {
        return DeviceStatus::NO_ERROR;
    }
    struct pcap_pkthdr *header;
    const u_char *pkt_data;
    while (true) {
        int res;
        if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0) {
            if (0 == pcap_offline_filter(&pcap_packet_filter_,
                                    header, pkt_data)) {
                continue;
            }
            if (!read_fast_) {
                usleep(packet_rate_);
            }
            set_receive_len(header->caplen-42);
            packet_count++;
            memcpy(&pkt[0], pkt_data + 42, receive_len_);
            timestamp_ = header->ts.tv_sec * 1000000 + header->ts.tv_usec;
            return DeviceStatus::SUCCESS;
        } else {
            LOG(INFO) << "pcap file " << filename_ << "is read completed";
            LOG(INFO) << "packet_count: " << packet_count;
            if (file_index_ < file_count_ - 1) {
                //  read next file
                file_index_ = file_index_ + 1;
                filename_ = config_.pcap_config().file_path(file_index_);
                initParam();
                continue;
            } else {
                LOG(INFO) << "end of file reached -- done reading.";
                read_once_ = true;
                return DeviceStatus::PCAP_END;
            }
        }
        return DeviceStatus::NO_ERROR;
    }
    return DeviceStatus::PCAP_END;
}

void PcapInput::set_source_ip_str(const std::string& source_ip_str) {
  source_ip_str_ = source_ip_str;
}
void PcapInput::set_port(const int& port) {
  port_ = port;
}
void PcapInput::set_receive_len(const int& receive_len) {
  receive_len_ = receive_len;
}
int PcapInput::get_receive_len() {
    return receive_len_;
}
}  // namespace hub
}  // namespace sensor

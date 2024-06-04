#include "common/common.h"
#include "ins_drivers/input/pcap/pcap_input.h"

namespace crdc {
namespace airi {

bool InsPcapInput::init(const InsInputConfig& config) {
  config_ = config;

  if (!config_.has_pcap_config()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] " << "has no pcap input config";
    return false;
  }

  if (config_.pcap_config().file_path().size() == 0) {
    LOG(FATAL) << "No pcap file given!";
  }

  file_name_ = config_.pcap_config().file_path();

  if (!(pcap_handle_ = pcap_open_offline(file_name_.c_str(), error_buf))) {
    LOG(ERROR) << "Error opening pcap file.\n";
    return false;
  }
  if (!init_pool()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] failed to init raw data pool.";
    return false;
  }
  packet_count_ = 0;
  return true;
}

int32_t InsPcapInput::get_ins_data(Packet** packet) {
  unsigned char service_id[2], method_id[2], payload_length[4], port[2];
  static std::once_flag s_flag;
  std::call_once(s_flag, []{
    LOG(INFO) << "Filter packets based ServiceID->0x0a01 and MethodID->0x9002";
  });

  Packet *raw_packet = get_raw_packet();
  if (!raw_packet) {
    LOG(ERROR) << "[" << config_.frame_id() << "] failed to get raw packet.";
    return DeviceStatus::GET_RAW_PACKET_ERROR;
  }

  while (pcap_next_ex(pcap_handle_, &header_, &pkt_data_) > 0) {
    std::memcpy(&port[0], pkt_data_ + 40, 2);
    std::memcpy(&service_id[0], pkt_data_ + PCAP_HEAD_DIZE, sizeof(service_id));
    std::memcpy(&method_id[0], pkt_data_ + PCAP_HEAD_DIZE + sizeof(service_id), sizeof(method_id));
    std::memcpy(&payload_length[0], pkt_data_ + PCAP_HEAD_DIZE +
            sizeof(service_id) + sizeof(method_id), sizeof(payload_length));
    if (service_id[0] == 0x0a && service_id[1] == 0x01 &&
            method_id[0] == 0x90 && method_id[1] == 0x02) {
      packet_count_++;
      int length = static_cast<int>(payload_length[3] | payload_length[2] << 8 |
                  payload_length[1] << 16 | payload_length[0] << 24) - 8;  // hex to int32
#ifdef WITH_ROS2
      raw_packet->data.resize(length);
      raw_packet->size = length;
      raw_packet->time_system = get_now_microsecond();
      raw_packet->port = static_cast<int32_t>(port[1] | port[0] << 8);  // hex to int16 to int32
      std::memcpy(raw_packet->data.data(), pkt_data_ + PCAP_HEAD_DIZE + SOMEIP_HEAD_SIZE, length);

#else
      raw_packet->mutable_data()->resize(length);
      raw_packet->set_size(length);
      raw_packet->set_time_system(get_now_microsecond());
      raw_packet->set_port(static_cast<int32_t>(port[1] | port[0] << 8));
      raw_packet->set_data(pkt_data_ + PCAP_HEAD_DIZE + SOMEIP_HEAD_SIZE,
              length);
#endif
      *packet = raw_packet;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      return DeviceStatus::SUCCESS;
    }
  }
  if (-2 == pcap_next_ex(pcap_handle_, &header_, &pkt_data_)) {
    LOG(WARNING) << "Pcap file is read completed. Valid packet_count: " << packet_count_;
    return DeviceStatus::PCAP_END;
  }
  return DeviceStatus::NO_ERROR;
}

}  // namespace airi
}  // namespace crdc

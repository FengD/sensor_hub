// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: ins

#include "ins_drivers/ins.h"
#ifdef WITH_ROS2
#include "ins_drivers/output/af_output.h"
#else
#include "ins_drivers/output/cyber_output.h"
#endif

namespace crdc {
namespace airi {
#ifdef WITH_ROS2
using Level = sensor_msg::msg::Level;
using InsOutput = InsAFOutput;
#else
using InsOutput = InsCyberOutput;
#endif

InsSensor::InsSensor(const InsComponentConfig& config) : common::Thread(true) {
  config_ = config;
  stop_ = false;
  ins_name_ = config_.frame_id();
  sensor_position_id_ = config_.sensor_position_id();
  std::string thread_name = ins_name_;
  if (thread_name.length() > MAX_THREAD_NAME_LENGTH) {
    thread_name = thread_name.substr(0, MAX_THREAD_NAME_LENGTH - 1);
  }
  set_thread_name(thread_name);

  if (config_.has_priority()) {
    set_priority(config_.priority());
  }

  std::string config_file_path =
      std::string(std::getenv("CRDC_WS")) + '/' + config_.config_file();
  if (!crdc::airi::util::is_path_exists(config_file_path)) {
    LOG(FATAL) << "[" << get_thread_name()
               << "] proto file not exits: " << config_file_path;
    return;
  }

  if (!crdc::airi::util::get_proto_from_file(config_file_path, &ins_config_)) {
    LOG(FATAL) << "[" << get_thread_name()
               << "] failed to read ins proto config: " << config_file_path;
    return;
  }

  LOG(INFO) << "[" << get_thread_name()
            << "] create InsSensor Instance: " << config_.DebugString()
            << ins_config_.DebugString();
  if (!init_parser() || !init_input()) {
    return;
  }
}

bool InsSensor::init_parser() {
  parser_ = nullptr;
  if (ins_config_.has_parser_config()) {
    ins_config_.mutable_parser_config()->set_frame_id(config_.frame_id());
    parser_ = InsParserFactory::get(ins_config_.parser_config().name());
    if (!parser_) {
      LOG(FATAL) << "[" << get_thread_name()
                 << "] Failed to get ins parser ptr: "
                 << ins_config_.parser_config().name();
      return false;
    }

    if (!parser_->init(ins_config_.parser_config())) {
      LOG(FATAL) << "[" << get_thread_name() << "] Failed to init ins parser: "
                 << ins_config_.parser_config().name();
      return false;
    }
  }

  return true;
}

bool InsSensor::init_input() {
  input_ = nullptr;
  if (ins_config_.has_input_config()) {
    ins_config_.mutable_input_config()->set_frame_id(config_.frame_id());
    input_ = InsInputFactory::get(ins_config_.input_config().name());
    if (!input_) {
      LOG(FATAL) << "[" << get_thread_name()
                 << "] Failed to get ins input ptr: "
                 << ins_config_.input_config().name();
      return false;
    }

    if (!input_->init(ins_config_.input_config())) {
      LOG(FATAL) << "[" << get_thread_name() << "] Failed to init ins input: "
                 << ins_config_.input_config().name();
      return false;
    }
  }

  return true;
}

void InsSensor::stop() {
  LOG(WARNING) << "[" << get_thread_name() << "] ins stop.";
  stop_ = false;
}

#ifdef WITH_ROS2
void InsSensor::send_diagnose_input(const uint32_t& position_id,
                                const DeviceStatus& error, const uint8_t& level,
                                const std::string& custom_desc, const std::string& context) {
    auto diagnose_input = DiagnoseInput(ModuleType::SENSOR_LIDAR, position_id,
                                      error, level, custom_desc, context);
    diagnose_inputs_.emplace_back(std::move(diagnose_input));
    common::Singleton<ModuleDiagnose>::get()->diagnose_list(diagnose_inputs_);
}
#else
void InsSensor::send_diagnose_input(const uint32_t& position_id,
                                const DeviceStatus& error, const Level& level,
                                const std::string& custom_desc, const std::string& context) {
    auto diagnose_input = DiagnoseInput(ModuleType::SENSOR_INS, position_id,
                                      error, level, custom_desc, context);
    diagnose_inputs_.emplace_back(std::move(diagnose_input));
    common::Singleton<ModuleDiagnose>::get()->diagnose_list(diagnose_inputs_);
}
#endif

void InsSensor::run() {
  while (!stop_) {
    Packet* raw_packet;
    int32_t code = input_->get_ins_data(&raw_packet);
    if (code != DeviceStatus::SUCCESS) {
      LOG(WARNING) << "[" << get_thread_name() << "] failed to get ins packet.";
      send_diagnose_input(sensor_position_id_, DeviceStatus(code), Level::ERROR,
                                "ins_driver_abnormal", "input_data_abnormal");
    } else {
#ifdef WITH_ROS2
      auto port = raw_packet->port;
#else
      auto port = raw_packet->port();
#endif
      if (port == ins_config_.input_config().ins_port()) {
        std::shared_ptr<InsData> ins_data;
        if (parser_->parse_ins_packet(raw_packet, &ins_data)) {
          // auto now = get_now_microsecond();

          /* TBD: no ins timestamp in proto
          auto receive_time = now -
          ins_data->proto_ins_data_->header().ins_timestamp();
          if (now < ins_data->proto_ins_data_->header().ins_timestamp()) {
              receive_time = ins_data->proto_ins_data_->header().ins_timestamp()
          - now;
          }
          LOG(INFO) << "[" << get_thread_name()
                    << "] [TIMER] [receive] elapsed_time(us): " << receive_time;
          */
#ifdef WITH_ROS2
          ins_data->proto_ins_data_->header.frame_id =
                        config_.frame_id();
#else
          ins_data->proto_ins_data_->mutable_header()->set_frame_id(
              config_.frame_id());
#endif
          auto now = get_now_microsecond();
          LOG(INFO) << "[" << get_thread_name()
                    << "] [TIMER] [callback] elapsed_time(us): "
                    << get_now_microsecond() - now;
          if (config_.has_channel_name()) {
            common::Singleton<InsOutput>::get()->write_ins_data(
                config_.channel_name(), ins_data->proto_ins_data_);
          }
        }
        send_diagnose_input(sensor_position_id_, DeviceStatus(code), Level::NOERR,
                                "ins_driver_normal", "input_data_normal");
      }
    }

    if (input_->is_packet_pool_full()) {
      std::shared_ptr<Packets> packets;
      input_->get_ins_packets(&packets);
      if (config_.has_raw_data_channel_name()) {
        common::Singleton<InsOutput>::get()->write_packet(
            config_.raw_data_channel_name(), packets);
      }
      input_->clear_pool();
    }
  }
}

}  // namespace airi
}  // namespace crdc

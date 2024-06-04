// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: chassis

#include "chassis_drivers/chassis.h"
#include "chassis_drivers/output/af_output.h"

namespace crdc {
namespace airi {
using Level = sensor_msg::msg::Level;
using ChassisOutput = ChassisAFOutput;

ChassisSensor::ChassisSensor(const ChassisComponentConfig& config) : common::Thread(true) {
  config_ = config;
  stop_ = false;
  chassis_name_ = config_.frame_id();
  sensor_position_id_ = config_.sensor_position_id();
  std::string thread_name = chassis_name_;
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

  if (!crdc::airi::util::get_proto_from_file(config_file_path, &chassis_config_)) {
    LOG(FATAL) << "[" << get_thread_name()
               << "] failed to read chassis proto config: " << config_file_path;
    return;
  }

  LOG(INFO) << "[" << get_thread_name()
            << "] create ChassisSensor Chassistance: " << config_.DebugString()
            << chassis_config_.DebugString();
  if (!init_parser() || !init_input()) {
    return;
  }
}

bool ChassisSensor::init_parser() {
  parser_ = nullptr;
  if (chassis_config_.has_parser_config()) {
    chassis_config_.mutable_parser_config()->set_frame_id(config_.frame_id());
    parser_ = ChassisParserFactory::get(chassis_config_.parser_config().name());
    if (!parser_) {
      LOG(FATAL) << "[" << get_thread_name()
                 << "] Failed to get chassis parser ptr: "
                 << chassis_config_.parser_config().name();
      return false;
    }

    if (!parser_->init(chassis_config_.parser_config())) {
      LOG(FATAL) << "[" << get_thread_name() << "] Failed to init chassis parser: "
                 << chassis_config_.parser_config().name();
      return false;
    }
  }

  return true;
}

bool ChassisSensor::init_input() {
  input_ = nullptr;
  if (chassis_config_.has_input_config()) {
    input_ = ChassisInputFactory::get(chassis_config_.input_config().name());
    if (!input_) {
      LOG(FATAL) << "[" << get_thread_name()
                 << "] Failed to get chassis input ptr: "
                 << chassis_config_.input_config().name();
      return false;
    }

    if (!input_->init(chassis_config_.input_config())) {
      LOG(FATAL) << "[" << get_thread_name() << "] Failed to init chassis input: "
                 << chassis_config_.input_config().name();
      return false;
    }
  }

  return true;
}

void ChassisSensor::stop() {
  LOG(WARNING) << "[" << get_thread_name() << "] chassis stop.";
  stop_ = false;
}

void ChassisSensor::send_diagnose_input(const uint32_t& position_id,
                                const DeviceStatus& error, const uint8_t& level,
                                const std::string& custom_desc, const std::string& context) {
    auto diagnose_input = DiagnoseInput(ModuleType::SENSOR_LIDAR, position_id,
                                      error, level, custom_desc, context);
    diagnose_inputs_.emplace_back(std::move(diagnose_input));
    common::Singleton<ModuleDiagnose>::get()->diagnose_list(diagnose_inputs_);
}

void ChassisSensor::run() {
  while (!stop_) {
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  LOG(INFO) << "[" << get_thread_name()
                    << "] [TIMER] [callback] run_time(us): "
                    << get_now_microsecond() - time_;
          time_ = get_now_microsecond();
  }
}

}  // namespace airi
}  // namespace crdc

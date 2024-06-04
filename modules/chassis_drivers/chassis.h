// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: chassis

#pragma once
#ifdef WITH_TEST
#include <gtest/gtest.h>
#endif
#include <utility>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include "common/common.h"
#include "module_diagnose/module_diagnose.h"
#include "chassis_drivers/input/input.h"
#include "chassis_drivers/parser/parser.h"
#include "chassis_drivers/proto/chassis_config.pb.h"
#include "chassis_drivers/proto/chassis_component_config.pb.h"

namespace crdc {
namespace airi {

class ChassisSensor : public common::Thread {
 public:
  explicit ChassisSensor(const ChassisComponentConfig &config);
  virtual ~ChassisSensor() = default;
  void stop();

  std::string get_name() const {
     return chassis_name_;
  }

 private:
  /**
   * @brief The run function execute the whole process of the chassis loop process.
   */
  void run() override;
  /**
   * @brief Init the encoder if defined.
   */
  bool init_parser();

  /**
   * @brief Init the input if defined.
   */
  bool init_input();

  /**
   * @brief Send diagnose message
   */
  void send_diagnose_input(const uint32_t& position_id,
                                const DeviceStatus& error, const uint8_t& level,
                                const std::string& custom_desc, const std::string& context);

  bool stop_;
  int32_t reload_;
  uint64_t time_;
  std::string chassis_name_;
  ChassisComponentConfig config_;
  ChassisConfig chassis_config_;
  std::shared_ptr<ChassisInput> input_;
  std::shared_ptr<ChassisParser> parser_;
  uint32_t sensor_position_id_;
  std::vector<DiagnoseInput> diagnose_inputs_;
};

}  // namespace airi
}  // namespace crdc

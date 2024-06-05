// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: ins

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
#include "ins_drivers/input/input.h"
#include "ins_drivers/parser/parser.h"
#include "ins_drivers/proto/ins_config.pb.h"
#include "ins_drivers/proto/ins_component_config.pb.h"
#ifdef WITH_ROS2
#include "sensor_msg/msg/ins.hpp"
#else
#include "cyber/sensor_proto/ins.pb.h"
#endif

#ifdef WITH_ROS2
using Packet = sensor_msg::msg::Packet;
using Ins = sensor_msg::msg::Ins;
#endif

namespace crdc {
namespace airi {

class InsSensor : public common::Thread {
 public:
  explicit InsSensor(const InsComponentConfig &config);
  virtual ~InsSensor() = default;
  void stop();

  std::string get_name() const {
     return ins_name_;
  }

 private:
  /**
   * @brief The run function execute the whole process of the ins loop process.
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
#ifdef WITH_ROS2
  void send_diagnose_input(const uint32_t& position_id,
                                const DeviceStatus& error, const uint8_t& level,
                                const std::string& custom_desc, const std::string& context);
#else
  void send_diagnose_input(const uint32_t& position_id,
                          const DeviceStatus& error, const Level& level,
                          const std::string& custom_desc,
                          const std::string& context);
#endif

  bool stop_;
  int32_t reload_;
  std::string ins_name_;
  InsComponentConfig config_;
  InsConfig ins_config_;
  std::shared_ptr<InsInput> input_;
  std::shared_ptr<InsParser> parser_;
  uint32_t sensor_position_id_;
  std::vector<DiagnoseInput> diagnose_inputs_;

  #ifdef WITH_TEST
  FRIEND_TEST(InsDriverTest, ins_name_test);
  FRIEND_TEST(InsDriverTest, init_parser_test);
  FRIEND_TEST(InsDriverTest, init_input_test);
  #endif
};

}  // namespace airi
}  // namespace crdc

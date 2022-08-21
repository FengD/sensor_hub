// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: ins

#pragma once

#include <utility>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include "common/common.h"
#include "ins_drivers/input/input.h"
#include "ins_drivers/parser/parser.h"
#include "ins_drivers/proto/ins_config.pb.h"
#include "ins_drivers/proto/ins_component_config.pb.h"
#include "cyber/sensor_proto/ins.pb.h"

namespace crdc {
namespace airi {

class InsSensor : public common::Thread {
 public:
  explicit InsSensor(const InsComponentConfig& config);
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

  bool stop_;
  int32_t reload_;
  std::string ins_name_;
  InsComponentConfig config_;
  InsConfig ins_config_;
  std::shared_ptr<InsInput> input_;
  std::shared_ptr<InsParser> parser_;
  uint32_t sensor_position_id_;
};

}  // namespace airi
}  // namespace crdc

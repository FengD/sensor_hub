// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: lidar parser

#pragma once

#include <memory>
#include <string>

#include "common/common.h"

namespace crdc {
namespace airi {

class LidarParser {
 public:
  LidarParser() = default;
  virtual ~LidarParser() = default;
};

REGISTER_COMPONENT(LidarParser);
#define REGISTER_PARSER_INPUT(name) REGISTER_CLASS(LidarParser, name)

}  // namespace airi
}  // namespace crdc
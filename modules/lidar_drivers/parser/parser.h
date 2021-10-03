// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: input

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

}  // namespace airi
}  // namespace crdc
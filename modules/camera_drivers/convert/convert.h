// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera convert

#pragma once

#include <string>
#include "common/common.h"

namespace crdc {
namespace airi {

class Convert {
 public:
  Convert() = default;
  virtual ~Convert() = default;

  static bool yuyv_to_bgr(const unsigned char *src, unsigned char *dst,
                          const int& width, const int& height,
                          const std::string& sensor);
};

}  // namespace airi
}  // namespace crdc
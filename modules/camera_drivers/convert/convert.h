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

  /**
   * @brief Convert the image from yuyv to bgr.
   * @param The input image data
   * @param The output image data
   * @param The width of the image
   * @param The height of the image
   * @param The name of the sensor
   * @return status
   */
  static bool yuyv_to_bgr(const unsigned char *src, unsigned char *dst,
                          const int& width, const int& height,
                          const std::string& sensor);

  /**
   * @brief Convert the image from nv12 to bgr.
   * @param The input image data
   * @param The output image data
   * @param The width of the image
   * @param The height of the image
   * @param The name of the sensor
   * @return status
   */
  static bool nv12_to_bgr(const unsigned char *src, unsigned char *dst,
                          const int& width, const int& height,
                          const std::string& sensor);
};

}  // namespace airi
}  // namespace crdc

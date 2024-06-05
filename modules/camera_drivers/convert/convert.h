// Copyright (C) 2020 FengD
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

  /**
   * @brief verify if the input and image is valid.
   * @param The input image data
   * @param The output image data
   * @param The width of the image
   * @param The height of the image
   * @param The name of the sensor
   * @return status
   */
  static bool verify_image(const unsigned char *src, unsigned char *dst,
                           const int& width, const int& height,
                           const std::string& sensor) {
    if (width <= 0) {
        LOG(ERROR) << "[" << sensor << "] width input error: " << width << std::endl;
        return false;
    }

    if (height <= 0) {
        LOG(ERROR) << "[" << sensor << "] height input error: " << height << std::endl;
        return false;
    }

    if (width % 8 != 0) {
        LOG(ERROR) << "[" << sensor << "] width must be a multiple of 8: " << width << std::endl;
        return false;
    }

    if (src == nullptr) {
        LOG(ERROR) << "[" << sensor << "] src input error." << std::endl;
        return false;
    }

    if (dst == nullptr) {
        LOG(ERROR) << "[" << sensor << "] dst input error." << std::endl;
        return false;
    }
    return true;
  }
};

}  // namespace airi
}  // namespace crdc

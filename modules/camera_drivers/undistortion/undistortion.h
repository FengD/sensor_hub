// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera undistortion

#pragma once

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "common/common.h"

namespace crdc {
namespace airi {

struct CameraSensorConfig {
    std::string distortion_type_;
    int32_t img_width_ = 0;
    int32_t img_height_ = 0;
    int32_t sensor_width_ = 0;
    int32_t sensor_height_ = 0;
    int32_t offset_x_ = 0;
    int32_t offset_y_ = 0;
    std::vector<double> intrinsic_data_;
    std::vector<double> dist_coeffs_;
    CameraSensorConfig() {}
};

class Undistortion {
 public:
  Undistortion() = default;
  virtual ~Undistortion() = default;

  /**
   * @brief Init the undistortion by sensor config
   * @param sensor config
   */
  virtual bool init(const CameraSensorConfig& config) {
      return true;
  }

  /**
   * @brief Execute the main process of undistortion
   * @param input image
   * @param output image
   * @return status
   */
  virtual bool process(const cv::Mat& img, cv::Mat& img_distorted) {
      img_distorted = img;
      return true;
  }

  virtual std::string get_name() const {
      return "SkipUndistortion";
  }

 protected:
  CameraSensorConfig config_;
};

REGISTER_COMPONENT(Undistortion);
#define REGISTER_UNDISTORTION(name) REGISTER_CLASS(Undistortion, name)

}  // namespace airi
}  // namespace crdc
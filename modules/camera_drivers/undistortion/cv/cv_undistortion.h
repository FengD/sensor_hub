// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera undistortion cv

#pragma once

#include <string>
#include "camera_drivers/undistortion/undistortion.h"

namespace crdc {
namespace airi {

class CvUndistortion : public Undistortion {
 public:
  CvUndistortion() = default;
  virtual ~CvUndistortion() = default;

  bool init(const CameraSensorConfig& config) override;
  bool process(const cv::Mat& img, cv::Mat& img_distorted) override;
  std::string get_name() const override {
      return "CvUndistortion";
  }

 private:
  cv::Mat image_;
  cv::Mat image_undistorted;
  cv::Mat map_x_;
  cv::Mat map_y_;

};

}  // namespace airi
}  // namespace crdc
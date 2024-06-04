// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera encoder

#include "camera_drivers/encoder/cv_encoder/cv_encoder.h"
#include <opencv2/imgcodecs/legacy/constants_c.h>

namespace crdc {
namespace airi {

bool CvEncoder::init(const EncoderConfig& config) {
  config_ = config;
  if (!config_.has_cv_encoder_config()) {
    LOG(ERROR) << "Config not contain cv encoder config." << std::endl;
    return false;
  }

  params_.resize(3, 0);
  params_[0] = CV_IMWRITE_JPEG_QUALITY;
  params_[1] = config_.cv_encoder_config().quality();
  return true;
}

int32_t CvEncoder::encode(const cv::Mat& image, unsigned char** encode_buffer) {
  compressed_buffer_.clear();
  if (!cv::imencode(".jpg", image, compressed_buffer_, params_)) {
    LOG(ERROR) << "Failed to encode jpg format." << std::endl;
    return 0;
  }

  *encode_buffer = compressed_buffer_.data();
  return compressed_buffer_.size();
}

}  // namespace airi
}  // namespace crdc

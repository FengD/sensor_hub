// Copyright (C) 2022 FengD
// License: Modified BSD Software License Agreement
// Author: Zilou CAO
// Description: camera encoder h264

#include "camera_drivers/encoder/h264_encoder/h264_encoder.h"

namespace crdc {
namespace airi {

bool H264Encoder::init(const EncoderConfig& config) {
  config_ = config;
  if (!config_.has_h264_encoder_config()) {
    LOG(ERROR) << "Config not contain h264 encoder config." << std::endl;
    return false;
  }
  if (!(config_.h264_encoder_config().has_width() &&
          config_.h264_encoder_config().has_height())) {
    LOG(ERROR) << "Config not contain width or height.";
    return false;
  }
  encoder_set_default_prm(&prm_);
  prm_.height = config_.h264_encoder_config().height();  // image size not data size
  prm_.width = config_.h264_encoder_config().width();
  prm_.bitrate = config_.h264_encoder_config().bitrate();
  outbuf_ = new uint8_t[prm_.height * prm_.width];
  handler_ = encoder_init(&prm_);
  return true;
}

int32_t H264Encoder::encode(const cv::Mat& image, unsigned char** encode_buffer) {
  size_t max_size = prm_.height / 16 * (64 + 25 * prm_.width);
  if (encoder_do_encode(handler_, reinterpret_cast<uint8_t*>(image.data),
              outbuf_, max_size, &size_) != encode_status::ENC_OK) {
    LOG(ERROR) << "Failed to encode.";
    return 0;
  }
  *encode_buffer = reinterpret_cast<unsigned char*>(outbuf_);
  return static_cast<int32_t>(size_);
}

}  // namespace airi
}  // namespace crdc

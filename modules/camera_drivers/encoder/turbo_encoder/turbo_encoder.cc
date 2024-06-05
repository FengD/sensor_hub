// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera encoder

#include "camera_drivers/encoder/turbo_encoder/turbo_encoder.h"

namespace crdc {
namespace airi {

bool TurboEncoder::init(const EncoderConfig& config) {
  config_ = config;
  if (!config_.has_turbo_encoder_config()) {
    LOG(ERROR) << "Config not contain turbo encoder config." << std::endl;
    return false;
  }

  jpeg_compressor_ = tjInitCompress();
  jpeg_quality_ = config_.turbo_encoder_config().quality();
  return true;
}

int32_t TurboEncoder::encode(const cv::Mat& image, unsigned char** encode_buffer) {
  uint64_t jpeg_size;
  if (tjCompress2(jpeg_compressor_, image.data, image.size().width, 0,
                  image.size().height, TJPF_BGR, encode_buffer, &jpeg_size,
                  TJSAMP_420, jpeg_quality_, TJFLAG_FASTDCT) < 0) {
    LOG(ERROR) << "Failed to encode jpg format." << std::endl;
    return 0;
  }

  return static_cast<int32_t>(jpeg_size);
}

}  // namespace airi
}  // namespace crdc

// Copyright (C) 2022 FengD
// License: Modified BSD Software License Agreement
// Author: Zilou CAO
// Description: camera encoder h264

#pragma once

#include <tiovx_h264_encode.h>
#include <string>
#include <vector>
#include "common/common.h"
#include "camera_drivers/encoder/encoder.h"

namespace crdc {
namespace airi {

class H264Encoder : public Encoder {
 public:
  H264Encoder() = default;
  virtual ~H264Encoder() {
    delete[] outbuf_;
    encoder_deinit(handler_);
  }
  bool init(const EncoderConfig& config) override;

  int32_t encode(const cv::Mat& image, unsigned char** encode_buffer) override;

  std::string get_name() const override {
    return "H264Encoder";
  }

 private:
  struct tiovx_encode_param prm_;
  EncHandler handler_;
  size_t size_;
  uint8_t *outbuf_ = NULL;
};

}  // namespace airi
}  // namespace crdc

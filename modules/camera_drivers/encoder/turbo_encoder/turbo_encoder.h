// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera encoder cv

#pragma once

#include <turbojpeg.h>
#include <string>
#include <vector>
#include "common/common.h"
#include "camera_drivers/encoder/encoder.h"

namespace crdc {
namespace airi {

class TurboEncoder : public Encoder {
 public:
  TurboEncoder() = default;
  virtual ~TurboEncoder() = default;
  bool init (const EncoderConfig& config) override;

  int32_t encode(const cv::Mat& image, unsigned char** encode_buffer) override;

  std::string get_name() const override {
    return "TurboEncoder";
  }

 private:
  int32_t jpeg_quality_;
  tjhandle jpeg_compressor_;
};

}  // namespace airi
}  // namespace crdc
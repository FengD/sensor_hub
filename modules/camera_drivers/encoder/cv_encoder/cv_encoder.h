// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera encoder cv

#pragma once

#include <string>
#include <vector>
#include "common/common.h"
#include "camera_drivers/encoder/encoder.h"

namespace crdc {
namespace airi {

class CvEncoder : public Encoder {
 public:
  CvEncoder() = default;
  virtual ~CvEncoder() = default;
  bool init (const EncoderConfig& config) override;

  int32_t encode(const cv::Mat& image, unsigned char** encode_buffer) override;

  std::string get_name() const override {
    return "CvEncoder";
  }

 private:
  std::vector<int32_t> params_;
  std::vector<uint8_t> compressed_buffer_;
};

}  // namespace airi
}  // namespace crdc
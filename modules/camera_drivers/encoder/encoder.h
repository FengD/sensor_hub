// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera encoder

#pragma once

#include <string>
#include <opencv2/opencv.hpp>
#include "common/common.h"
#include "camera_drivers/proto/encoder_config.pb.h"
#include "cyber/sensor_proto/image.pb.h"

namespace crdc {
namespace airi {

class Encoder {
 public:
  Encoder() = default;
  virtual ~Encoder() = default;

  /**
   * @brief Init the encoder. It needs to be redefined for each subclass.
   * @param The encode config.
   * @return status
   */
  virtual bool init(const EncoderConfig& config) {
    return false;
  }

  /**
   * @brief The encode process. It needs to be redefined for each subclass.
   * @param The input image.
   * @param The ouput encode data.
   * @return The size of the encode data.
   */
  virtual int32_t encode(const cv::Mat& image, unsigned char** encode_buffer) {
    AERROR << "NOT IMPLEMENTED";
    return 0;
  }

  virtual std::string get_name() const = 0;

 protected:
  EncoderConfig config_;
};

REGISTER_COMPONENT(Encoder);
#define REGISTER_ENCODER(name) REGISTER_CLASS(Encoder, name)

}  // namespace airi
}  // namespace crdc

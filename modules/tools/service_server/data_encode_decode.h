/**
* Copyright (c) 2023 Beijing Jingwei Hirain Technologies Co., Inc. All rights reserved
* @file data_encode_decode.h
* @brief
* @author Jiao.He
* @date 2023-07-19
* @license Modified BSD Software License Agreement
*/
#pragma once

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace crdc {
namespace airi {
class DataEncoderDecoder {
 public:
  DataEncoderDecoder(){}
  /**
  * @brief  Convert binary data to base64 data stream format
  * @param  [data]: binary data
  * @param  [len]: binary data size
  * @return [std::string]: base64 data stream
  */
  std::string base64_encode(const unsigned char* data, size_t len);
  /**
  * @brief Convert base64 data stream to binary data
  * @param  [encoded_data]: base64 data stream
  * @return [std::vector<unsigned char>]: binary data
  */
  std::string base64_decode(const std::string& encoded_data);
  std::string convert_image_to_base64(cv::Mat &image);
};
}  // namespace airi
}  // namespace crdc

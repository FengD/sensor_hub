// Copyright (C) 2022 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Yangyang LIU

#include <gtest/gtest.h>
#include "camera_drivers/convert/convert.h"
#include <opencv2/opencv.hpp>

namespace crdc {
namespace airi {

class CameraConvertTest : public ::testing::Test {};

TEST_F(CameraConvertTest, camera_convert_nv12_to_bgr_test) {
  Convert convert_test;
  const unsigned char *src;
  unsigned char *dst;
  const int width = 50;
  const int height = 1280;
  const std::string sensor;
  EXPECT_EQ(false, convert_test.nv12_to_bgr(src, dst, width, height, sensor));
}

TEST_F(CameraConvertTest, camera_convert_image_test) {
  Convert c;
  unsigned char *src;
  unsigned char *dst;
  int width;
  int height;
  std::string config_file;
  EXPECT_EQ(false, c.verify_image(src, dst, width, height, config_file));

  width = 1920;
  height = 1080;
  cv::Mat Img_1 = cv::Mat::ones(height, width, CV_8UC1);
  cv::Mat Img_2 = cv::Mat::ones(height, width, CV_8UC3);
  src = Img_1.data;
  dst = Img_2.data;
  config_file = "./out";
  EXPECT_EQ(true, c.verify_image(src, dst, width, height, config_file));
}


TEST_F(CameraConvertTest, camera_yuyv_to_bgr_test) {
  Convert convert_test;
  const unsigned char *src;
  unsigned char *dst;
  const int width = 1920;
  const int height = 1080;
  const std::string sensor;
  EXPECT_EQ(false, convert_test.yuyv_to_bgr(src, dst, width, height, sensor));
}

}  // namespace airi
}  // namespace crdc

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

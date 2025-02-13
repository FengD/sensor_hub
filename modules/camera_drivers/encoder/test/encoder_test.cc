// Copyright (C) 2022 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING

#include <gtest/gtest.h>

#include "camera_drivers/encoder/encoder.h"
#include "camera_drivers/encoder/cv_encoder/cv_encoder.h"
#include "camera_drivers/encoder/turbo_encoder/turbo_encoder.h"
#include "camera_drivers/proto/encoder_config.pb.h"

namespace sensor {
namespace hub {

REGISTER_ENCODER(CvEncoder);
REGISTER_ENCODER(TurboEncoder);

class CameraEncoderTest : public ::testing::Test {};

// cv_encoder_test
TEST_F(CameraEncoderTest, cv_encoder_name_test) {
  CvEncoder cv_encoder_test;
  EXPECT_EQ("CvEncoder", cv_encoder_test.get_name());
}

TEST_F(CameraEncoderTest, cv_encoder_init_test) {
  CvEncoder cv_encoder_test;
  EncoderConfig config;
  EXPECT_EQ(false, cv_encoder_test.init(config));
  config.mutable_cv_encoder_config()->set_quality(75);
  EXPECT_EQ(true, cv_encoder_test.init(config));
  config.mutable_cv_encoder_config()->set_quality(0);
  EXPECT_EQ(true, cv_encoder_test.init(config));
  config.mutable_cv_encoder_config()->set_quality(-75);
  EXPECT_EQ(true, cv_encoder_test.init(config));
}

TEST_F(CameraEncoderTest, cv_encoder_encode_test) {
  CvEncoder cv_encoder_test;
  EncoderConfig config;
  unsigned char* compress_buffer;
  cv::Mat image_c1(100, 100, CV_8UC1, cv::Scalar(0));
  cv::Mat image_c3(100, 100, CV_8UC3, cv::Scalar(0, 0, 0));

  config.mutable_cv_encoder_config()->set_quality(-75);
  cv_encoder_test.init(config);
  EXPECT_EQ(458, cv_encoder_test.encode(image_c1, &compress_buffer));
  EXPECT_EQ(822, cv_encoder_test.encode(image_c3, &compress_buffer));

  config.mutable_cv_encoder_config()->set_quality(75);
  cv_encoder_test.init(config);
  EXPECT_EQ(459, cv_encoder_test.encode(image_c1, &compress_buffer));
  EXPECT_EQ(823, cv_encoder_test.encode(image_c3, &compress_buffer));

  config.mutable_cv_encoder_config()->set_quality(150);
  cv_encoder_test.init(config);
  EXPECT_EQ(460, cv_encoder_test.encode(image_c1, &compress_buffer));
  EXPECT_EQ(825, cv_encoder_test.encode(image_c3, &compress_buffer));
}

// turbo_encoder_test
TEST_F(CameraEncoderTest, turbo_encoder_name_test) {
  TurboEncoder turbo_encoder_test;
  EXPECT_EQ("TurboEncoder", turbo_encoder_test.get_name());
}

TEST_F(CameraEncoderTest, turbo_encoder_init_test) {
  TurboEncoder turbo_encoder_test;
  EncoderConfig config;
  EXPECT_EQ(false, turbo_encoder_test.init(config));
  config.mutable_turbo_encoder_config()->set_quality(75);
  EXPECT_EQ(true, turbo_encoder_test.init(config));
}

}  // namespace hub
}  // namespace sensor

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

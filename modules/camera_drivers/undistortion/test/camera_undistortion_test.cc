// Copyright (C) 2022 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: CHUNXIAO HUANG

#include <string>
#include <gtest/gtest.h>
#include "camera_drivers/undistortion/cv/cv_undistortion.h"
#include "camera_drivers/undistortion/undistortion.h"
#include <opencv2/opencv.hpp>

namespace crdc {
namespace airi {
REGISTER_UNDISTORTION(Undistortion);
REGISTER_UNDISTORTION(CvUndistortion);

class CvUndistortionTest : public ::testing::Test {};

TEST_F(CvUndistortionTest, undistortion_name_test) {
  Undistortion s;
  EXPECT_EQ("SkipUndistortion", s.get_name());
}

TEST_F(CvUndistortionTest, cv_undistortion_name_test) {
  CvUndistortion s;
  EXPECT_EQ("CvUndistortion", s.get_name());
}

TEST_F(CvUndistortionTest, undistortion_init_test) {
  Undistortion s;
  CameraSensorConfig config;
  EXPECT_EQ(true, s.init(config));
}


TEST_F(CvUndistortionTest, cv_undistortion_init_test) {
  CvUndistortion s;
  CameraSensorConfig config;
  EXPECT_EQ(true, s.init(config));
}

TEST_F(CvUndistortionTest, undistortion_process_test) {
  Undistortion s;
  cv::Mat img;
  cv::Mat img_distorted;
  EXPECT_EQ(true, s.process(img, img_distorted));
}

TEST_F(CvUndistortionTest, cv_undistortion_process_test) {
  CvUndistortion s;
  cv::Mat img;
  cv::Mat img_distorted;
  EXPECT_EQ(true, s.process(img, img_distorted));
}


}  // namespace airi
}  // namespace crdc

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

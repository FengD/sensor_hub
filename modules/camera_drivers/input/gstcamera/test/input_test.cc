// Copyright (C) 2022 FengD
// License: Modified BSD Software License Agreement
// Author: CHUNXIAO HUANG

#include <gtest/gtest.h>
#include "camera_drivers/input/input.h"
#include "camera_drivers/input/gstcamera/gstcamera.h"
#include "camera_drivers/proto/camera_config.pb.h"
#include "camera_drivers/proto/camera_component_config.pb.h"
#include <opencv2/opencv.hpp>

namespace crdc {
namespace airi {

REGISTER_CAMERA_INPUT(CameraInput);
REGISTER_CAMERA_INPUT(GstCamera);

class GstCameraInputTest : public ::testing::Test {};

TEST_F(GstCameraInputTest, camera_init_test) {
  CameraInput s;
  CameraInputConfig config;
  EXPECT_EQ(false, s.init(config));
}

TEST_F(GstCameraInputTest, gstcamera_init_test) {
  GstCamera *s = new GstCamera();
  CameraInputConfig config;
  EXPECT_EQ(false, s->init(config));
}

TEST_F(GstCameraInputTest, gstcamera_camera_init1_test) {
  GstCamera *s = new GstCamera();
  EXPECT_EQ(false, s->camera_init());
}

TEST_F(GstCameraInputTest, gstcamera_camera_camera_start_test) {
  GstCamera *s = new GstCamera();
  EXPECT_EQ(false, s->camera_start());
}

TEST_F(GstCameraInputTest, gstcamera_camera_camera_stop_test) {
  GstCamera *s = new GstCamera();
  EXPECT_EQ(false, s->camera_stop());
}

TEST_F(GstCameraInputTest, gstcamera_camera_start_test) {
  GstCamera *s = new GstCamera();
  EXPECT_EQ(false, s->start());
}

TEST_F(GstCameraInputTest, gstcamera_camera_stop_test) {
  GstCamera *s = new GstCamera();
  EXPECT_EQ(false, s->stop());
}

}  // namespace airi
}  // namespace crdc

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
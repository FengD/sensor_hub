// Copyright (C) 2022 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING

#include <gtest/gtest.h>

#include "camera_drivers/camera.h"
#include "camera_drivers/decoder/decoder.h"
#include "camera_drivers/encoder/cv_encoder/cv_encoder.h"
#include "camera_drivers/encoder/encoder.h"
#include "camera_drivers/encoder/turbo_encoder/turbo_encoder.h"
#include "camera_drivers/input/input.h"
#include "camera_drivers/input/testing/testing.h"
#include "camera_drivers/proto/camera_component_config.pb.h"
#include "camera_drivers/proto/camera_config.pb.h"
#include "camera_drivers/proto/encoder_config.pb.h"
#include "camera_drivers/undistortion/cv/cv_undistortion.h"
#include "camera_drivers/undistortion/undistortion.h"
#include "cyber/sensor_proto/image.pb.h"

namespace crdc {
namespace airi {
REGISTER_ENCODER(CvEncoder);
REGISTER_ENCODER(TurboEncoder);

REGISTER_UNDISTORTION(CvUndistortion);
REGISTER_UNDISTORTION(Undistortion);

REGISTER_CAMERA_INPUT(TestingCamera);
class CameraDriverTest : public ::testing::Test {};

TEST_F(CameraDriverTest, camera_name_test) {
  setenv("CRDC_WS", ".", 1);
  CameraComponent config;
  CameraComponentConfig config2;
  std::string config_file_path =
      "../../../../modules/camera_drivers/params/drivers/camera/gtest/camera_config.prototxt";
  crdc::airi::util::get_proto_from_file(config_file_path, &config);
  config2 = config.component_config()[0];
  config2.set_config_file(
      "../../../../modules/camera_drivers/params/drivers/camera/gtest/camera_front.prototxt");
  Camera s(config2);
  EXPECT_EQ("CAMERA_FRONT", s.get_name());
}

TEST_F(CameraDriverTest, init_encoder_test) {
  setenv("CRDC_WS", ".", 1);
  CameraComponentConfig config;
  Camera s(config);
  CameraConfig config2;
  std::string config_file_path =
      "../../../../modules/camera_drivers/params/drivers/camera/gtest/camera_front.prototxt";
  crdc::airi::util::get_proto_from_file(config_file_path, &config2);
  s.camera_config_ = config2;
  EXPECT_EQ(true, s.init_encoder());
  EXPECT_EQ(Image2_Compression_JPEG, s.proto_encode_image_->compression());
}

TEST_F(CameraDriverTest, init_input_test) {
  setenv("CRDC_WS", ".", 1);
  CameraComponentConfig config;
  Camera s(config);
  EXPECT_EQ(true, s.init_input());
}

TEST_F(CameraDriverTest, init_undistortion_test) {
  setenv("CRDC_WS", ".", 1);
  CameraComponentConfig config;
  Camera s(config);
  EXPECT_EQ(true, s.init_undistortion());
}

TEST_F(CameraDriverTest, dill_encode_image_test) {
  setenv("CRDC_WS", ".", 1);
  CameraComponentConfig config;
  Camera s(config);
  CameraConfig config2;
  std::string config_file_path =
      "../../../../modules/camera_drivers/params/drivers/camera/gtest/camera_front.prototxt";
  crdc::airi::util::get_proto_from_file(config_file_path, &config2);
  s.camera_config_ = config2;
  auto raw_data = std::make_shared<CameraRawData>();
  raw_data->image_ = cv::Mat(s.camera_config_.input_config().height(),
                             s.camera_config_.input_config().width(), CV_8UC3);
  raw_data->data_type = "Y";
  auto offset = 0;
  s.init_input();
  s.init_encoder();
  EXPECT_EQ(true, s.dill_encode_image(raw_data, offset));
  EXPECT_EQ(raw_data->utime_, s.proto_encode_image_->header().camera_timestamp());
  EXPECT_EQ(0, s.proto_encode_image_->header().sequence_num());
  EXPECT_EQ(raw_data->exposure_time_, s.proto_encode_image_->exposuretime());
  EXPECT_EQ(raw_data->data_type, s.proto_encode_image_->type());
}

} // namespace airi
} // namespace crdc

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

// Copyright (C) 2022 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Zilou Cao

#include <gtest/gtest.h>
#include <stdlib.h>
#include <string>
#include <opencv2/opencv.hpp>
#include "cyber/sensor_proto/image.pb.h"
#include "camera_drivers/output/cyber_output.h"

namespace crdc {
namespace airi {

void construct_image(const int &width, const int &height, std::shared_ptr<Image2> &image_data) {
  cv::Mat image(height, width, CV_8UC3);
  image_data->set_data(image.data, width * height);
  image_data->set_width(width);
  image_data->set_height(height);
}

class CameraCyberOutputTest : public ::testing::Test {};

TEST_F(CameraCyberOutputTest, cyber_output_init_test) {
  CameraCyberOutput c;
  apollo::cyber::Init("MODULE");
  std::string name = "test_node";
  EXPECT_EQ(true, c.init(name));
  EXPECT_EQ("test_node", c.node_->Name());
  setenv("MACHINE_SUBCODE", "with_machine_subscode", 1);
  c.init(name);
  EXPECT_EQ("test_node_with_machine_subscode", c.node_->Name());
}

TEST_F(CameraCyberOutputTest, cyber_output_write_image_test) {
  CameraCyberOutput c;
  c.init("test_node");
  std::shared_ptr<Image2> test_image(new Image2);
  construct_image(1920, 1080, test_image);
  EXPECT_EQ(true, c.write_image("camera_topic", test_image));
  EXPECT_EQ("CameraDriver", test_image->mutable_header()->module_name());
}

}  // namespace airi
}  // namespace crdc

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

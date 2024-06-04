#include <gtest/gtest.h>
#include <gflags/gflags.h>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <fstream>
#include <chrono>

#include "common/common.h"
#include "camera_drivers/proto/camera_diagnose_config.pb.h"
#include "camera_drivers/camera_diagnose/camera_diagnose.h"
#include <opencv2/opencv.hpp>

#define INIT_POINTER(datatype, data, num)            \
  data = (datatype *)malloc(sizeof(datatype) * num); \
  memset(data, 0, sizeof(datatype) * num);

namespace crdc {
namespace airi {
class CameraDiagnoseTest : public ::testing::Test {};

void read_config(const std::string &config_path, CameraDiagnoseConfig& config) {
  if (!crdc::airi::util::is_path_exists(config_path)) {
    LOG(FATAL) << "[TEST] Path of " << config_path << " not exists.";
    return;
  }
  if (!crdc::airi::util::get_proto_from_file(config_path, &config)) {
    LOG(FATAL) << "[TEST] Failed to get proto from " << config_path << ".";
    return;
  }
  LOG(INFO) << config.DebugString();
}

std::vector<cv::Mat> read_images_in_folder(cv::String pattern) {
  std::vector<cv::String> fn;
  cv::glob(pattern, fn, false);
  std::vector<cv::Mat> images;
  size_t count = fn.size();  // number of png files in images folder

  for (size_t i = 0; i < count; ++i) {
    images.emplace_back(cv::imread(fn[i]));
  }
  return images;
}

void classify_an_image(cv::Mat& image, CameraDiagnoseConfig& diagnose_config,
                       DeviceStatus& device_status, std::string& diagnose_description) {
  cv::Mat gray_img;
  cv::cvtColor(image, gray_img, cv::COLOR_BGR2GRAY);

  CameraDiagnoser diagnoser(diagnose_config);
  diagnoser.diagnose(image, gray_img);

}


TEST_F(CameraDiagnoseTest, camera_diagnose_green_screen_test) {
  std::string config_path =
      "../../../../../modules/camera_drivers/params/drivers/camera/HH03-4/"
      "camera_diagnose_config.prototxt";
  CameraDiagnoseConfig diagnose_config;
  read_config(config_path, diagnose_config);
  std::string image_path = "../../../../../modules/camera_drivers/camera_diagnose/test/data/green_screen.png";
  cv::Mat image = cv::imread(image_path);
  DeviceStatus device_status;
  std::string diagnose_description = "";
  CameraDiagnoser diagnoser(diagnose_config);
  diagnoser.diagnose_an_image(image);
  device_status = diagnoser.get_device_status();
  diagnose_description = diagnoser.get_status_description();
  EXPECT_EQ(device_status, DeviceStatus::CAMERA_DATA_GREEN);
  EXPECT_EQ(diagnose_description, "camera_raw_image_is_all_green(pixel_are_all_zero)");
}

TEST_F(CameraDiagnoseTest, camera_diagnose_dark_test) {
  std::string config_path =
      "../../../../../modules/camera_drivers/params/drivers/camera/HH03-4/"
      "camera_diagnose_config.prototxt";
  CameraDiagnoseConfig diagnose_config;
  read_config(config_path, diagnose_config);
  std::string image_path = "../../../../../modules/camera_drivers/camera_diagnose/test/data/dark.png";
  cv::Mat image = cv::imread(image_path);
  DeviceStatus device_status;
  std::string diagnose_description = "";
  CameraDiagnoser diagnoser(diagnose_config);
  diagnoser.diagnose_an_image(image);
  device_status = diagnoser.get_device_status();
  diagnose_description = diagnoser.get_status_description();
  EXPECT_EQ(device_status, DeviceStatus::CAMERA_IMAGE_DARK);
  EXPECT_EQ(diagnose_description, "camera_is_being_exposed_to_extremely_weak_light");
}

TEST_F(CameraDiagnoseTest, camera_diagnose_faucla_test) {
  std::string config_path =
      "../../../../../modules/camera_drivers/params/drivers/camera/HH03-4/"
      "camera_diagnose_config.prototxt";
  CameraDiagnoseConfig diagnose_config;
  read_config(config_path, diagnose_config);
  std::string image_path = "../../../../../modules/camera_drivers/camera_diagnose/test/data/facula.png";
  cv::Mat image = cv::imread(image_path);
  DeviceStatus device_status;
  std::string diagnose_description = "";
  CameraDiagnoser diagnoser(diagnose_config);
  diagnoser.diagnose_an_image(image);
  device_status = diagnoser.get_device_status();
  diagnose_description = diagnoser.get_status_description();
  EXPECT_EQ(device_status, DeviceStatus::CAMERA_STRONG_BACKLIGHT);
  EXPECT_EQ(diagnose_description, "camera_is_being_exposed_to_extremely_strong_light");
}

TEST_F(CameraDiagnoseTest, camera_diagnose_blocked_test) {
  std::string config_path =
      "../../../../../modules/camera_drivers/params/drivers/camera/HH03-4/"
      "camera_diagnose_config.prototxt";
  CameraDiagnoseConfig diagnose_config;
  read_config(config_path, diagnose_config);
  std::string image_path = "../../../../../modules/camera_drivers/camera_diagnose/test/data/blocked.png";
  cv::Mat image = cv::imread(image_path);
  DeviceStatus device_status;
  std::string diagnose_description = "";
  CameraDiagnoser diagnoser(diagnose_config);
  diagnoser.diagnose_an_image(image);
  device_status = diagnoser.get_device_status();
  diagnose_description = diagnoser.get_status_description();
  EXPECT_EQ(device_status, DeviceStatus::CAMERA_WINDOW_DIRTY_BLOCKED);
  EXPECT_EQ(diagnose_description, "camera_is_being_blocked_by_other_objects");
}

TEST_F(CameraDiagnoseTest, camera_diagnose_blur_test) {
  std::string config_path =
      "../../../../../modules/camera_drivers/params/drivers/camera/HH03-4/"
      "camera_diagnose_config.prototxt";
  CameraDiagnoseConfig diagnose_config;
  read_config(config_path, diagnose_config);
  std::string image_path = "../../../../../modules/camera_drivers/camera_diagnose/test/data/blur.png";
  cv::Mat image = cv::imread(image_path);
  DeviceStatus device_status;
  std::string diagnose_description = "";
  CameraDiagnoser diagnoser(diagnose_config);
  diagnoser.diagnose_an_image(image);
  device_status = diagnoser.get_device_status();
  diagnose_description = diagnoser.get_status_description();
  EXPECT_EQ(device_status, DeviceStatus::CAMERA_IMAGE_BLUR);
  EXPECT_EQ(diagnose_description, "camera_image_data_is_blurry");
}

}  // namespace airi
}  // namespace crdc

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

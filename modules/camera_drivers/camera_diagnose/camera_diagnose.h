#pragma once

#include <memory>
#include <unordered_map>
#include <string>
#include "common/common.h"
#include "camera_drivers/input/input.h"
#include "camera_drivers/camera_diagnose/facula_detector.h"
#include "camera_drivers/camera_diagnose/green_screen_detector.h"
#include "camera_drivers/camera_diagnose/dark_detector.h"
#include "camera_drivers/camera_diagnose/blocked_detector.h"
#include "camera_drivers/camera_diagnose/blur_detector.h"

namespace crdc {
namespace airi {

enum CameraDiagnoseStatus : std::uint8_t {
  GREEN_SCREEN,
  FACULA,
  DARK,
  BLOCKED,
  BLUR
};

class CameraDiagnoser {
 public:
  explicit CameraDiagnoser(const CameraDiagnoseConfig& cfg);
  void diagnose(const cv::Mat& image, const cv::Mat& gray);
  void diagnose_an_image(const cv::Mat& raw_data_bgr);
  std::unordered_map<int, int> get_diagnose_results_map() { return diagnose_results_; }
  DeviceStatus get_device_status() { return device_status_; }
  std::string get_status_description() { return status_description_; }

 private:
  void green_screen_diagnose(const cv::Mat& image, const cv::Mat& gray);
  void facula_diagnose(const cv::Mat& image, const cv::Mat& gray);
  void dark_diagnose(const cv::Mat& image, const cv::Mat& gray);
  void blocked_diagnose(const cv::Mat& image, const cv::Mat& gray);
  void blur_diagnose(const cv::Mat& image, const cv::Mat& gray);
  void yuv_to_bgr(const cv::Mat& yuv_img, cv::Mat& bgr_img,
                  const uint32_t& height, const uint32_t& width);
  /**
   * @brief The function is used to diagnose an image data
   * @param the camera raw data
   * @param the address offset
   * @param the diagnostics of camera data
  */
  void initialize_status_mappings();
  void reset_diagnose_results();
  std::unique_ptr<GreenScreenDetector> green_screen_detector_;
  std::unique_ptr<FaculaDetector> facula_detector_;
  std::unique_ptr<DarkDetector> dark_detector_;
  std::unique_ptr<BlockedDetector> blocked_detector_;
  std::unique_ptr<BlurDetector> blur_detector_;
  cv::Mat img_;
  DeviceStatus device_status_;
  std::string status_description_;
  std::unordered_map<int, std::string> status_description_map_;
  std::unordered_map<int, DeviceStatus> device_status_map_;
  std::unordered_map<int, int> diagnose_results_;
};
}  // namespace airi
}  // namespace crdc
